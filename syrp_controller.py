import threading
import time
import signal
import sys
import traceback

import dbus
import dbus.mainloop.glib
import subprocess

from lcd_display import Display
from buttons import ButtonHandler

dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

bus_pt  = dbus.SystemBus()
bus_lin = dbus.SystemBus()

PAN_TILT     = "/org/bluez/hci1/dev_D8_A0_1D_59_2D_05"
LINEAR       = "/org/bluez/hci1/dev_D8_A0_1D_64_1B_E5"
ADAPTER      = "/org/bluez/hci1"
ADAPTER_MAC  = "10:5A:95:63:39:23"
ADAPTER_HCI  = "hci1"
PAN_TILT_MAC = "D8:A0:1D:59:2D:05"
LINEAR_MAC   = "D8:A0:1D:64:1B:E5"
KEEPALIVE_CMD           = bytes.fromhex("00042805000000")
KEEPALIVE_SUPPRESS_SECS = 0.3
CMD_CHAR                = "/service0028/char002c"

last_frame_sent = {PAN_TILT: 0.0, LINEAR: 0.0}

# ================================================================
# State
# ================================================================

class State:
    MAIN_MENU     = 'main_menu'
    SETUP         = 'setup'
    EDIT_FRAMES   = 'edit_frames'
    EDIT_INTERVAL = 'edit_interval'
    RUNNING       = 'running'
    PAUSED        = 'paused'
    RESTARTING    = 'restarting'

state         = State.MAIN_MENU
menu_index    = 0
ble_ready     = False
cfg_frames    = 100
cfg_interval  = 5
current_frame = 0
pause_event   = threading.Event()
stop_event    = threading.Event()
pause_event.set()

display = Display()

# ================================================================
# BLE helpers
# ================================================================

def get_char(bus, device_path, char_path):
    return dbus.Interface(
        bus.get_object("org.bluez", device_path + char_path),
        "org.bluez.GattCharacteristic1"
    )

def write_char(bus, device_path, char_path, data):
    char = get_char(bus, device_path, char_path)
    char.WriteValue(
        dbus.Array([dbus.Byte(b) for b in data], signature='y'),
        dbus.Dictionary({"type": dbus.String("request")}, signature='sv')
    )

def start_notify(bus, device_path, char_path):
    try:
        get_char(bus, device_path, char_path).StartNotify()
    except:
        pass

def is_connected(bus, device_path):
    try:
        props = dbus.Interface(bus.get_object("org.bluez", device_path),
                               "org.freedesktop.DBus.Properties")
        return bool(props.Get("org.bluez.Device1", "Connected"))
    except:
        return False

def wait_for_adapter(timeout=15):
    om = dbus.Interface(bus_pt.get_object("org.bluez", "/"),
                        "org.freedesktop.DBus.ObjectManager")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ADAPTER in om.GetManagedObjects():
            return True
        time.sleep(0.5)
    return False

def reset_adapter():
    subprocess.run(["sudo", "hciconfig", ADAPTER_HCI, "down"], check=False)
    time.sleep(1)
    for mac in [PAN_TILT_MAC, LINEAR_MAC]:
        subprocess.run(["sudo", "rm", "-f",
            f"/var/lib/bluetooth/{ADAPTER_MAC}/cache/{mac}"], check=False)
        subprocess.run(["sudo", "rm", "-f",
            f"/var/lib/bluetooth/{ADAPTER_MAC}/{mac}/attributes"], check=False)
    time.sleep(1)
    subprocess.run(["sudo", "hciconfig", ADAPTER_HCI, "up"], check=False)
    time.sleep(5)
    deadline = time.time() + 15
    while time.time() < deadline:
        try:
            props = dbus.Interface(
                bus_pt.get_object("org.bluez", ADAPTER),
                "org.freedesktop.DBus.Properties"
            )
            if props.Get("org.bluez.Adapter1", "Powered"):
                return True
        except:
            pass
        time.sleep(0.5)
    return False

def scan_until_found(bus, device_mac, timeout=15):
    adapter = dbus.Interface(bus.get_object("org.bluez", ADAPTER),
                             "org.bluez.Adapter1")
    device_path = ADAPTER + "/dev_" + device_mac.replace(":", "_")
    om = dbus.Interface(bus.get_object("org.bluez", "/"),
                        "org.freedesktop.DBus.ObjectManager")
    adapter.StartDiscovery()
    deadline = time.time() + timeout
    found = False
    while time.time() < deadline:
        if device_path in om.GetManagedObjects():
            found = True
            break
        time.sleep(0.5)
    adapter.StopDiscovery()
    return found

def connect_device(bus, device_path, name):
    device = dbus.Interface(bus.get_object("org.bluez", device_path),
                            "org.bluez.Device1")
    props  = dbus.Interface(bus.get_object("org.bluez", device_path),
                            "org.freedesktop.DBus.Properties")
    try:
        device.Connect()
    except:
        return False
    for _ in range(80):
        if props.Get("org.bluez.Device1", "ServicesResolved"):
            start_notify(bus, device_path, "/service0028/char0029")
            start_notify(bus, device_path, "/service0028/char002e")
            try:
                write_char(bus, device_path, "/service0028/char0031",
                           KEEPALIVE_CMD)
                time.sleep(0.06)
                write_char(bus, device_path, "/service0028/char0031",
                           KEEPALIVE_CMD)
            except:
                pass
            return True
        time.sleep(0.1)
    return False

def connect_with_retry(bus, device_path, name, device_mac, attempts=5):
    for attempt in range(attempts):
        if attempt > 0:
            display.show_message(
                "Connecting...",
                f"{name}",
                f"Attempt {attempt+1}/{attempts}",
                "Please wait..."
            )
            time.sleep(5)
        if not scan_until_found(bus, device_mac):
            continue
        if connect_device(bus, device_path, name):
            return True
    return False

def reconnect_if_needed(bus, device_path, name, device_mac):
    if is_connected(bus, device_path):
        return True
    success = connect_with_retry(bus, device_path, name, device_mac)
    if success:
        time.sleep(1)
    return success

def disconnect_all():
    for bus, path in [(bus_pt, PAN_TILT), (bus_lin, LINEAR)]:
        try:
            dbus.Interface(bus.get_object("org.bluez", path),
                           "org.bluez.Device1").Disconnect()
        except:
            pass

def send_frame(frame):
    command = bytes([0x00, 0x0c, 0x28, 0x17, 0x00, 0x00, 0x02,
                     frame & 0xff, (frame >> 8) & 0xff,
                     0x00, 0x00, 0x01, 0x00, 0x00, 0x00])
    results = {}

    def send_pt():
        try:
            write_char(bus_pt, PAN_TILT, CMD_CHAR, command)
            last_frame_sent[PAN_TILT] = time.time()
            results['pt'] = True
        except:
            results['pt'] = False

    def send_lin():
        try:
            write_char(bus_lin, LINEAR, CMD_CHAR, command)
            last_frame_sent[LINEAR] = time.time()
            results['lin'] = True
        except:
            results['lin'] = False

    threads = [threading.Thread(target=send_pt),
               threading.Thread(target=send_lin)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return results

def wait_and_keepalive(deadline):
    while True:
        remaining = deadline - time.time()
        if remaining <= 0.05:
            break
        now = time.time()
        for bus, dev_path in ((bus_pt, PAN_TILT), (bus_lin, LINEAR)):
            if now - last_frame_sent[dev_path] < KEEPALIVE_SUPPRESS_SECS:
                continue
            def _write(b=bus, p=dev_path):
                try:
                    write_char(b, p, CMD_CHAR, KEEPALIVE_CMD)
                except:
                    pass
            threading.Thread(target=_write, daemon=True).start()
        remaining = deadline - time.time()
        time.sleep(min(0.18, max(0, remaining - 0.02)))

def return_to_start():
    return_cmd = bytes.fromhex("000c28170000020000000001000000")
    for bus, path in [(bus_pt, PAN_TILT), (bus_lin, LINEAR)]:
        try:
            write_char(bus, path, CMD_CHAR, return_cmd)
        except:
            pass

# ================================================================
# Sequence thread
# ================================================================

def run_sequence():
    global state, current_frame
    try:
        while not stop_event.is_set():
            current_frame += 1
            if current_frame > cfg_frames:
                display.show_message(
                    "Complete!",
                    f"All {cfg_frames} frames done",
                    "Returning to start",
                    "Please wait..."
                )
                return_to_start()
                time.sleep(3)
                set_state(State.MAIN_MENU)
                return

            reconnect_if_needed(
                bus_pt, PAN_TILT, "Pan/Tilt", PAN_TILT_MAC)
            reconnect_if_needed(
                bus_lin, LINEAR, "Linear", LINEAR_MAC)

            send_frame(current_frame)
            frame_deadline = time.time() + cfg_interval

            while time.time() < frame_deadline:
                if stop_event.is_set():
                    break
                if not pause_event.is_set():
                    while not pause_event.is_set():
                        if stop_event.is_set():
                            break
                        time.sleep(0.1)
                    frame_deadline = time.time() + cfg_interval
                secs_left = max(0, int(frame_deadline - time.time()))
                display.show_progress(
                    current_frame, cfg_frames, cfg_interval, secs_left)
                wait_and_keepalive(min(frame_deadline, time.time() + 0.2))

        display.show_message(
            "Stopped",
            f"Frame {current_frame}/{cfg_frames}",
            "Returning to start",
            "Please wait..."
        )
        return_to_start()
        time.sleep(3)
        set_state(State.MAIN_MENU)

    except Exception as e:
        display.show_message(
            "Error!",
            str(e)[:20],
            str(e)[20:40],
            "Check terminal"
        )
        print(f"run_sequence exception: {e}")
        traceback.print_exc()
        set_state(State.MAIN_MENU)

# ================================================================
# Menu rendering
# ================================================================

def render():
    if state == State.MAIN_MENU:
        items = ["Setup", "Run"]
        if ble_ready:
            items.append("Disconnect")
        display.show_menu("Syrp Controller", items, menu_index)

    elif state == State.SETUP:
        display.show_menu(
            "Setup",
            [f"Frames:   {cfg_frames:>5}",
             f"Interval: {cfg_interval:>4}s",
             "< Back"],
            menu_index
        )

    elif state == State.EDIT_FRAMES:
        display.show_edit("Setup", "Frames", cfg_frames, "frames")

    elif state == State.EDIT_INTERVAL:
        display.show_edit("Setup", "Interval", cfg_interval, "secs")

    elif state == State.PAUSED:
        display.show_menu(
            f"PAUSED  {current_frame}/{cfg_frames}",
            ["Resume", "Restart", "Stop"],
            menu_index
        )

    elif state == State.RESTARTING:
        display.show_message(
            "Restarting...",
            "Returning to start",
            "Please wait...", ""
        )

# ================================================================
# State transitions
# ================================================================

sequence_thread = None

def set_state(new_state):
    global state, menu_index
    state = new_state
    menu_index = 0
    render()

def do_return_and_restart():
    global current_frame
    set_state(State.RESTARTING)
    return_to_start()
    time.sleep(3)
    current_frame = 0
    do_run()

def do_run():
    global sequence_thread, stop_event, pause_event, current_frame, ble_ready
    if ble_ready:
        if not is_connected(bus_pt, PAN_TILT) or \
           not is_connected(bus_lin, LINEAR):
            display.show_message(
                "Connection lost",
                "Reconnecting...", "", "")
            ble_ready = False
            threading.Thread(target=ble_connect, daemon=True).start()
            return
    current_frame = 0
    stop_event  = threading.Event()
    pause_event = threading.Event()
    pause_event.set()
    set_state(State.RUNNING)
    sequence_thread = threading.Thread(target=run_sequence, daemon=True)
    sequence_thread.start()

def ble_connect():
    global ble_ready
    display.show_message(
        "Connecting...", "Resetting adapter", "Please wait...", "")
    if not reset_adapter():
        display.show_message("Error", "Adapter not found", "", "")
        time.sleep(3)
        set_state(State.MAIN_MENU)
        return
    display.show_message("Connecting...", "Pan/Tilt...", "", "")
    if not connect_with_retry(
            bus_pt, PAN_TILT, "Pan/Tilt", PAN_TILT_MAC):
        display.show_message("Error", "Pan/Tilt failed", "", "")
        time.sleep(3)
        set_state(State.MAIN_MENU)
        return
    display.show_message("Connecting...", "Linear...", "", "")
    if not connect_with_retry(
            bus_lin, LINEAR, "Linear", LINEAR_MAC):
        display.show_message("Error", "Linear failed", "", "")
        time.sleep(3)
        set_state(State.MAIN_MENU)
        return
    ble_ready = True
    display.show_message("Connected!", "Both devices ready", "", "")
    time.sleep(1)
    do_run()

# ================================================================
# Button handler
# ================================================================

def on_button(btn, step=1):
    global state, menu_index, cfg_frames, cfg_interval, ble_ready

    if state == State.MAIN_MENU:
        max_index = 2 if ble_ready else 1
        if btn == 'up':
            menu_index = max(0, menu_index - 1)
        elif btn in ('down', 'right'):
            menu_index = min(max_index, menu_index + 1)
        elif btn == 'centre':
            if menu_index == 0:
                set_state(State.SETUP)
            elif menu_index == 1:
                if ble_ready:
                    do_run()
                else:
                    threading.Thread(
                        target=ble_connect, daemon=True).start()
            elif menu_index == 2 and ble_ready:
                disconnect_all()
                ble_ready = False
                display.show_message("Disconnected", "", "", "")
                time.sleep(1)
                set_state(State.MAIN_MENU)
        render()

    elif state == State.SETUP:
        if btn == 'up':
            menu_index = max(0, menu_index - 1)
        elif btn == 'down':
            menu_index = min(2, menu_index + 1)
        elif btn == 'centre':
            if menu_index == 0:
                set_state(State.EDIT_FRAMES)
            elif menu_index == 1:
                set_state(State.EDIT_INTERVAL)
            elif menu_index == 2:
                set_state(State.MAIN_MENU)
        elif btn == 'left':
            set_state(State.MAIN_MENU)
        render()

    elif state == State.EDIT_FRAMES:
        if btn == 'up':
            cfg_frames += step
        elif btn == 'down':
            cfg_frames = max(1, cfg_frames - step)
        elif btn in ('centre', 'left'):
            set_state(State.SETUP)
        render()

    elif state == State.EDIT_INTERVAL:
        if btn == 'up':
            cfg_interval += step
        elif btn == 'down':
            cfg_interval = max(1, cfg_interval - step)
        elif btn in ('centre', 'left'):
            set_state(State.SETUP)
        render()

    elif state == State.RUNNING:
        if btn in ('centre', 'right'):
            pause_event.clear()
            set_state(State.PAUSED)

    elif state == State.PAUSED:
        if btn == 'up':
            menu_index = max(0, menu_index - 1)
        elif btn == 'down':
            menu_index = min(2, menu_index + 1)
        elif btn == 'centre':
            if menu_index == 0:
                pause_event.set()
                set_state(State.RUNNING)
            elif menu_index == 1:
                stop_event.set()
                pause_event.set()
                threading.Thread(
                    target=do_return_and_restart, daemon=True).start()
            elif menu_index == 2:
                stop_event.set()
                pause_event.set()
        render()

# ================================================================
# Main
# ================================================================

def cleanup(signum=None, frame=None):
    display.show_message("Shutting down...", "", "", "")
    if sequence_thread and sequence_thread.is_alive():
        stop_event.set()
        pause_event.set()
        sequence_thread.join(timeout=5)
    disconnect_all()
    display.clear()
    sys.exit(0)

signal.signal(signal.SIGINT,  cleanup)
signal.signal(signal.SIGTERM, cleanup)

buttons = ButtonHandler(on_button)
render()

print("Syrp Controller running. Press Ctrl+C to exit.")
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    cleanup()
