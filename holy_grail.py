#!/usr/bin/env python3
"""
Holy Grail Timelapse Controller
Integrates Syrp BLE motion control with gphoto2 camera control
and automatic exposure ramping for dawn/dusk timelapses.
"""

import threading
import time
import signal
import sys
import traceback
import subprocess
import math
import os

import dbus
import dbus.mainloop.glib

from lcd_display import Display
from buttons import ButtonHandler

dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

bus = dbus.SystemBus()

# ================================================================
# BLE Device config
# ================================================================

PAN_TILT     = "/org/bluez/hci0/dev_D8_A0_1D_59_2D_05"
LINEAR       = "/org/bluez/hci0/dev_D8_A0_1D_64_1B_E5"
ADAPTER      = "/org/bluez/hci0"
ADAPTER_MAC  = "88:A2:9E:B2:05:B0"
ADAPTER_HCI  = "hci0"
PAN_TILT_MAC = "D8:A0:1D:59:2D:05"
LINEAR_MAC   = "D8:A0:1D:64:1B:E5"
KEEPALIVE_CMD           = bytes.fromhex("00042805000000")
KEEPALIVE_INTERVAL_SECS = 0.18
CMD_CHAR                = "/service0028/char002c"

# ================================================================
# Camera config
# ================================================================

JPEG_PATH   = "/tmp/hg_frame.jpg"
GPHOTO2_CMD = ["gphoto2", "--capture-image-and-download", "--keep",
               "--filename", JPEG_PATH, "--force-overwrite"]

# ================================================================
# Session config (user settable via menu)
# ================================================================

cfg_frames        = 200       # total frames
cfg_rest          = 5         # fixed rest after shutter closes (secs)
cfg_wobble        = 0.5       # stabilisation delay after BLE move (secs)
cfg_max_tv        = 20.0      # max shutter speed before ISO rises (secs)
cfg_max_iso       = 6400      # ISO ceiling
cfg_ev_threshold  = 0.33      # EV drift before correction applied
cfg_drift_ev      = -1.5      # total creative EV drift over session (neg=D->N)
cfg_direction     = 'D->N'    # 'D->N' or 'N->D'

# Shutter speed steps — standard 1/3 stop ladder (seconds)
TV_STEPS = [
    1/8000, 1/6400, 1/5000, 1/4000, 1/3200, 1/2500, 1/2000,
    1/1600, 1/1250, 1/1000, 1/800,  1/640,  1/500,  1/400,
    1/320,  1/250,  1/200,  1/160,  1/125,  1/100,  1/80,
    1/60,   1/50,   1/40,   1/30,   1/25,   1/20,   1/15,
    1/13,   1/10,   1/8,    1/6,    1/5,    1/4,    0.3,
    0.4,    0.5,    0.6,    0.8,    1.0,    1.3,    1.6,
    2.0,    2.5,    3.2,    4.0,    5.0,    6.0,    8.0,
    10.0,   13.0,   15.0,   20.0,   25.0,   30.0
]

# ISO steps — standard 1/3 stop ladder
ISO_STEPS = [64, 80, 100, 125, 160, 200, 250, 320, 400, 500,
             640, 800, 1000, 1250, 1600, 2000, 2500, 3200,
             4000, 5000, 6400, 8000, 10000, 12800]

# ================================================================
# State
# ================================================================

class State:
    MAIN_MENU     = 'main_menu'
    SETUP         = 'setup'
    EDIT_FRAMES   = 'edit_frames'
    EDIT_REST     = 'edit_rest'
    EDIT_WOBBLE   = 'edit_wobble'
    EDIT_MAX_TV   = 'edit_max_tv'
    EDIT_MAX_ISO  = 'edit_max_iso'
    EDIT_DRIFT    = 'edit_drift'
    EDIT_DIR      = 'edit_dir'
    RUNNING       = 'running'
    PAUSED        = 'paused'
    RESTARTING    = 'restarting'

state         = State.MAIN_MENU
menu_index    = 0
ble_ready     = False
current_frame = 0
pause_event   = threading.Event()
stop_event    = threading.Event()
pause_event.set()

# Exposure state (set from first frame EXIF)
base_tv       = None
base_iso      = None
current_tv    = None
current_iso   = None

display = Display()

# ================================================================
# Exposure helpers
# ================================================================

def tv_to_str(tv_secs):
    if tv_secs is None:
        return "?"
    if tv_secs >= 1.0:
        return f"{tv_secs:.1f}s"
    else:
        return f"1/{int(round(1/tv_secs))}"

def nearest_tv(tv_secs):
    return min(TV_STEPS, key=lambda x: abs(x - tv_secs))

def nearest_iso(iso):
    return min(ISO_STEPS, key=lambda x: abs(x - iso))

def tv_index(tv_secs):
    return TV_STEPS.index(nearest_tv(tv_secs))

def iso_index(iso):
    return ISO_STEPS.index(nearest_iso(iso))

def s_curve(x):
    """S-curve mapping 0..1 -> 0..1, slow-fast-slow."""
    return (math.tanh((x - 0.5) * 4) + math.tanh(2)) / (2 * math.tanh(2))

def creative_drift_ev(frame, total_frames):
    """Returns the cumulative creative EV offset for this frame."""
    if total_frames <= 1:
        return 0.0
    t = (frame - 1) / (total_frames - 1)
    curve = s_curve(t)
    drift = cfg_drift_ev * curve
    if cfg_direction == 'N->D':
        drift = -drift
    return drift

def apply_exposure_correction(tv, iso, correction_ev):
    """
    Apply correction_ev stops of exposure change.
    Positive = more exposure. Adjust Tv first, spill into ISO.
    Returns (new_tv, new_iso, actual_ev_applied).
    """
    max_tv_allowed  = min(cfg_max_tv, max(TV_STEPS))
    max_iso_allowed = min(cfg_max_iso, max(ISO_STEPS))

    ti = tv_index(tv)
    ii = iso_index(iso)
    stops_remaining = correction_ev
    actual = 0.0

    # Adjust Tv first
    while abs(stops_remaining) >= 0.16:
        if stops_remaining > 0:
            if ti < len(TV_STEPS) - 1 and TV_STEPS[ti + 1] <= max_tv_allowed:
                ti += 1
                stops_remaining -= 1/3
                actual += 1/3
            else:
                break
        else:
            if ti > 0:
                ti -= 1
                stops_remaining += 1/3
                actual -= 1/3
            else:
                break

    # Spill into ISO
    while abs(stops_remaining) >= 0.16:
        if stops_remaining > 0:
            if ii < len(ISO_STEPS) - 1 and ISO_STEPS[ii + 1] <= max_iso_allowed:
                ii += 1
                stops_remaining -= 1/3
                actual += 1/3
            else:
                break
        else:
            if ii > 0:
                ii -= 1
                stops_remaining += 1/3
                actual -= 1/3
            else:
                break

    return TV_STEPS[ti], ISO_STEPS[ii], actual

def set_camera_exposure(tv, iso):
    """Set shutter speed and ISO on Z8 via gphoto2."""
    if tv >= 1.0:
        tv_str = f"{tv:.6g}"
    else:
        denom = int(round(1/tv))
        tv_str = f"1/{denom}"

    for config, val in [("shutterspeed", tv_str), ("iso", str(iso))]:
        r = subprocess.run(
            ["gphoto2", "--set-config", f"{config}={val}"],
            capture_output=True, text=True)
        if r.returncode != 0:
            print(f"set_camera_exposure {config}={val} failed: {r.stderr.strip()}", flush=True)

def capture_and_read_exif():
    """Fire shutter, download JPEG, read EXIF.
    Returns (ev_diff, tv_secs, iso) or None on failure."""
    if os.path.exists(JPEG_PATH):
        os.remove(JPEG_PATH)

    r = subprocess.run(GPHOTO2_CMD, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"capture failed: {r.stderr.strip()}", flush=True)
        return None

    if not os.path.exists(JPEG_PATH):
        print("capture: no file downloaded", flush=True)
        return None

    r = subprocess.run(
        ["exiftool", "-ExposureDifference", "-ExposureTime", "-ISO",
         "-csv", JPEG_PATH],
        capture_output=True, text=True)

    if r.returncode != 0:
        print(f"exiftool failed: {r.stderr.strip()}", flush=True)
        return None

    try:
        lines = r.stdout.strip().split('\n')
        data  = lines[1].split(',')
        ev_diff = float(data[1])
        tv_str  = data[2].strip()
        if '/' in tv_str:
            num, den = tv_str.split('/')
            tv = float(num) / float(den)
        else:
            tv = float(tv_str)
        iso = int(float(data[3]))
        return ev_diff, tv, iso
    except Exception as e:
        print(f"exiftool parse error: {e} raw: {r.stdout}", flush=True)
        return None

# ================================================================
# BLE helpers
# ================================================================

def get_char(device_path, char_path):
    return dbus.Interface(
        bus.get_object("org.bluez", device_path + char_path),
        "org.bluez.GattCharacteristic1"
    )

def write_char(device_path, char_path, data):
    char = get_char(device_path, char_path)
    char.WriteValue(
        dbus.Array([dbus.Byte(b) for b in data], signature='y'),
        dbus.Dictionary({"type": dbus.String("request")}, signature='sv')
    )

def start_notify(device_path, char_path):
    try:
        get_char(device_path, char_path).StartNotify()
    except Exception as e:
        print(f"StartNotify failed: {e}", flush=True)

def is_connected(device_path):
    try:
        props = dbus.Interface(bus.get_object("org.bluez", device_path),
                               "org.freedesktop.DBus.Properties")
        return bool(props.Get("org.bluez.Device1", "Connected"))
    except:
        return False

def reset_adapter():
    subprocess.run(["sudo", "hciconfig", ADAPTER_HCI, "down"], check=False)
    time.sleep(2)
    subprocess.run(["sudo", "hciconfig", ADAPTER_HCI, "up"], check=False)
    time.sleep(5)
    deadline = time.time() + 15
    while time.time() < deadline:
        try:
            props = dbus.Interface(
                bus.get_object("org.bluez", ADAPTER),
                "org.freedesktop.DBus.Properties"
            )
            if props.Get("org.bluez.Adapter1", "Powered"):
                return True
        except:
            pass
        time.sleep(0.5)
    return False

def scan_until_found(device_mac, timeout=15):
    adapter = dbus.Interface(bus.get_object("org.bluez", ADAPTER),
                             "org.bluez.Adapter1")
    device_path = ADAPTER + "/dev_" + device_mac.replace(":", "_")
    om = dbus.Interface(bus.get_object("org.bluez", "/"),
                        "org.freedesktop.DBus.ObjectManager")
    try:
        adapter.StartDiscovery()
    except:
        return False
    deadline = time.time() + timeout
    found = False
    while time.time() < deadline:
        if device_path in om.GetManagedObjects():
            found = True
            break
        time.sleep(0.5)
    try:
        adapter.StopDiscovery()
    except:
        pass
    return found

def connect_device(device_path, name):
    device = dbus.Interface(bus.get_object("org.bluez", device_path),
                            "org.bluez.Device1")
    props  = dbus.Interface(bus.get_object("org.bluez", device_path),
                            "org.freedesktop.DBus.Properties")
    try:
        device.Connect()
    except:
        return False
    for i in range(80):
        try:
            if props.Get("org.bluez.Device1", "ServicesResolved"):
                start_notify(device_path, "/service0028/char0029")
                start_notify(device_path, "/service0028/char002e")
                try:
                    write_char(device_path, "/service0028/char0031", KEEPALIVE_CMD)
                    time.sleep(0.06)
                    write_char(device_path, "/service0028/char0031", KEEPALIVE_CMD)
                except:
                    pass
                return True
        except:
            pass
        time.sleep(0.1)
    return False

def connect_with_retry(device_path, name, device_mac, attempts=5):
    for attempt in range(attempts):
        if attempt > 0:
            display.show_message("Connecting...", name,
                f"Attempt {attempt+1}/{attempts}", "Please wait...")
            time.sleep(5)
        if not scan_until_found(device_mac):
            continue
        if connect_device(device_path, name):
            return True
    return False

def disconnect_all():
    for path in [PAN_TILT, LINEAR]:
        try:
            dbus.Interface(bus.get_object("org.bluez", path),
                           "org.bluez.Device1").Disconnect()
        except:
            pass

def send_frame(frame):
    command = bytes([0x00, 0x0c, 0x28, 0x17, 0x00, 0x00, 0x02,
                     frame & 0xff, (frame >> 8) & 0xff,
                     0x00, 0x00, 0x01, 0x00, 0x00, 0x00])
    def send_pt():
        try:
            write_char(PAN_TILT, CMD_CHAR, command)
        except Exception as e:
            print(f"send_frame pt failed: {e}", flush=True)
    def send_lin():
        try:
            write_char(LINEAR, CMD_CHAR, command)
        except Exception as e:
            print(f"send_frame lin failed: {e}", flush=True)
    threads = [threading.Thread(target=send_pt),
               threading.Thread(target=send_lin)]
    for t in threads: t.start()
    for t in threads: t.join()

keepalive_active = threading.Event()

def keepalive_loop():
    while not stop_event.is_set():
        if keepalive_active.is_set():
            for path in (PAN_TILT, LINEAR):
                try:
                    write_char(path, CMD_CHAR, KEEPALIVE_CMD)
                except:
                    pass
        time.sleep(KEEPALIVE_INTERVAL_SECS)

def return_to_start():
    return_cmd = bytes.fromhex("000c28170000020000000001000000")
    for path in [PAN_TILT, LINEAR]:
        try:
            write_char(path, CMD_CHAR, return_cmd)
        except:
            pass

# ================================================================
# Main sequence
# ================================================================

def run_sequence():
    global state, current_frame, base_tv, base_iso, current_tv, current_iso

    keepalive_active.set()

    try:
        while not stop_event.is_set():
            current_frame += 1
            if current_frame > cfg_frames:
                display.show_message("Complete!",
                    f"All {cfg_frames} frames done",
                    "Returning to start", "Please wait...")
                keepalive_active.clear()
                return_to_start()
                time.sleep(3)
                set_state(State.MAIN_MENU)
                return

            # Reconnect BLE if needed
            for path, name, mac in [
                (PAN_TILT, "Pan/Tilt", PAN_TILT_MAC),
                (LINEAR,   "Linear",   LINEAR_MAC)
            ]:
                if not is_connected(path):
                    keepalive_active.clear()
                    display.show_message("Reconnecting...", name, "", "")
                    connect_with_retry(path, name, mac)
                    keepalive_active.set()

            # Step 1: BLE frame advance
            send_frame(current_frame)

            # Step 2: Wobble delay — let camera settle after move
            time.sleep(cfg_wobble)

            # Step 3: Fire shutter and read EXIF
            display.show_message(
                f"Frame {current_frame}/{cfg_frames}",
                f"Tv:{tv_to_str(current_tv)}  ISO:{current_iso or '?'}",
                "Capturing...", "")

            result = capture_and_read_exif()

            if result is None:
                print(f"Frame {current_frame}: capture failed, skipping", flush=True)
            else:
                ev_diff, exif_tv, exif_iso = result
                print(f"Frame {current_frame}: ev_diff={ev_diff:+.2f} "
                      f"tv={tv_to_str(exif_tv)} iso={exif_iso}", flush=True)

                # Lock base exposure from first successful frame
                if base_tv is None:
                    base_tv     = exif_tv
                    base_iso    = exif_iso
                    current_tv  = exif_tv
                    current_iso = exif_iso
                    print(f"Base locked: Tv={tv_to_str(base_tv)} ISO={base_iso}", flush=True)

                # Scene correction: push ev_diff back toward zero
                # ev_diff negative = scene darker than metered = need more exposure
                scene_correction = 0.0
                if abs(ev_diff) >= cfg_ev_threshold:
                    scene_correction = -ev_diff

                # Creative drift step for this frame
                drift_step = 0.0
                if current_frame > 1:
                    drift_step = (creative_drift_ev(current_frame, cfg_frames) -
                                  creative_drift_ev(current_frame - 1, cfg_frames))

                total_correction = scene_correction + drift_step

                if abs(total_correction) >= 0.16:
                    new_tv, new_iso, applied = apply_exposure_correction(
                        current_tv, current_iso, total_correction)
                    if new_tv != current_tv or new_iso != current_iso:
                        current_tv  = new_tv
                        current_iso = new_iso
                        print(f"Frame {current_frame}: "
                              f"scene={scene_correction:+.2f} "
                              f"drift={drift_step:+.2f} "
                              f"total={total_correction:+.2f} "
                              f"-> Tv={tv_to_str(current_tv)} ISO={current_iso}",
                              flush=True)
                        set_camera_exposure(current_tv, current_iso)

            # Step 4: Fixed rest interval
            rest_deadline = time.time() + cfg_rest
            while time.time() < rest_deadline:
                if stop_event.is_set():
                    break
                if not pause_event.is_set():
                    while not pause_event.is_set():
                        if stop_event.is_set():
                            break
                        time.sleep(0.1)
                    rest_deadline = time.time() + cfg_rest
                secs_left = max(0, int(rest_deadline - time.time()))
                cumulative_drift = creative_drift_ev(current_frame, cfg_frames)
                display.show_message(
                    f"Frame {current_frame}/{cfg_frames}",
                    f"Tv:{tv_to_str(current_tv)}  ISO:{current_iso or '?'}",
                    f"Drift:{cumulative_drift:+.2f}EV",
                    f"Rest: {secs_left}s")
                time.sleep(0.5)

        # Stopped by user
        display.show_message("Stopped",
            f"Frame {current_frame}/{cfg_frames}",
            "Returning to start", "Please wait...")
        keepalive_active.clear()
        return_to_start()
        time.sleep(3)
        set_state(State.MAIN_MENU)

    except Exception as e:
        keepalive_active.clear()
        display.show_message("Error!", str(e)[:20], str(e)[20:40], "Check terminal")
        print(f"run_sequence exception: {e}", flush=True)
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
        display.show_menu("Holy Grail", items, menu_index)

    elif state == State.SETUP:
        display.show_menu("Setup", [
            f"Frames:   {cfg_frames:>5}",
            f"Rest:     {cfg_rest:>4}s",
            f"Wobble:   {cfg_wobble:>4}s",
            f"Max Tv:   {cfg_max_tv:>4}s",
            f"Max ISO:{cfg_max_iso:>6}",
            f"Drift:  {cfg_drift_ev:>+5.1f}EV",
            f"Dir:    {cfg_direction}",
            "< Back"
        ], menu_index)

    elif state == State.EDIT_FRAMES:
        display.show_edit("Setup", "Frames", cfg_frames, "frames")
    elif state == State.EDIT_REST:
        display.show_edit("Setup", "Rest interval", cfg_rest, "secs")
    elif state == State.EDIT_WOBBLE:
        display.show_edit("Setup", "Wobble time", cfg_wobble, "secs")
    elif state == State.EDIT_MAX_TV:
        display.show_edit("Setup", "Max shutter", cfg_max_tv, "secs")
    elif state == State.EDIT_MAX_ISO:
        display.show_edit("Setup", "Max ISO", cfg_max_iso, "")
    elif state == State.EDIT_DRIFT:
        display.show_edit("Setup", "Creative drift", cfg_drift_ev, "EV")
    elif state == State.EDIT_DIR:
        display.show_menu("Direction", ["D->N", "N->D"], menu_index)

    elif state == State.PAUSED:
        display.show_menu(
            f"PAUSED {current_frame}/{cfg_frames}",
            ["Resume", "Stop"], menu_index)

    elif state == State.RESTARTING:
        display.show_message("Restarting...",
            "Returning to start", "Please wait...", "")

# ================================================================
# State machine
# ================================================================

sequence_thread  = None
keepalive_thread = None

def set_state(new_state):
    global state, menu_index
    state = new_state
    menu_index = 0
    render()

def do_run():
    global sequence_thread, keepalive_thread, stop_event, pause_event
    global current_frame, ble_ready, base_tv, base_iso, current_tv, current_iso

    if ble_ready:
        if not is_connected(PAN_TILT) or not is_connected(LINEAR):
            display.show_message("Connection lost", "Reconnecting...", "", "")
            ble_ready = False
            threading.Thread(target=ble_connect, daemon=True).start()
            return

    current_frame = 0
    base_tv       = None
    base_iso      = None
    current_tv    = None
    current_iso   = None
    stop_event    = threading.Event()
    pause_event   = threading.Event()
    pause_event.set()
    keepalive_active.clear()

    set_state(State.RUNNING)

    keepalive_thread = threading.Thread(target=keepalive_loop, daemon=True)
    keepalive_thread.start()

    sequence_thread = threading.Thread(target=run_sequence, daemon=True)
    sequence_thread.start()

def ble_connect():
    global ble_ready
    display.show_message("Connecting...", "Resetting adapter", "Please wait...", "")
    if not reset_adapter():
        display.show_message("Error", "Adapter not found", "", "")
        time.sleep(3)
        set_state(State.MAIN_MENU)
        return
    display.show_message("Connecting...", "Pan/Tilt...", "", "")
    if not connect_with_retry(PAN_TILT, "Pan/Tilt", PAN_TILT_MAC):
        display.show_message("Error", "Pan/Tilt failed", "", "")
        time.sleep(3)
        set_state(State.MAIN_MENU)
        return
    display.show_message("Connecting...", "Linear...", "", "")
    if not connect_with_retry(LINEAR, "Linear", LINEAR_MAC):
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
    global state, menu_index, ble_ready
    global cfg_frames, cfg_rest, cfg_wobble, cfg_max_tv
    global cfg_max_iso, cfg_drift_ev, cfg_direction

    if state == State.MAIN_MENU:
        max_idx = 2 if ble_ready else 1
        if btn == 'up':
            menu_index = max(0, menu_index - 1)
        elif btn in ('down', 'right'):
            menu_index = min(max_idx, menu_index + 1)
        elif btn == 'centre':
            if menu_index == 0:
                set_state(State.SETUP)
            elif menu_index == 1:
                if ble_ready:
                    do_run()
                else:
                    threading.Thread(target=ble_connect, daemon=True).start()
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
            menu_index = min(7, menu_index + 1)
        elif btn == 'centre':
            targets = [State.EDIT_FRAMES, State.EDIT_REST, State.EDIT_WOBBLE,
                       State.EDIT_MAX_TV, State.EDIT_MAX_ISO, State.EDIT_DRIFT,
                       State.EDIT_DIR, State.MAIN_MENU]
            set_state(targets[menu_index])
        elif btn == 'left':
            set_state(State.MAIN_MENU)
        render()

    elif state == State.EDIT_FRAMES:
        if btn == 'up':     cfg_frames += step
        elif btn == 'down': cfg_frames = max(1, cfg_frames - step)
        elif btn in ('centre', 'left'): set_state(State.SETUP)
        render()

    elif state == State.EDIT_REST:
        if btn == 'up':     cfg_rest += 1
        elif btn == 'down': cfg_rest = max(1, cfg_rest - 1)
        elif btn in ('centre', 'left'): set_state(State.SETUP)
        render()

    elif state == State.EDIT_WOBBLE:
        if btn == 'up':     cfg_wobble = round(cfg_wobble + 0.25, 2)
        elif btn == 'down': cfg_wobble = max(0.25, round(cfg_wobble - 0.25, 2))
        elif btn in ('centre', 'left'): set_state(State.SETUP)
        render()

    elif state == State.EDIT_MAX_TV:
        tv_vals = [1, 2, 4, 6, 8, 10, 15, 20, 25, 30]
        idx = min(range(len(tv_vals)), key=lambda i: abs(tv_vals[i] - cfg_max_tv))
        if btn == 'up':     idx = min(len(tv_vals) - 1, idx + 1)
        elif btn == 'down': idx = max(0, idx - 1)
        cfg_max_tv = tv_vals[idx]
        if btn in ('centre', 'left'): set_state(State.SETUP)
        render()

    elif state == State.EDIT_MAX_ISO:
        iso_vals = [400, 800, 1600, 3200, 6400, 12800]
        idx = min(range(len(iso_vals)), key=lambda i: abs(iso_vals[i] - cfg_max_iso))
        if btn == 'up':     idx = min(len(iso_vals) - 1, idx + 1)
        elif btn == 'down': idx = max(0, idx - 1)
        cfg_max_iso = iso_vals[idx]
        if btn in ('centre', 'left'): set_state(State.SETUP)
        render()

    elif state == State.EDIT_DRIFT:
        if btn == 'up':     cfg_drift_ev = round(cfg_drift_ev + 0.5, 1)
        elif btn == 'down': cfg_drift_ev = round(cfg_drift_ev - 0.5, 1)
        elif btn in ('centre', 'left'): set_state(State.SETUP)
        render()

    elif state == State.EDIT_DIR:
        if btn in ('up', 'down'):
            cfg_direction = 'N->D' if cfg_direction == 'D->N' else 'D->N'
        elif btn in ('centre', 'left'):
            set_state(State.SETUP)
        render()

    elif state == State.RUNNING:
        if btn in ('centre', 'right'):
            pause_event.clear()
            set_state(State.PAUSED)

    elif state == State.PAUSED:
        if btn == 'up':     menu_index = max(0, menu_index - 1)
        elif btn == 'down': menu_index = min(1, menu_index + 1)
        elif btn == 'centre':
            if menu_index == 0:
                pause_event.set()
                set_state(State.RUNNING)
            elif menu_index == 1:
                stop_event.set()
                pause_event.set()
        render()

# ================================================================
# Main
# ================================================================

def cleanup(signum=None, frame=None):
    display.show_message("Shutting down...", "", "", "")
    stop_event.set()
    pause_event.set()
    keepalive_active.clear()
    if sequence_thread and sequence_thread.is_alive():
        sequence_thread.join(timeout=5)
    disconnect_all()
    display.clear()
    sys.exit(0)

signal.signal(signal.SIGINT,  cleanup)
signal.signal(signal.SIGTERM, cleanup)

buttons = ButtonHandler(on_button)
render()

print("Holy Grail Controller running. Press Ctrl+C to exit.", flush=True)
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    cleanup()
