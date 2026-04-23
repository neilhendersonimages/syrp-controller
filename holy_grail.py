#!/usr/bin/env python3
"""
Holy Grail Timelapse Controller
GPIO shutter trigger + gphoto2 EXIF read + Syrp BLE motion control

Frame timing:
  total wall time = wobble + Tv + rest + overhead
  cfg_interval = wobble + rest + overhead (Tv is additional)
  rest = whatever remains of cfg_interval after wobble + overhead
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
import RPi.GPIO as GPIO

from lcd_display import Display
from buttons import ButtonHandler

dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()

# ================================================================
# Hardware config
# ================================================================

SHUTTER_PIN = 5
SHUTTER_MS  = 200

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SHUTTER_PIN, GPIO.OUT, initial=GPIO.LOW)

# ================================================================
# BLE config
# ================================================================

PAN_TILT     = "/org/bluez/hci0/dev_D8_A0_1D_59_2D_05"
LINEAR       = "/org/bluez/hci0/dev_D8_A0_1D_64_1B_E5"
ADAPTER      = "/org/bluez/hci0"
ADAPTER_MAC  = "88:A2:9E:B2:05:B0"
ADAPTER_HCI  = "hci0"
PAN_TILT_MAC = "D8:A0:1D:59:2D:05"
LINEAR_MAC   = "D8:A0:1D:64:1B:E5"
KEEPALIVE_CMD           = bytes.fromhex("00042805000000")
KEEPALIVE_SUPPRESS_SECS = 0.3
CMD_CHAR                = "/service0028/char002c"

last_frame_sent = {PAN_TILT: 0.0, LINEAR: 0.0}

# ================================================================
# Camera config
# ================================================================

JPEG_PATH   = "/tmp/hg_frame.jpg"
CARD_FOLDER = None
LAST_FILE_N = None

# ================================================================
# Session config
# ================================================================

cfg_frames        = 200
cfg_interval      = 5      # target interval excl. Tv (secs)
cfg_wobble        = 0.5
cfg_max_tv        = 20.0
cfg_max_iso       = 6400
cfg_ev_threshold  = 0.33
cfg_drift_ev      = -1.5
cfg_direction     = 'D->N'

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

ISO_STEPS = [64, 80, 100, 125, 160, 200, 250, 320, 400, 500,
             640, 800, 1000, 1250, 1600, 2000, 2500, 3200,
             4000, 5000, 6400, 8000, 10000, 12800]

# ================================================================
# State
# ================================================================

class State:
    MAIN_MENU    = 'main_menu'
    SETUP        = 'setup'
    EDIT_FRAMES  = 'edit_frames'
    EDIT_INT     = 'edit_interval'
    EDIT_WOBBLE  = 'edit_wobble'
    EDIT_MAX_TV  = 'edit_max_tv'
    EDIT_MAX_ISO = 'edit_max_iso'
    EDIT_DRIFT   = 'edit_drift'
    EDIT_DIR     = 'edit_dir'
    RUNNING      = 'running'
    PAUSED       = 'paused'

state         = State.MAIN_MENU
menu_index    = 0
ble_ready     = False
current_frame = 0
pause_event   = threading.Event()
stop_event    = threading.Event()
pause_event.set()

base_tv     = None
base_iso    = None
current_tv  = None
current_iso = None

display = Display()

# ================================================================
# BLE helpers — verbatim from syrp_controller.py
# ================================================================

def get_char(bus, device_path, char_path):
    return dbus.Interface(
        bus.get_object("org.bluez", device_path + char_path),
        "org.bluez.GattCharacteristic1")

def write_char(bus, device_path, char_path, data):
    char = get_char(bus, device_path, char_path)
    char.WriteValue(
        dbus.Array([dbus.Byte(b) for b in data], signature='y'),
        dbus.Dictionary({"type": dbus.String("request")}, signature='sv'))

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
    om = dbus.Interface(bus.get_object("org.bluez", "/"),
                        "org.freedesktop.DBus.ObjectManager")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ADAPTER in om.GetManagedObjects():
            return True
        time.sleep(0.5)
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
                "org.freedesktop.DBus.Properties")
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
            display.show_message("Connecting...", name,
                f"Attempt {attempt+1}/{attempts}", "Please wait...")
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
    results = {}

    def send_pt():
        try:
            write_char(bus, PAN_TILT, CMD_CHAR, command)
            last_frame_sent[PAN_TILT] = time.time()
            results['pt'] = True
        except Exception as e:
            results['pt'] = False
            print(f"send_frame pt failed: {e}", flush=True)

    def send_lin():
        try:
            write_char(bus, LINEAR, CMD_CHAR, command)
            last_frame_sent[LINEAR] = time.time()
            results['lin'] = True
        except Exception as e:
            results['lin'] = False
            print(f"send_frame lin failed: {e}", flush=True)

    threads = [threading.Thread(target=send_pt),
               threading.Thread(target=send_lin)]
    for t in threads: t.start()
    for t in threads: t.join()
    return results

def wait_and_keepalive(deadline):
    """Wait until deadline sending keepalives on same thread."""
    while True:
        remaining = deadline - time.time()
        if remaining <= 0.05:
            break
        now = time.time()
        for dev_path in (PAN_TILT, LINEAR):
            if now - last_frame_sent[dev_path] < KEEPALIVE_SUPPRESS_SECS:
                continue
            try:
                write_char(bus, dev_path, CMD_CHAR, KEEPALIVE_CMD)
            except:
                pass
        remaining = deadline - time.time()
        time.sleep(min(0.18, max(0, remaining - 0.02)))

def return_to_start():
    return_cmd = bytes.fromhex("000c28170000020000000001000000")
    for path in [PAN_TILT, LINEAR]:
        try:
            write_char(bus, path, CMD_CHAR, return_cmd)
        except:
            pass

# ================================================================
# Camera helpers
# ================================================================

def fire_shutter():
    GPIO.output(SHUTTER_PIN, GPIO.HIGH)
    time.sleep(SHUTTER_MS / 1000)
    GPIO.output(SHUTTER_PIN, GPIO.LOW)
    print("fire_shutter: fired", flush=True)

def detect_card_folder():
    global CARD_FOLDER
    r = subprocess.run(["gphoto2", "--list-folders"],
                       capture_output=True, text=True, timeout=15)
    if r.returncode != 0:
        return False
    folders = []
    for line in r.stdout.split('\n'):
        line = line.strip()
        if line.startswith('- '):
            name = line[2:].strip()
            if 'NCZ_8' in name or 'NZ_8' in name:
                folders.append(name)
    if not folders:
        return False
    CARD_FOLDER = '/store_00020001/DCIM/' + sorted(folders)[-1]
    print(f"card folder: {CARD_FOLDER}", flush=True)
    return True

def get_last_file_number():
    try:
        r = subprocess.run(
            ["gphoto2", "--list-files", "--folder", CARD_FOLDER],
            capture_output=True, text=True, timeout=15)
        if r.returncode != 0:
            return None
        lines = [l for l in r.stdout.split('\n') if l.strip().startswith('#')]
        if not lines:
            return None
        return int(lines[-1].split()[0].lstrip('#'))
    except:
        return None

def download_last_jpeg():
    """Download the next new JPEG from the card."""
    global LAST_FILE_N
    try:
        r = subprocess.run(
            ["gphoto2", "--list-files", "--folder", CARD_FOLDER],
            capture_output=True, text=True, timeout=15)
        if r.returncode != 0:
            print(f"list-files failed: {r.stderr.strip()}", flush=True)
            return False

        lines = [l.strip() for l in r.stdout.split('\n')
                 if l.strip().startswith('#')]
        new_jpegs = []
        for line in lines:
            parts = line.split()
            try:
                n    = int(parts[0].lstrip('#'))
                name = parts[1]
                if n > (LAST_FILE_N or 0) and name.upper().endswith('.JPG'):
                    new_jpegs.append((n, name))
            except:
                continue

        if not new_jpegs:
            # Fall back to last JPEG on card
            all_jpegs = [(int(l.split()[0].lstrip('#')), l.split()[1])
                         for l in lines
                         if l.split()[1].upper().endswith('.JPG')
                         if len(l.split()) > 1]
            if all_jpegs:
                new_jpegs = [all_jpegs[-1]]
            else:
                print("no JPEG found on card", flush=True)
                return False

        file_n, file_name = new_jpegs[0]
        if os.path.exists(JPEG_PATH):
            os.remove(JPEG_PATH)

        r = subprocess.run(
            ["gphoto2", "--get-file", str(file_n),
             "--folder", CARD_FOLDER,
             "--filename", JPEG_PATH, "--force-overwrite"],
            capture_output=True, text=True, timeout=20)

        if r.returncode != 0 or not os.path.exists(JPEG_PATH):
            print(f"download failed: {r.stderr.strip()}", flush=True)
            return False

        LAST_FILE_N = file_n
        print(f"downloaded {file_name} (#{file_n})", flush=True)
        return True

    except subprocess.TimeoutExpired:
        print("download_last_jpeg: timeout", flush=True)
        return False
    except Exception as e:
        print(f"download_last_jpeg error: {e}", flush=True)
        return False

def read_exif():
    """Read exposure EXIF from downloaded JPEG."""
    try:
        r = subprocess.run(
            ["exiftool", "-ExposureDifference", "-ExposureTime", "-ISO",
             "-csv", JPEG_PATH],
            capture_output=True, text=True, timeout=10)
        if r.returncode != 0:
            return None
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
    except subprocess.TimeoutExpired:
        print("read_exif: timeout", flush=True)
        return None
    except Exception as e:
        print(f"read_exif error: {e}", flush=True)
        return None

def set_camera_exposure(tv, iso):
    if tv >= 1.0:
        tv_str = f"{tv:.6g}"
    else:
        tv_str = f"1/{int(round(1/tv))}"
    for config, val in [("shutterspeed", tv_str), ("iso", str(iso))]:
        try:
            r = subprocess.run(
                ["gphoto2", "--set-config", f"{config}={val}"],
                capture_output=True, text=True, timeout=10)
            if r.returncode != 0:
                print(f"set {config} failed: {r.stderr.strip()}", flush=True)
        except subprocess.TimeoutExpired:
            print(f"set {config}: timeout", flush=True)

# ================================================================
# Exposure helpers
# ================================================================

def tv_to_str(tv_secs):
    if tv_secs is None:
        return "?"
    if tv_secs >= 1.0:
        return f"{tv_secs:.1f}s"
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
    return (math.tanh((x - 0.5) * 4) + math.tanh(2)) / (2 * math.tanh(2))

def creative_drift_ev(frame, total_frames):
    if total_frames <= 1:
        return 0.0
    t = (frame - 1) / (total_frames - 1)
    drift = cfg_drift_ev * s_curve(t)
    return -drift if cfg_direction == 'N->D' else drift

def apply_exposure_correction(tv, iso, correction_ev):
    ti = tv_index(tv)
    ii = iso_index(iso)
    stops   = correction_ev
    actual  = 0.0
    max_tv  = min(cfg_max_tv, max(TV_STEPS))
    max_iso = min(cfg_max_iso, max(ISO_STEPS))

    while abs(stops) >= 0.16:
        if stops > 0:
            if ti < len(TV_STEPS)-1 and TV_STEPS[ti+1] <= max_tv:
                ti += 1; stops -= 1/3; actual += 1/3
            else: break
        else:
            if ti > 0:
                ti -= 1; stops += 1/3; actual -= 1/3
            else: break

    while abs(stops) >= 0.16:
        if stops > 0:
            if ii < len(ISO_STEPS)-1 and ISO_STEPS[ii+1] <= max_iso:
                ii += 1; stops -= 1/3; actual += 1/3
            else: break
        else:
            if ii > 0:
                ii -= 1; stops += 1/3; actual -= 1/3
            else: break

    return TV_STEPS[ti], ISO_STEPS[ii], actual

# ================================================================
# Main sequence
# ================================================================

def run_sequence():
    global state, current_frame, base_tv, base_iso, current_tv, current_iso

    try:
        while not stop_event.is_set():
            current_frame += 1
            if current_frame > cfg_frames:
                display.show_message("Complete!",
                    f"All {cfg_frames} frames done",
                    "Returning to start", "Please wait...")
                return_to_start()
                time.sleep(3)
                set_state(State.MAIN_MENU)
                return

            # Reconnect if needed
            reconnect_if_needed(bus, PAN_TILT, "Pan/Tilt", PAN_TILT_MAC)
            reconnect_if_needed(bus, LINEAR,   "Linear",   LINEAR_MAC)

            # Step 1: BLE frame advance
            send_frame(current_frame)

            # Step 2: Wobble — keepalive runs here
            wait_and_keepalive(time.time() + cfg_wobble)

            # Step 3: Start interval timer NOW — everything from here
            # (shutter + Tv wait + download + EXIF + set exposure + rest)
            # must fit within cfg_interval. Tv is on top of this.
            interval_end = time.time() + cfg_interval

            # Step 4: Fire shutter
            display.show_message(
                f"Frame {current_frame}/{cfg_frames}",
                f"Tv:{tv_to_str(current_tv)}  ISO:{current_iso or '?'}",
                "Capturing...", "")
            fire_shutter()

            # Step 5: Wait for exposure to complete + card write buffer
            # Keepalive runs during this wait
            tv_wait = (current_tv or 1/400) + 1.5
            wait_and_keepalive(time.time() + tv_wait)

            # Step 6: Download JPEG — send keepalive burst first
            for _ in range(3):
                for path in (PAN_TILT, LINEAR):
                    try:
                        write_char(bus, path, CMD_CHAR, KEEPALIVE_CMD)
                    except:
                        pass
                time.sleep(0.05)

            if download_last_jpeg():
                result = read_exif()
            else:
                result = None

            # Step 7: Send keepalive burst after download
            for _ in range(3):
                for path in (PAN_TILT, LINEAR):
                    try:
                        write_char(bus, path, CMD_CHAR, KEEPALIVE_CMD)
                    except:
                        pass
                time.sleep(0.05)

            # Step 8: Process EXIF and update exposure
            if result is None:
                print(f"Frame {current_frame}: no EXIF", flush=True)
            else:
                ev_diff, exif_tv, exif_iso = result
                print(f"Frame {current_frame}: ev_diff={ev_diff:+.2f} "
                      f"tv={tv_to_str(exif_tv)} iso={exif_iso}", flush=True)

                if base_tv is None:
                    base_tv     = exif_tv
                    base_iso    = exif_iso
                    current_tv  = exif_tv
                    current_iso = exif_iso
                    print(f"Base: Tv={tv_to_str(base_tv)} ISO={base_iso}",
                          flush=True)

                scene_correction = 0.0
                if abs(ev_diff) >= cfg_ev_threshold:
                    scene_correction = -ev_diff

                drift_step = 0.0
                if current_frame > 1:
                    drift_step = (creative_drift_ev(current_frame, cfg_frames) -
                                  creative_drift_ev(current_frame - 1, cfg_frames))

                total_correction = scene_correction + drift_step
                if abs(total_correction) >= 0.16:
                    new_tv, new_iso, _ = apply_exposure_correction(
                        current_tv, current_iso, total_correction)
                    if new_tv != current_tv or new_iso != current_iso:
                        current_tv  = new_tv
                        current_iso = new_iso
                        print(f"Frame {current_frame}: "
                              f"scene={scene_correction:+.2f} "
                              f"drift={drift_step:+.2f} "
                              f"-> Tv={tv_to_str(current_tv)} ISO={current_iso}",
                              flush=True)
                        set_camera_exposure(current_tv, current_iso)

            # Step 9: Rest — wait out remainder of interval
            # Display countdown, keepalive runs throughout
            while True:
                remaining = interval_end - time.time()
                if remaining <= 0.05:
                    break
                if stop_event.is_set():
                    break
                if not pause_event.is_set():
                    while not pause_event.is_set():
                        if stop_event.is_set():
                            break
                        time.sleep(0.1)
                    # Reset interval on resume
                    interval_end = time.time() + cfg_interval
                secs_left = max(0, int(interval_end - time.time()))
                drift = creative_drift_ev(current_frame, cfg_frames)
                display.show_message(
                    f"Frame {current_frame}/{cfg_frames}",
                    f"Tv:{tv_to_str(current_tv)}  ISO:{current_iso or '?'}",
                    f"Drift:{drift:+.2f}EV",
                    f"Next: {secs_left}s")
                wait_and_keepalive(min(interval_end, time.time() + 0.5))

        display.show_message("Stopped",
            f"Frame {current_frame}/{cfg_frames}",
            "Returning to start", "Please wait...")
        return_to_start()
        time.sleep(3)
        set_state(State.MAIN_MENU)

    except Exception as e:
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
            f"Interval: {cfg_interval:>4}s",
            f"Wobble:   {cfg_wobble:>4}s",
            f"Max Tv:   {cfg_max_tv:>4}s",
            f"Max ISO:{cfg_max_iso:>6}",
            f"Drift:  {cfg_drift_ev:>+5.1f}EV",
            f"Dir:    {cfg_direction}",
            "< Back"
        ], menu_index)
    elif state == State.EDIT_FRAMES:
        display.show_edit("Setup", "Frames", cfg_frames, "frames")
    elif state == State.EDIT_INT:
        display.show_edit("Setup", "Interval", cfg_interval, "secs")
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

# ================================================================
# State machine
# ================================================================

sequence_thread = None

def set_state(new_state):
    global state, menu_index
    state = new_state
    menu_index = 0
    render()

def do_run():
    global sequence_thread, stop_event, pause_event
    global current_frame, ble_ready, base_tv, base_iso, current_tv, current_iso
    global LAST_FILE_N

    if ble_ready:
        if not is_connected(bus, PAN_TILT) or not is_connected(bus, LINEAR):
            display.show_message("Connection lost", "Reconnecting...", "", "")
            ble_ready = False
            threading.Thread(target=ble_connect, daemon=True).start()
            return

    display.show_message("Checking camera...", "Finding card...", "", "")
    if not detect_card_folder():
        display.show_message("Camera error", "Check USB cable", "", "")
        time.sleep(3)
        set_state(State.MAIN_MENU)
        return

    LAST_FILE_N = get_last_file_number()
    print(f"card: {CARD_FOLDER} last_file={LAST_FILE_N}", flush=True)

    current_frame = 0
    base_tv       = None
    base_iso      = None
    current_tv    = None
    current_iso   = None
    stop_event    = threading.Event()
    pause_event   = threading.Event()
    pause_event.set()

    set_state(State.RUNNING)
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
    if not connect_with_retry(bus, PAN_TILT, "Pan/Tilt", PAN_TILT_MAC):
        display.show_message("Error", "Pan/Tilt failed", "", "")
        time.sleep(3)
        set_state(State.MAIN_MENU)
        return
    display.show_message("Connecting...", "Linear...", "", "")
    if not connect_with_retry(bus, LINEAR, "Linear", LINEAR_MAC):
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
    global cfg_frames, cfg_interval, cfg_wobble, cfg_max_tv
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
            targets = [State.EDIT_FRAMES, State.EDIT_INT, State.EDIT_WOBBLE,
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

    elif state == State.EDIT_INT:
        if btn == 'up':     cfg_interval += 1
        elif btn == 'down': cfg_interval = max(1, cfg_interval - 1)
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
        if btn == 'up':     idx = min(len(tv_vals)-1, idx+1)
        elif btn == 'down': idx = max(0, idx-1)
        cfg_max_tv = tv_vals[idx]
        if btn in ('centre', 'left'): set_state(State.SETUP)
        render()

    elif state == State.EDIT_MAX_ISO:
        iso_vals = [400, 800, 1600, 3200, 6400, 12800]
        idx = min(range(len(iso_vals)), key=lambda i: abs(iso_vals[i] - cfg_max_iso))
        if btn == 'up':     idx = min(len(iso_vals)-1, idx+1)
        elif btn == 'down': idx = max(0, idx-1)
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
    GPIO.output(SHUTTER_PIN, GPIO.LOW)
    GPIO.cleanup()
    stop_event.set()
    pause_event.set()
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
