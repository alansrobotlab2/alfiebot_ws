#  RP2040-Zero has no auto-reset circuit, so after rp2040load writes the new
#  binary the board occasionally stays parked in the BOOTSEL bootloader instead
#  of starting the app (you'd have to unplug/replug). This post-upload hook kicks
#  it into the application with `picotool reboot -a` -- but ONLY when the board is
#  actually stuck.
#
#  rp2040load already reboots into the app on its own most of the time. When it
#  does, there's no BOOTSEL device left, and `picotool reboot` would BLOCK waiting
#  for one (adding ~15s to every good upload). So we first check whether a board
#  is sitting in BOOTSEL mode (USB id 2e8a:0003) and skip picotool entirely if
#  not -- zero added latency in the normal case.

import os
import subprocess
import time

Import("env")  # noqa: F821  (injected by PlatformIO)

# USB VID:PID an RP2040 presents while in the BOOTSEL bootloader.
BOOTSEL_VID = "2e8a"
BOOTSEL_PID = "0003"


def bootsel_device_present():
    root = "/sys/bus/usb/devices"
    try:
        entries = os.listdir(root)
    except OSError:
        return False
    for name in entries:
        try:
            with open(os.path.join(root, name, "idVendor")) as f:
                vid = f.read().strip().lower()
            with open(os.path.join(root, name, "idProduct")) as f:
                pid = f.read().strip().lower()
        except OSError:
            continue
        if vid == BOOTSEL_VID and pid == BOOTSEL_PID:
            return True
    return False


def find_picotool(env):
    candidates = []
    # The platform may register the tool under a name we can't predict, so try
    # the registry but don't depend on it.
    try:
        d = env.PioPlatform().get_package_dir("tool-rp2040tools")
        if d:
            candidates.append(d)
    except Exception:
        pass
    # Fall back to the core packages dir, where the tool physically lives.
    candidates.append(os.path.join(env.subst("$PROJECT_PACKAGES_DIR"), "tool-rp2040tools"))
    for d in candidates:
        p = os.path.join(d, "picotool")
        if os.path.isfile(p):
            return p
    return None


def reboot_into_app(source, target, env):
    # Let rp2040load's own reboot settle so a normally-rebooted board has already
    # dropped off the USB bus before we check.
    time.sleep(0.5)
    if not bootsel_device_present():
        print("reboot_after_upload: board already running (no BOOTSEL device)")
        return
    picotool = find_picotool(env)
    if not picotool:
        print("reboot_after_upload: board stuck in BOOTSEL but picotool not found")
        return
    print("reboot_after_upload: board stuck in BOOTSEL, rebooting into application...")
    try:
        subprocess.run(
            [picotool, "reboot", "-a"],
            timeout=10,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:  # never let the safety net break the upload
        print("reboot_after_upload: %s (skipping)" % e)


env.AddPostAction("upload", reboot_into_app)  # noqa: F821
