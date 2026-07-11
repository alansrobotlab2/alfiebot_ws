"""
Control driver for the reSpeaker XVF3800 4-Mic Array (USB VID:PID 2886:001a).

Ported from Seeed's official python_control example (xvf_host.py):
  https://github.com/respeaker/reSpeaker_XVF3800_USB_4MIC_ARRAY/tree/master/python_control
  https://wiki.seeedstudio.com/respeaker_xvf3800_python_sdk/

The XVF3800 speaks a *different* USB control protocol than the older
XVF3000-based array. Parameters are addressed by (resid, cmdid); reads carry a
leading status byte and must be retried while the servicer returns
SERVICER_COMMAND_RETRY.
"""
import sys
import time
import struct

import usb.core
import usb.util

VID = 0x2886
PID = 0x001A

CONTROL_SUCCESS = 0
SERVICER_COMMAND_RETRY = 64

# name: (resid, cmdid, count, access, type[, description])
#   resid  -> USB control transfer wIndex
#   cmdid  -> USB control transfer wValue (OR'd with 0x80 for reads)
#   access -> "ro" | "wo" | "rw"
PARAMETERS = {
    # APPLICATION_SERVICER_RESID (48)
    "VERSION": (48, 0, 3, "ro", "uint8"),
    "REBOOT": (48, 7, 1, "wo", "uint8"),
    "SAVE_CONFIGURATION": (48, 9, 1, "wo", "uint8"),
    "CLEAR_CONFIGURATION": (48, 10, 1, "wo", "uint8"),

    # AEC_RESID (33)
    "AEC_AZIMUTH_VALUES": (33, 75, 4, "ro", "radians"),
    "AEC_SPENERGY_VALUES": (33, 80, 4, "ro", "float"),

    # AUDIO_MGR_RESID (35)
    "AUDIO_MGR_MIC_GAIN": (35, 0, 1, "rw", "float"),
    "AUDIO_MGR_REF_GAIN": (35, 1, 1, "rw", "float"),
    "AUDIO_MGR_SELECTED_AZIMUTHS": (35, 11, 2, "ro", "radians"),

    # GPO_SERVICER_RESID (20) - LEDs + direction of arrival
    "LED_EFFECT": (20, 12, 1, "rw", "uint8"),
    "LED_BRIGHTNESS": (20, 13, 1, "rw", "uint8"),
    "LED_GAMMIFY": (20, 14, 1, "rw", "uint8"),   # gamma correction on/off (smooths breath)
    "LED_SPEED": (20, 15, 1, "rw", "uint8"),
    "LED_COLOR": (20, 16, 1, "rw", "uint32"),
    "LED_DOA_COLOR": (20, 17, 2, "rw", "uint32"),  # (base_color, doa_color) for DOA mode
    "DOA_VALUE": (20, 18, 2, "ro", "uint16"),

    # PP_RESID (17) - post-processing: AGC / noise / echo suppression
    "PP_AGCONOFF": (17, 10, 1, "rw", "int32"),
    "PP_AGCMAXGAIN": (17, 11, 1, "rw", "float"),
    "PP_AGCDESIREDLEVEL": (17, 12, 1, "rw", "float"),
    "PP_AGCGAIN": (17, 13, 1, "rw", "float"),
    "PP_AGCTIME": (17, 14, 1, "rw", "float"),
    "PP_LIMITONOFF": (17, 19, 1, "rw", "int32"),
    "PP_MIN_NS": (17, 21, 1, "rw", "float"),
    "PP_MIN_NN": (17, 22, 1, "rw", "float"),
    "PP_ECHOONOFF": (17, 23, 1, "rw", "int32"),
    "PP_NLATTENONOFF": (17, 27, 1, "rw", "int32"),
}


class ReSpeaker(object):
    TIMEOUT = 100000

    def __init__(self, dev):
        self.dev = dev

    def write(self, name, data_list):
        try:
            resid, cmdid, count, access, dtype = PARAMETERS[name][:5]
        except KeyError:
            raise ValueError("Unknown parameter: {}".format(name))

        if access == "ro":
            raise ValueError("{} is read-only".format(name))
        if len(data_list) != count:
            raise ValueError("{} expects {} value(s)".format(name, count))

        payload = bytearray()
        if dtype in ("float", "radians"):
            for v in data_list:
                payload += struct.pack("<f", float(v))
        elif dtype in ("uint8", "char"):
            for v in data_list:
                payload += int(v).to_bytes(1, byteorder="little")
        elif dtype == "uint32":
            for v in data_list:
                payload += struct.pack("<I", int(v))
        else:  # int32 / default
            for v in data_list:
                payload += struct.pack("<i", int(v))

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmdid, resid, bytes(payload), self.TIMEOUT)

    def read(self, name):
        try:
            resid, cmdid, count, access, dtype = PARAMETERS[name][:5]
        except KeyError:
            raise ValueError("Unknown parameter: {}".format(name))

        wvalue = 0x80 | cmdid
        if dtype in ("uint8", "char"):
            length = count + 1
        elif dtype == "uint16":
            length = count * 2 + 1
        else:  # float / radians / uint32 / int32
            length = count * 4 + 1

        attempts = 0
        while True:
            response = self.dev.ctrl_transfer(
                usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0, wvalue, resid, length, self.TIMEOUT)
            status = response[0]
            if status == CONTROL_SUCCESS:
                break
            if status != SERVICER_COMMAND_RETRY:
                raise ValueError("Unknown status code: {}".format(status))
            attempts += 1
            if attempts > 100:
                raise ValueError("Read of {} exceeded 100 retries".format(name))
            time.sleep(0.01)

        body = response.tobytes()[1:]
        if dtype == "char":
            return body.rstrip(b"\x00").decode("utf-8", errors="ignore")
        fmt = {"uint8": "B", "uint16": "H", "uint32": "I",
               "int32": "i", "float": "f", "radians": "f"}[dtype]
        return struct.unpack("<" + fmt * count, body)

    def close(self):
        usb.util.dispose_resources(self.dev)


def find(vid=VID, pid=PID):
    """Return a ReSpeaker wrapper for the first matching device, or None."""
    if sys.platform.startswith("win"):
        try:
            import libusb_package
            dev = libusb_package.find(idVendor=vid, idProduct=pid)
        except ImportError:
            dev = usb.core.find(idVendor=vid, idProduct=pid)
    else:
        dev = usb.core.find(idVendor=vid, idProduct=pid)
    if not dev:
        return None
    return ReSpeaker(dev)
