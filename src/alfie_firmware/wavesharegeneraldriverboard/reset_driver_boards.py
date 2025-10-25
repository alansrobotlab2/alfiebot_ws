import os

"""
import os
upload_port = "/dev/ttyUSB0"

def get_esptoolpy_reset_flags(resetmethod):
    # no dtr, no_sync
    resets = ("no_reset_no_sync", "soft_reset")
    if resetmethod == "nodemcu":
        # dtr
        resets = ("default_reset", "hard_reset")
    elif resetmethod == "ck":
        # no dtr
        resets = ("no_reset", "soft_reset")

    return ["--before", resets[0], "--after", resets[1]]

reset_flags = ' '.join(get_esptoolpy_reset_flags("nodemcu"))

print(f"esptool.py --port {upload_port} {reset_flags} --no-stub run")
"""




def main(args=None):
    commands = [
    "esptool --port /dev/ttyUSB0 --before default-reset --after hard-reset --no-stub run",
    "esptool --port /dev/ttyUSB1 --before default-reset --after hard-reset --no-stub run"
    ]

    for command in commands:
        os.system(command)


if __name__ == '__main__':
    main()