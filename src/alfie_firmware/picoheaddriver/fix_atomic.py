"""
Resolve the RP2040 / micro-ROS 64-bit atomics conflict.

micro-ROS's rcutils ships librcutils__atomic_64bits.c, which defines the
outline 64-bit atomics (__atomic_load_8, __atomic_store_8, __atomic_exchange_8,
__atomic_fetch_add_8, ...) AND references __atomic_test_and_set, which GCC does
not provide for the Cortex-M0+. Meanwhile the arduino-pico Pico SDK (libpico.a,
pico_atomic/atomic.c) already provides the FULL set of these atomics, and does
so cross-core-safely via hardware spinlocks.

Left alone this produces either an undefined reference to __atomic_test_and_set
or "multiple definition" errors against libpico.a. The fix is to drop the
redundant micro-ROS object so the Pico SDK's implementation is the only one:
that removes the duplicate definitions and, since that object was the sole
referencer of __atomic_test_and_set, removes that dangling reference too.

The removal must run once libmicroros.a exists but before the final link, so we
hook it as a pre-action on the firmware ELF.
"""

Import("env")

import os
import subprocess


def strip_micro_ros_atomics(source, target, env):
    libmicroros_path = os.path.join(
        env.subst("$PROJECT_LIBDEPS_DIR"),
        env.subst("$PIOENV"),
        "micro_ros_platformio",
        "libmicroros",
        "libmicroros.a",
    )

    if not os.path.exists(libmicroros_path):
        print(f"fix_atomic: libmicroros.a not found at {libmicroros_path} — skipping")
        return

    ar_tool = env.subst("$AR")
    obj = "librcutils__atomic_64bits.c.obj"

    # Is the object still present? (`ar t` lists members.)
    listing = subprocess.run(
        [ar_tool, "t", libmicroros_path],
        check=False,
        capture_output=True,
        text=True,
    )
    if obj not in listing.stdout.split():
        return  # already stripped on a previous build

    result = subprocess.run(
        [ar_tool, "d", libmicroros_path, obj],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        print(f"fix_atomic: removed {obj} from libmicroros.a (using Pico SDK atomics)")
    else:
        print(f"fix_atomic: WARNING failed to remove {obj}: {result.stderr.strip()}")


# Run right before the ELF is linked, when libmicroros.a is guaranteed to exist.
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", strip_micro_ros_atomics)
