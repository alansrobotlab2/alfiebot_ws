"""
Custom build script to fix atomic operation conflicts between Pico SDK and micro-ROS.
This script removes the conflicting atomic_64bits object from libmicroros.a
and uses the Pico SDK's atomic implementation instead.
"""

Import("env")

def fix_atomic_conflicts(source, target, env):
    """Remove conflicting atomic object from libmicroros after library is built."""
    import os
    import subprocess
    
    # Path to libmicroros.a
    libmicroros_path = os.path.join(
        env.subst("$PROJECT_LIBDEPS_DIR"),
        env.subst("$PIOENV"),
        "micro_ros_platformio",
        "libmicroros",
        "libmicroros.a"
    )
    
    if os.path.exists(libmicroros_path):
        print(f"Fixing atomic conflicts in {libmicroros_path}")
        
        # Get the toolchain path
        ar_tool = env.subst("$AR")
        
        # Remove the conflicting atomic_64bits object file
        try:
            subprocess.run(
                [ar_tool, "d", libmicroros_path, "librcutils__atomic_64bits.c.obj"],
                check=False,
                capture_output=True
            )
            print("Removed conflicting atomic_64bits object from libmicroros.a")
        except Exception as e:
            print(f"Warning: Could not remove atomic_64bits object: {e}")

# Run immediately before linking.
#
# This was previously a post-action on $BUILD_DIR/libFrameworkArduino.a, which
# only fires when the Arduino framework library itself is rebuilt. Any build that
# regenerated libmicroros.a while libFrameworkArduino.a stayed cached - notably
# `pio run -t clean_microros` followed by a normal build, which is what you do
# after changing a .msg - left the conflicting object in place and failed at link
# with "multiple definition of __atomic_load_8".
#
# The ELF target always needs building when anything changed, so hooking here
# means the fix is applied on every link. Re-running `ar d` when the member is
# already gone is a harmless no-op.
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", fix_atomic_conflicts)
