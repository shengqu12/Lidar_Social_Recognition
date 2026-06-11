#!/usr/bin/env python3
"""
fix_rosbridge_imu.py
--------------------
Patches rosbridge_library 2.0.6 message_conversion.py on ROS2 Humble so that
any ROS message object (e.g. sensor_msgs/Imu with nested Quaternion, Vector3,
Header, and float64[9] covariance numpy arrays) can be serialised to JSON
without hitting "cannot serialize type <class '...'>".

Run with:
    sudo python3 fix_rosbridge_imu.py

Safe to re-run: backs up once to .bak2, is idempotent after that.
"""

import sys
import os
import shutil
import difflib

TARGET = (
    "/opt/ros/humble/local/lib/python3.10/dist-packages/"
    "rosbridge_library/internal/message_conversion.py"
)
BACKUP = TARGET + ".bak2"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def die(msg):
    print(f"[ERROR] {msg}", file=sys.stderr)
    sys.exit(1)


def show_diff(original: str, patched: str, filename: str = TARGET):
    diff = list(
        difflib.unified_diff(
            original.splitlines(keepends=True),
            patched.splitlines(keepends=True),
            fromfile=filename + " (before)",
            tofile=filename + " (after)",
        )
    )
    if diff:
        print("\n===== DIFF =====")
        for line in diff:
            print(line, end="")
        print("\n================\n")
    else:
        print("[INFO] No diff — file unchanged.")


# ---------------------------------------------------------------------------
# The old extract_values / _from_object_inst code we are replacing.
# rosbridge 2.0.6 ships exactly this block inside extract_values().
#
# The function tries isinstance checks in order:
#   bool → int → float → str → list/tuple → dict
# and then raises TypeError for anything else (including ROS msg objects).
#
# We replace the final "raise TypeError" branch with a generic ROS-message
# handler that uses get_fields_and_field_types() — the standard ROS2 API
# present on every generated message class.
# ---------------------------------------------------------------------------

# The exact string we look for (rosbridge 2.0.6 – covers the TypeError raise
# and a few lines of context so the match is unique and safe).
OLD_FRAGMENT = '''\
    elif isinstance(inst, list) or isinstance(inst, tuple):
        return [extract_values(item) for item in inst]
    elif isinstance(inst, dict):
        return {k: extract_values(v) for k, v in inst.items()}
    else:
        raise TypeError("Cannot serialize type %s" % type(inst))'''

# Replacement: same list/tuple/dict branches, then a generic ROS-message
# handler, then the original TypeError for truly unknown types.
NEW_FRAGMENT = '''\
    elif isinstance(inst, list) or isinstance(inst, tuple):
        return [extract_values(item) for item in inst]
    elif isinstance(inst, dict):
        return {k: extract_values(v) for k, v in inst.items()}
    elif hasattr(inst, 'get_fields_and_field_types'):
        # Generic handler for any ROS2 message object (Imu, Quaternion, …).
        # get_fields_and_field_types() is part of the rclpy generated-message
        # API and is available on every sensor_msgs, geometry_msgs, etc. type.
        result = {}
        for field_name in inst.get_fields_and_field_types().keys():
            field_val = getattr(inst, field_name)
            result[field_name] = extract_values(field_val)
        return result
    else:
        raise TypeError("Cannot serialize type %s" % type(inst))'''

# ---------------------------------------------------------------------------
# numpy array handling — covariance fields are numpy.ndarray of float64.
# We also need to handle numpy scalars (numpy.float64, numpy.int32, …).
# These are inserted BEFORE the list/tuple branch (closer to top of the
# elif chain) so they are caught early.
# ---------------------------------------------------------------------------

OLD_NUMPY_ANCHOR = '''\
    elif isinstance(inst, list) or isinstance(inst, tuple):
        return [extract_values(item) for item in inst]'''

NEW_NUMPY_ANCHOR = '''\
    elif hasattr(inst, 'tolist') and hasattr(inst, 'dtype'):
        # numpy ndarray or numpy scalar → convert to plain Python type(s).
        return inst.tolist()
    elif isinstance(inst, list) or isinstance(inst, tuple):
        return [extract_values(item) for item in inst]'''

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print(f"[INFO] Target : {TARGET}")
    print(f"[INFO] Backup : {BACKUP}")

    # ── 1. Sanity checks ────────────────────────────────────────────────────
    if not os.path.isfile(TARGET):
        die(f"Target file not found: {TARGET}")

    if os.geteuid() != 0:
        die("This script must be run as root (sudo).")

    # ── 2. Read current content ─────────────────────────────────────────────
    with open(TARGET, "r", encoding="utf-8") as fh:
        original = fh.read()

    # ── 3. Back up once ─────────────────────────────────────────────────────
    if not os.path.isfile(BACKUP):
        shutil.copy2(TARGET, BACKUP)
        print(f"[INFO] Backed up to {BACKUP}")
    else:
        print(f"[INFO] Backup already exists at {BACKUP}, skipping.")

    # ── 4. Show the current extract_values function ─────────────────────────
    # Locate and print the function so the user can see what's there now.
    func_start = original.find("def extract_values(")
    if func_start == -1:
        # Some versions call it _from_object_inst — try that.
        func_start = original.find("def _from_object_inst(")
    if func_start == -1:
        die(
            "Could not locate 'extract_values' or '_from_object_inst' in the file.\n"
            "The rosbridge version on this Jetson may differ from 2.0.6.\n"
            "Please inspect the file manually:\n"
            f"  grep -n 'Cannot serialize' {TARGET}"
        )

    # Print ~50 lines of the function for inspection.
    func_lines = original[func_start:].splitlines()[:60]
    print("\n===== CURRENT extract_values (first 60 lines) =====")
    for i, line in enumerate(func_lines, start=1):
        print(f"  {i:3d}  {line}")
    print("===================================================\n")

    # ── 5. Check idempotency ─────────────────────────────────────────────────
    if "get_fields_and_field_types" in original:
        print("[INFO] Patch already applied (get_fields_and_field_types found). Nothing to do.")
        sys.exit(0)

    # ── 6. Apply patches ─────────────────────────────────────────────────────
    patched = original

    # 6a. numpy ndarray / scalar handling (insert before list/tuple branch)
    if OLD_NUMPY_ANCHOR not in patched:
        print(
            "[WARN] Could not find the list/tuple branch anchor for numpy patch.\n"
            "       Numpy arrays may not be handled. Continuing with ROS-message patch only."
        )
    else:
        patched = patched.replace(OLD_NUMPY_ANCHOR, NEW_NUMPY_ANCHOR, 1)
        print("[INFO] numpy ndarray/scalar patch applied.")

    # 6b. ROS message object generic handler (replace the TypeError block)
    if OLD_FRAGMENT not in patched:
        # The numpy patch already shifted the text — try matching without the
        # list/tuple lines (they are now after the numpy block).
        ALT_OLD = '''\
    elif isinstance(inst, dict):
        return {k: extract_values(v) for k, v in inst.items()}
    else:
        raise TypeError("Cannot serialize type %s" % type(inst))'''
        ALT_NEW = '''\
    elif isinstance(inst, dict):
        return {k: extract_values(v) for k, v in inst.items()}
    elif hasattr(inst, 'get_fields_and_field_types'):
        result = {}
        for field_name in inst.get_fields_and_field_types().keys():
            field_val = getattr(inst, field_name)
            result[field_name] = extract_values(field_val)
        return result
    else:
        raise TypeError("Cannot serialize type %s" % type(inst))'''
        if ALT_OLD not in patched:
            die(
                "Could not find the 'raise TypeError(\"Cannot serialize type\")' block.\n"
                "The file layout differs from expected rosbridge 2.0.6.\n"
                "Inspect manually and apply the patch by hand:\n\n"
                "Add BEFORE the final 'else: raise TypeError(...)' in extract_values:\n\n"
                "    elif hasattr(inst, 'get_fields_and_field_types'):\n"
                "        result = {}\n"
                "        for field_name in inst.get_fields_and_field_types().keys():\n"
                "            field_val = getattr(inst, field_name)\n"
                "            result[field_name] = extract_values(field_val)\n"
                "        return result\n"
            )
        patched = patched.replace(ALT_OLD, ALT_NEW, 1)
    else:
        patched = patched.replace(OLD_FRAGMENT, NEW_FRAGMENT, 1)

    print("[INFO] ROS message generic-handler patch applied.")

    # ── 7. Show diff ─────────────────────────────────────────────────────────
    show_diff(original, patched)

    # ── 8. Write patched file ────────────────────────────────────────────────
    with open(TARGET, "w", encoding="utf-8") as fh:
        fh.write(patched)

    print(f"[OK] {TARGET} patched successfully.\n")

    # ── 9. Quick sanity-check: import the patched module ─────────────────────
    print("[INFO] Attempting to import patched module …")
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("message_conversion", TARGET)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        print("[OK] Module imports without error.\n")
    except Exception as exc:
        print(f"[WARN] Import check failed: {exc}")
        print("       This may be a false alarm if rclpy is not on sys.path yet.")
        print("       Verify after sourcing the ROS environment.\n")

    # ── 10. What to do next ──────────────────────────────────────────────────
    print("=" * 60)
    print("NEXT STEPS — run these ON THE JETSON:")
    print("=" * 60)
    print()
    print("# 1. Kill any running rosbridge / rosapi processes")
    print("pkill -9 -f rosbridge_websocket; pkill -9 -f rosapi; sleep 3")
    print()
    print("# 2. Source ROS environments (in the same terminal you will launch from)")
    print("source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash")
    print()
    print("# 3. Launch rosbridge in the background")
    print(
        "nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml "
        "> /tmp/rosbridge.log 2>&1 &"
    )
    print()
    print("# 4. Tail the log to confirm no errors")
    print("tail -f /tmp/rosbridge.log")
    print()
    print("# 5. In Foxglove, reconnect and subscribe to /livox/imu — it should stream.")
    print()
    print("=" * 60)


if __name__ == "__main__":
    main()
