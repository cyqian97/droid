#!/usr/bin/env python3
"""
Test script to verify Oculus Quest headset data can be read correctly.
Checks connectivity, data availability, data types, and value ranges.

Usage:
    python scripts/tests/check_headset.py
"""
import sys
import time
import numpy as np

# --------------------------------------------------------------------------- #
#  1. Check ADB connectivity
# --------------------------------------------------------------------------- #
print("=" * 60)
print("Step 1: Checking ADB connectivity")
print("=" * 60)

try:
    from ppadb.client import Client as AdbClient
    client = AdbClient(host="127.0.0.1", port=5037)
    devices = client.devices()
    if not devices:
        print("[FAIL] No ADB devices found.")
        print("  - Is the headset connected via USB?")
        print("  - Run 'adb devices' to check.")
        print("  - Did you approve USB debugging on the headset?")
        sys.exit(1)
    print(f"[PASS] Found {len(devices)} ADB device(s):")
    for d in devices:
        print(f"  - {d.serial}")
except Exception as e:
    print(f"[FAIL] ADB connection error: {e}")
    print("  - Make sure ADB server is running: 'adb start-server'")
    sys.exit(1)

# --------------------------------------------------------------------------- #
#  2. Initialize OculusReader
# --------------------------------------------------------------------------- #
print()
print("=" * 60)
print("Step 2: Initializing OculusReader")
print("=" * 60)

try:
    from oculus_reader.reader import OculusReader
    reader = OculusReader(run=True)
    print("[PASS] OculusReader initialized and running.")
except Exception as e:
    print(f"[FAIL] OculusReader init failed: {e}")
    sys.exit(1)

# --------------------------------------------------------------------------- #
#  3. Wait for data from the headset
# --------------------------------------------------------------------------- #
print()
print("=" * 60)
print("Step 3: Waiting for data from headset (up to 15 seconds)...")
print("        Make sure the headset is on and controllers are active.")
print("=" * 60)

transforms = {}
buttons = {}
max_wait = 15
start = time.time()

while time.time() - start < max_wait:
    transforms, buttons = reader.get_transformations_and_buttons()
    if transforms and buttons:
        elapsed = time.time() - start
        print(f"[PASS] Data received after {elapsed:.1f}s")
        break
    time.sleep(0.1)
else:
    print(f"[FAIL] No data received after {max_wait}s.")
    print("  - Is the APK running on the headset?")
    print("  - Are the controllers turned on and visible to headset?")
    print("  - Check: adb logcat -s wE9ryARX")
    reader.stop()
    sys.exit(1)

# --------------------------------------------------------------------------- #
#  4. Validate transforms
# --------------------------------------------------------------------------- #
print()
print("=" * 60)
print("Step 4: Validating transforms")
print("=" * 60)

errors = []

# Check keys
expected_controller_keys = {'r', 'l'}
found_keys = set(transforms.keys())
print(f"  Controller keys found: {found_keys}")
if not found_keys:
    errors.append("No controller keys in transforms")
if not found_keys.issubset(expected_controller_keys):
    errors.append(f"Unexpected keys: {found_keys - expected_controller_keys}")

for key in found_keys:
    mat = transforms[key]

    # Check type
    if not isinstance(mat, np.ndarray):
        errors.append(f"transforms['{key}'] is {type(mat).__name__}, expected np.ndarray")
        continue

    # Check shape
    if mat.shape != (4, 4):
        errors.append(f"transforms['{key}'] shape is {mat.shape}, expected (4, 4)")
        continue

    # Check dtype is float
    if not np.issubdtype(mat.dtype, np.floating):
        errors.append(f"transforms['{key}'] dtype is {mat.dtype}, expected float")

    # Check no NaN or Inf
    if np.any(np.isnan(mat)):
        errors.append(f"transforms['{key}'] contains NaN values")
    if np.any(np.isinf(mat)):
        errors.append(f"transforms['{key}'] contains Inf values")

    # Check last row is [0, 0, 0, 1] (standard homogeneous transform)
    expected_last_row = np.array([0, 0, 0, 1], dtype=float)
    if not np.allclose(mat[3], expected_last_row, atol=1e-3):
        errors.append(f"transforms['{key}'] last row is {mat[3]}, expected [0, 0, 0, 1]")

    # Check rotation part is a valid rotation matrix (det ≈ 1, orthogonal)
    rot = mat[:3, :3]
    det = np.linalg.det(rot)
    if not np.isclose(det, 1.0, atol=0.1):
        errors.append(f"transforms['{key}'] rotation det={det:.4f}, expected ~1.0")

    ortho_check = rot @ rot.T
    if not np.allclose(ortho_check, np.eye(3), atol=0.1):
        errors.append(f"transforms['{key}'] rotation matrix not orthogonal")

    # Print the matrix
    name = "Right" if key == "r" else "Left"
    pos = mat[:3, 3]
    print(f"\n  [{name} controller]")
    print(f"    Type: {type(mat).__name__}, dtype: {mat.dtype}, shape: {mat.shape}")
    print(f"    Position (x,y,z): ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"    Rotation det: {det:.4f}")
    print(f"    Last row: {mat[3]}")

if not errors:
    print("\n[PASS] All transform checks passed.")
else:
    for e in errors:
        print(f"\n[FAIL] {e}")

# --------------------------------------------------------------------------- #
#  5. Validate buttons
# --------------------------------------------------------------------------- #
print()
print("=" * 60)
print("Step 5: Validating buttons")
print("=" * 60)

button_errors = []

# Expected boolean buttons (right controller)
right_bool_keys = {'A', 'B', 'RThU', 'RJ', 'RG', 'RTr'}
# Expected boolean buttons (left controller)
left_bool_keys = {'X', 'Y', 'LThU', 'LJ', 'LG', 'LTr'}
# Expected float tuple buttons
right_float_keys = {'rightTrig': (1, 0.0, 1.0),
                    'rightGrip': (1, 0.0, 1.0),
                    'rightJS': (2, -1.0, 1.0)}
left_float_keys = {'leftTrig': (1, 0.0, 1.0),
                   'leftGrip': (1, 0.0, 1.0),
                   'leftJS': (2, -1.0, 1.0)}

print(f"  Button keys found: {sorted(buttons.keys())}")
print()

# Check boolean buttons
all_bool_keys = right_bool_keys | left_bool_keys
for key in sorted(all_bool_keys):
    if key in buttons:
        val = buttons[key]
        if isinstance(val, bool):
            print(f"    {key:>10}: {val:>10}  (bool)  [PASS]")
        else:
            button_errors.append(f"buttons['{key}'] is {type(val).__name__}={val}, expected bool")
            print(f"    {key:>10}: {str(val):>10}  ({type(val).__name__})  [FAIL] expected bool")
    else:
        print(f"    {key:>10}: {'MISSING':>10}  [WARN] key not present")

print()

# Check float tuple buttons
all_float_keys = {**right_float_keys, **left_float_keys}
for key, (expected_len, min_val, max_val) in sorted(all_float_keys.items()):
    if key in buttons:
        val = buttons[key]
        if isinstance(val, tuple):
            type_ok = all(isinstance(v, float) for v in val)
            len_ok = len(val) == expected_len
            range_ok = all(min_val - 0.01 <= v <= max_val + 0.01 for v in val)

            status = "[PASS]" if (type_ok and len_ok and range_ok) else "[FAIL]"
            print(f"    {key:>10}: {str(val):>20}  (tuple of {len(val)} float(s))  {status}")

            if not type_ok:
                button_errors.append(f"buttons['{key}'] contains non-float: {val}")
            if not len_ok:
                button_errors.append(f"buttons['{key}'] has {len(val)} elements, expected {expected_len}")
            if not range_ok:
                button_errors.append(f"buttons['{key}'] value {val} outside [{min_val}, {max_val}]")
        else:
            button_errors.append(f"buttons['{key}'] is {type(val).__name__}, expected tuple")
            print(f"    {key:>10}: {str(val):>20}  ({type(val).__name__})  [FAIL] expected tuple")
    else:
        print(f"    {key:>10}: {'MISSING':>20}  [WARN] key not present")

if not button_errors:
    print(f"\n[PASS] All button checks passed.")
else:
    for e in button_errors:
        print(f"\n[FAIL] {e}")

# --------------------------------------------------------------------------- #
#  6. Test data streaming (FPS check)
# --------------------------------------------------------------------------- #
print()
print("=" * 60)
print("Step 6: Measuring data streaming rate (5 seconds)...")
print("=" * 60)

sample_count = 0
samples = []
fps_start = time.time()

while time.time() - fps_start < 5.0:
    t, b = reader.get_transformations_and_buttons()
    if t and b:
        sample_count += 1
        samples.append({'transforms': t, 'buttons': b})
    time.sleep(0.01)  # 100Hz polling

print(f"  Samples received: {sample_count} in 5s")
print(f"  Effective read rate: ~{sample_count / 5:.1f} Hz")

if sample_count > 10:
    print("[PASS] Data is streaming.")
else:
    print("[WARN] Low data rate. Check headset APK and controller status.")

# --------------------------------------------------------------------------- #
#  7. Test that VRPolicy can consume the data
# --------------------------------------------------------------------------- #
print()
print("=" * 60)
print("Step 7: Testing VRPolicy compatibility")
print("=" * 60)

vr_compat_errors = []

# Simulate what VRPolicy._update_internal_state expects
if 'r' in transforms:
    controller_id = 'r'
elif 'l' in transforms:
    controller_id = 'l'
else:
    vr_compat_errors.append("No controller data ('r' or 'l') in transforms")
    controller_id = None

if controller_id:
    # Check movement_enabled: needs RG/LG
    grip_key = controller_id.upper() + "G"
    if grip_key not in buttons:
        vr_compat_errors.append(f"Missing '{grip_key}' (grip boolean) needed for movement enable")
    elif not isinstance(buttons[grip_key], bool):
        vr_compat_errors.append(f"'{grip_key}' should be bool, got {type(buttons[grip_key]).__name__}")

    # Check joystick press: needs RJ/LJ
    js_key = controller_id.upper() + "J"
    if js_key not in buttons:
        vr_compat_errors.append(f"Missing '{js_key}' (joystick press) needed for orientation reset")

    # Check trigger: needs rightTrig/leftTrig
    trig_key = "rightTrig" if controller_id == "r" else "leftTrig"
    if trig_key not in buttons:
        vr_compat_errors.append(f"Missing '{trig_key}' (index trigger) needed for gripper control")
    elif not isinstance(buttons[trig_key], tuple) or len(buttons[trig_key]) < 1:
        vr_compat_errors.append(f"'{trig_key}' should be tuple with at least 1 float")

    # Check A/B or X/Y for success/failure signals
    if controller_id == 'r':
        for k in ['A', 'B']:
            if k not in buttons:
                vr_compat_errors.append(f"Missing '{k}' button for success/failure signal")
    else:
        for k in ['X', 'Y']:
            if k not in buttons:
                vr_compat_errors.append(f"Missing '{k}' button for success/failure signal")

    # Test transform processing (what VRPolicy._process_reading does)
    try:
        from droid.misc.transformations import rmat_to_quat
        rot_mat = np.asarray(transforms[controller_id])
        inv_mat = np.linalg.inv(rot_mat)  # vr_to_global_mat
        rmat_reorder = [-2, -1, -3, 4]
        X = np.zeros((len(rmat_reorder), len(rmat_reorder)))
        for i in range(X.shape[0]):
            ind = int(abs(rmat_reorder[i])) - 1
            X[i, ind] = np.sign(rmat_reorder[i])
        global_to_env_mat = X

        processed = global_to_env_mat @ inv_mat @ rot_mat
        pos = processed[:3, 3]
        quat = rmat_to_quat(processed[:3, :3])
        print(f"  Processed position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
        print(f"  Processed quaternion: ({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})")
        print(f"  Gripper trigger: {buttons[trig_key][0]:.4f}")
    except Exception as e:
        vr_compat_errors.append(f"Transform processing failed: {e}")

if not vr_compat_errors:
    print("[PASS] Data is compatible with VRPolicy.")
else:
    for e in vr_compat_errors:
        print(f"[FAIL] {e}")

# --------------------------------------------------------------------------- #
#  Summary
# --------------------------------------------------------------------------- #
print()
print("=" * 60)
print("Summary")
print("=" * 60)

all_errors = errors + button_errors + vr_compat_errors
if not all_errors:
    print("[PASS] All checks passed! Headset data is ready for teleoperation.")
else:
    print(f"[ISSUES] {len(all_errors)} issue(s) found:")
    for e in all_errors:
        print(f"  - {e}")

# Cleanup
print()
print("Stopping reader...")
reader.stop()
print("Done.")
