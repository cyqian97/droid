#!/usr/bin/env python3
"""
Test script to check ZED camera connectivity
"""
import sys

try:
    import pyzed.sl as sl
    print("✓ PyZED imported successfully")
except ImportError as e:
    print(f"✗ Failed to import pyzed: {e}")
    print("Install with: pip install pyzed")
    sys.exit(1)

# Try to detect cameras
print("\n" + "="*50)
print("Searching for ZED cameras...")
print("="*50)

cameras_found = []

# Try up to 4 camera indices
for i in range(4):
    cam = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.sdk_verbose = False

    # Try to open camera at index i
    err = cam.open(init_params)

    if err == sl.ERROR_CODE.SUCCESS:
        # Get camera info
        cam_info = cam.get_camera_information()
        serial = cam_info.serial_number
        model = cam_info.camera_model

        print(f"\n✓ Camera {i} detected:")
        print(f"  Serial Number: {serial}")
        print(f"  Model: {model}")
        print(f"  Firmware: {cam_info.camera_configuration.firmware_version}")

        cameras_found.append({
            'index': i,
            'serial': serial,
            'model': str(model)
        })

        cam.close()
    elif err == sl.ERROR_CODE.CAMERA_NOT_DETECTED:
        # No camera at this index, skip silently
        pass
    else:
        print(f"\n✗ Error at index {i}: {err}")

print("\n" + "="*50)
print(f"Total cameras found: {len(cameras_found)}")
print("="*50)

if cameras_found:
    print("\nTo use these cameras, update droid/misc/parameters.py:")
    for cam in cameras_found:
        print(f"  Camera {cam['index']} (SN: {cam['serial']})")

    print("\nExample configuration:")
    if len(cameras_found) >= 1:
        print(f"hand_camera_id = '{cameras_found[0]['serial']}'")
    if len(cameras_found) >= 2:
        print(f"varied_camera_1_id = '{cameras_found[1]['serial']}'")
    if len(cameras_found) >= 3:
        print(f"varied_camera_2_id = '{cameras_found[2]['serial']}'")
else:
    print("\n✗ No ZED cameras detected!")
    print("\nTroubleshooting:")
    print("1. Check USB connections")
    print("2. Try: lsusb | grep -i stereolabs")
    print("3. Check camera permissions: ls -l /dev/video*")
    print("4. Ensure ZED SDK is installed")
    print("5. Try unplugging and replugging cameras")
    print("6. Check Docker USB device mapping if using containers")

print("\n")