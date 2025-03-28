#!/usr/bin/env python3
import os
import sys
import importlib

def check_file(filepath):
    exists = os.path.exists(filepath)
    print(f"{'✓' if exists else '✗'} {filepath} {'exists' if exists else 'missing'}")
    return exists

def check_module(module_name):
    try:
        importlib.import_module(module_name)
        print(f"✓ Module '{module_name}' is available")
        return True
    except ImportError as e:
        print(f"✗ Module '{module_name}' is missing: {str(e)}")
        return False

if __name__ == "__main__":
    print("Checking system setup...")
    
    # Check critical files
    required_files = [
        "grid_distance_detection.py",
        "test_api.py",
        "combined_app.py",
        "yolov3-tiny.weights",
        "yolov3-tiny.cfg",
        "coco.names"
    ]
    
    files_ok = all(check_file(f) for f in required_files)
    
    # Check required Python modules
    required_modules = [
        "pyrealsense2",
        "numpy",
        "cv2",
        "flask"
    ]
    
    modules_ok = all(check_module(m) for m in required_modules)
    
    # Summary
    print("\nSummary:")
    if files_ok and modules_ok:
        print("✓ All checks passed! System is ready to run.")
    else:
        print("✗ Some checks failed. Please resolve the issues above.")
        sys.exit(1)
