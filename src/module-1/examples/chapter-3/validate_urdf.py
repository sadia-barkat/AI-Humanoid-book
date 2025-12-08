# src/module-1/examples/chapter-3/validate_urdf.py
import subprocess
import os

def validate_humanoid_urdf():
    urdf_path = os.path.join(os.path.dirname(__file__), '../../urdf/humanoid.urdf')
    
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found at {urdf_path}")
        return False, f"URDF file not found at {urdf_path}"

    try:
        # Use subprocess to run check_urdf
        # check_urdf is typically a C++ executable, so we call it directly
        result = subprocess.run(['check_urdf', urdf_path], capture_output=True, text=True, check=True)
        
        print("check_urdf output:")
        print(result.stdout)
        print(result.stderr)
        
        if "ERROR" in result.stderr.upper() or "ERROR" in result.stdout.upper():
            return False, "URDF validation failed with errors."
        
        print(f"Successfully validated URDF file: {urdf_path}")
        return True, "URDF validation successful."
    except subprocess.CalledProcessError as e:
        print(f"check_urdf command failed with exit code {e.returncode}")
        print("stdout:", e.stdout)
        print("stderr:", e.stderr)
        return False, f"check_urdf command failed: {e.stderr}"
    except FileNotFoundError:
        return False, "check_urdf command not found. Is ROS 2 sourced correctly?"

if __name__ == '__main__':
    success, message = validate_humanoid_urdf()
    print(message)
    if not success:
        exit(1)
