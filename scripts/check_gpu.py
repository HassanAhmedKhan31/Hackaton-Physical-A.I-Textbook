import shutil
import subprocess
import sys

def check_gpu():
    """
    Checks for NVIDIA GPU and VRAM availability using nvidia-smi.
    """
    print("Checking hardware capabilities...")

    if shutil.which("nvidia-smi") is None:
        print("[-] nvidia-smi not found.")
        print("    Recommendation: If you have an NVIDIA GPU, install drivers.")
        print("    If not, consider using NVIDIA Omniverse Cloud or AWS RoboMaker.")
        return

    try:
        # Run nvidia-smi to get memory info
        # --query-gpu=memory.total --format=csv,noheader,nounits
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.total", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, check=True
        )
        
        # Output might contain multiple lines if multiple GPUs are present
        output = result.stdout.strip().split('\n')
        
        if not output:
             print("[-] Could not retrieve GPU memory info.")
             return

        # Check each GPU
        found_capable_gpu = False
        for i, vram_str in enumerate(output):
            try:
                vram_mb = int(vram_str.strip())
                vram_gb = vram_mb / 1024
                print(f"[+] GPU {i}: {vram_gb:.2f} GB VRAM detected.")
                
                if vram_gb >= 8:
                    found_capable_gpu = True
            except ValueError:
                continue

        if found_capable_gpu:
            print("\n[SUCCESS] You have a GPU capable of running Isaac Sim locally!")
            print("          Ensure you have the latest drivers installed.")
        else:
            print("\n[WARNING] Your GPU may struggle with Isaac Sim (Recommend 8GB+ VRAM).")
            print("          Consider closing other apps or using Cloud rendering.")

    except subprocess.CalledProcessError as e:
        print(f"[-] Error running nvidia-smi: {e}")
    except Exception as e:
        print(f"[-] An unexpected error occurred: {e}")

if __name__ == "__main__":
    check_gpu()
