import os
import sys
import subprocess
import time
from glob import glob
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
platform = env.PioPlatform()

# --- 1. Конфигурация пути к pioasm (Кроссплатформенный поиск) ---
try:
    pioasm_dir = platform.get_package_dir("framework-picosdk")
    if not pioasm_dir:
        raise Exception("Package tool-wizio-pico not found!")
    
    PIOASM_BIN = None
    if os.name == 'nt': 
        PIOASM_BIN = os.path.join(pioasm_dir, "windows_x86_64", "pioasm.exe")
    elif sys.platform == 'darwin':
        PIOASM_BIN = os.path.join(pioasm_dir, "macos_x86_64", "pioasm")
    else: # Linux
        PIOASM_BIN = os.path.join(pioasm_dir, "linux_x86_64", "pioasm")

    if not PIOASM_BIN or not os.path.exists(PIOASM_BIN):
        # Запасной вариант
        PIOASM_BIN = os.path.join(pioasm_dir, "pioasm")
        if not os.path.exists(PIOASM_BIN):
           raise Exception(f"PIOASM executable not found at: {PIOASM_BIN}")
            
    if os.name != 'nt':
        os.chmod(PIOASM_BIN, 0o755)

except Exception as e:
    print(f"FATAL: Could not locate pioasm executable automatically. Error: {e}")
    env.Exit(1) 


# --- 2. Функция генерации PIO ---
def build_pio_files(target, source, env):
    print("--- Running pioasm on .pio files ---")
    pio_files_found = glob(os.path.join(env.subst("$PROJECT_DIR"), "src", "*.pio"))
    
    if not pio_files_found:
        print("No .pio files found in src/, skipping pioasm.")
        return
    include_dir = os.path.join(env.subst("$PROJECT_DIR"), "include")
    if not os.path.exists(include_dir):
        os.makedirs(include_dir)

    for pio_file in pio_files_found:
        header_file = os.path.join(include_dir, os.path.basename(pio_file) + ".h")
        try:
            subprocess.run([PIOASM_BIN, "-o", "c-sdk", pio_file, header_file], check=True, capture_output=True, text=True)
            print(f"Generated {header_file}")
        except subprocess.CalledProcessError as e:
             print(f"Error generating {header_file}: {e.stderr}")
             sys.exit(1)
    print("------------------------------------------")



# def auto_upload(target, source, env):  # pylint: disable=W0613,W0621
#     print("--- Running BeforeUpload Hook ---")
#     print("Running env.AutodetectUploadPort()...")
#     env.AutodetectUploadPort()
#     upload_port = env.subst("$UPLOAD_PORT")
#     print(f"[DEBUG] UPLOAD_PORT determined: {upload_port}")
#     if not upload_port:
#         print("Fatal error: PlatformIO could not autodetect upload port.")
#         env.Exit(1)
#
#     print(f"Sending 1200bps touch signal to {upload_port}")
#     env.TouchSerialPort("$UPLOAD_PORT", 1200)
#     time.sleep(5)
# env.AddPreAction("upload", auto_upload)


# --- 4. Привязка функций к событиям PlatformIO ---
env.AddPreAction("buildprog", build_pio_files)
