import os
import pathlib
import subprocess

pioasm_path = str(pathlib.Path("C:/Users/user/.platformio/packages/tool-wizio-pico/windows/pioasm.exe"))

def compile_pio_files(pioasm_bin, src_dir, out_dir):
    for root, dirs, files in os.walk(src_dir):
        for file in files:
            if file.endswith('.pio'):
                rel_path = os.path.relpath(root, src_dir)
                output_dir = os.path.join(out_dir, rel_path)
                os.makedirs(output_dir, exist_ok=True)
                src_path = os.path.join(root, file)
                out_path = os.path.join(output_dir, file.replace('.pio', '.pio.h'))
                try:
                    # Запускаем pioasm с аргументами для генерации заголовочного файла
                    subprocess.run([pioasm_path, "-o", "c-sdk", src_path, out_path], check=True)
                    print(f"Compiled {src_path} -> {out_path}")
                except subprocess.CalledProcessError as e:
                    print(f"Error compiling {src_path}: {e}")

# Запуск компиляции
if __name__ == "__main__":
    script_dir = pathlib.Path(__file__).parent.resolve()
    source_directory = script_dir / "../src"
    include_directory = script_dir / "../include"
    compile_pio_files(pioasm_path, source_directory, include_directory)
