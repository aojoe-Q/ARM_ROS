"""Run portable link/model tests; optionally cross-check the actual firmware codec.

python src/dm_mc02_hardware/test/run_standalone.py --firmware ../S27_TEST_2
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--firmware", type=Path)
parser.add_argument("--compiler", default="g++")
args = parser.parse_args()
package = Path(__file__).resolve().parents[1]
build = package.parents[1] / "build/dm_mc02_standalone"
build.mkdir(parents=True, exist_ok=True)
compiler = shutil.which(args.compiler)
if not compiler:
    raise SystemExit(f"Compiler not found: {args.compiler}")


def compile_run(name, sources, includes=()):
    output = build / (name + (".exe" if os.name == "nt" else ""))
    command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-I", str(package / "include")]
    for directory in includes:
        command.extend(["-I", str(directory)])
    command.extend(str(s) for s in sources)
    command.extend(["-o", str(output)])
    subprocess.run(command, check=True)
    subprocess.run([str(output)], check=True)


compile_run("test_link", [package / "test/test_link.cpp", package / "src/arm_link.cpp"])
subprocess.run([sys.executable, str(package / "test/test_real_model.py")], check=True)
if args.firmware:
    firmware_app = args.firmware.resolve() / "APP"
    compile_run("test_firmware_codec", [package / "test/test_firmware_codec.cpp", firmware_app / "arm_usb_protocol.c"],
                [firmware_app])
print("All standalone checks passed. Full ROS/ament build and hardware commissioning are separate checks.")
