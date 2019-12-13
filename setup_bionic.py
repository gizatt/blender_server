#!/usr/bin/env python3
import argparse
from subprocess import check_call
import os
from os.path import dirname, abspath, join, isdir, isfile, expanduser
import tarfile



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--with_drake", action="store_true")
    args = parser.parse_args()

    cur_dir = dirname(abspath(__file__))
    build_dir = join(cur_dir, "build")
    os.makedirs(build_dir, exist_ok=True)

    # Extract.
    blender_base = "blender-2.81-linux-glibc217-x86_64"
    blender_dir = join(build_dir, blender_base)
    blender_url = f"https://mirror.clarkson.edu/blender/release/Blender2.81/{blender_base}.tar.bz2"
    blender_archive = join(build_dir, f"{blender_base}.tar.bz2")
    if not isdir(blender_dir):
        print(blender_archive)
        if not isfile(blender_archive):
            check_call(["wget", blender_url, "-O", blender_archive])
            # sha256sum = "7276216e95b28c74306cec21b6d61e202cbe14035a15a77dbc45fe9d98fca7aa  blender-2.81-linux-glibc217-x86_64.tar.bz"
        with tarfile.open(blender_archive, "r") as tar:
            tar.extractall(build_dir)
    assert isdir(blender_dir)

    py_dir = join(blender_dir, "2.81/python")
    py_bin = join(py_dir, "bin/python3.7m")
    check_call([py_bin, "-m", "ensurepip"])
    check_call([py_bin, "-m", "pip", "install", "attrs", "zmq"])

    blender_bin = join(blender_dir, "blender")

    drake_dir = join(build_dir, "drake")
    if args.with_drake and not isdir(drake_dir):
        drake_archive = join(build_dir, "drake.tar.gz")
        drake_url = "https://drake-packages.csail.mit.edu/drake/nightly/drake-20191007-bionic.tar.gz"
        print(drake_archive)
        if not isfile(drake_archive):
            check_call(["wget", drake_url, "-O", drake_archive])
        with tarfile.open(drake_archive, "r") as tar:
            tar.extractall(build_dir)
        check_call(
            f"python3 -m virtualenv -p python3 {drake_dir} --system-site-packages",
            shell=True)
        check_call(f"{drake_dir}/bin/pip install imageio", shell=True)

    setup_script = join(build_dir, "setup.bash")
    with open(setup_script, "w") as f:
        f.write(f"""
    export BLENDER_PATH={blender_bin}
    export PYTHONPATH={dirname(cur_dir)}:${{PYTHON_PATH}}
    """)
        if args.with_drake:
            f.write(f"""
    source {drake_dir}/bin/activate
    """)

    if not isdir(join(cur_dir, "data/test_objs")):
        check_call([join(cur_dir, "data/get_example_assets.sh")])

    print(f"""

    To use blender_server functionality:

        source ./build/setup.bash

    """)


if __name__ == "__main__":
    main()
