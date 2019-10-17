#!/usr/bin/env python3
import argparse
from subprocess import check_call
import os
from os.path import dirname, abspath, join, isdir, isfile, expanduser
import tarfile
from textwrap import dedent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--with_drake", action="store_true")
    parser.add_argument("--with_jupyter", action="store_true")
    args = parser.parse_args()

    cur_dir = dirname(abspath(__file__))
    build_dir = join(cur_dir, "build")
    os.makedirs(build_dir, exist_ok=True)

    archive_dir = expanduser("~/Downloads")

    # Extract.
    blender_base = "blender-2.80-linux-glibc217-x86_64"
    blender_dir = join(build_dir, blender_base)
    blender_url = f"https://mirror.clarkson.edu/blender/release/Blender2.80/{blender_base}.tar.bz2"
    blender_archive = join(archive_dir, f"{blender_base}.tar.bz2")
    if not isdir(blender_dir):
        print(blender_archive)
        if not isfile(blender_archive):
            check_call(["wget", blender_url, "-O", blender_archive])
            # sha256sum = "7276216e95b28c74306cec21b6d61e202cbe14035a15a77dbc45fe9d98fca7aa  blender-2.80-linux-glibc217-x86_64.tar.bz"
        with tarfile.open(blender_archive, "r") as tar:
            tar.extractall(build_dir)
    assert isdir(blender_dir)

    py_dir = join(blender_dir, "2.80/python")
    py_bin = join(py_dir, "bin/python3.7m")
    py_pkgs_dir = join(py_dir, "lib/python3.7/site-packages")
    check_call([py_bin, "-m", "ensurepip"])
    check_call([py_bin, "-m", "pip", "install", "attrs", "zmq"])

    blender_bin = join(blender_dir, "blender")

    drake_dir = join(build_dir, "drake")
    if args.with_drake and not isdir(drake_dir):
        drake_base = "drake-20191007-bionic"
        drake_archive = join(archive_dir, f"{drake_base}.tar.gz")
        drake_url = f"https://drake-packages.csail.mit.edu/drake/nightly/{drake_base}.tar.gz"
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
        f.write(dedent(f"""\
        export PATH={dirname(blender_bin)}:{dirname(py_bin)}:${{PATH}}
        export PYTHONPATH={dirname(cur_dir)}:${{PYTHON_PATH}}
        export JUPYTER_PATH={build_dir}
        """))
        if args.with_drake:
            f.write(dedent(f"""\
            source {drake_dir}/bin/activate
            """))

    if not isdir(join(cur_dir, "data/test_objs")):
        check_call([join(cur_dir, "data/get_example_assets.sh")])

    kernel_dir = join(build_dir, "kernels", "blender_python3")
    with_jupyter = (args.with_jupyter or isdir(kernel_dir))
    if with_jupyter:
        check_call([
            py_bin, "-m", "pip", "install",
            "jupyter",
            # For whatever reason, Blender's NumPy isn't picked up by PIP.
            # Reinstall the exact same version.
            "numpy==1.15.0",
            # For plottig in notebooks.
            "matplotlib",
            # For visualizing images.
            "Pillow",
        ])
        os.makedirs(kernel_dir, exist_ok=True)
        with open(join(kernel_dir, "kernel.json"), "w") as f:
            # To retrieve pixels from image data.
            with_window = '"--window-geometry", "100", "100", "200", "200"'
            # Otherwise.
            without_window = '"--background"'
            # "-b",  # Need foreground for rendering...
            f.write(dedent(f"""\
            {{
             "argv": [
              "blender",
              {without_window},
              "--python",
              "{cur_dir}/blender_ipykernel_launcher.py",
              "--",
              "-f",
              "{{connection_file}}"
             ],
             "display_name": "Blender Python 3",
             "language": "python"
            }}
            """))

    print(dedent(f"""\

    To use blender_server functionality:

        source ./build/setup.bash
    """))
    if with_jupyter:
        print(dedent(f"""\
        To use notebooks, run `jupyter notebook` (which will be shadowed), and
        ensure your notebook uses the "Blender Python 3" kernel.
        """))


if __name__ == "__main__":
    main()
