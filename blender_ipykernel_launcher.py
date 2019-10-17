"""Copied from nominally install `ipykernel_launcher.py` file."""

import sys

if __name__ == '__main__':
    if sys.path[0] == '':
        del sys.path[0]

    # For blender, delete all argv up to and including '--'.
    assert '--' in sys.argv
    del sys.argv[1:sys.argv.index('--') + 1]

    from ipykernel import kernelapp as app
    app.launch_new_instance()
