#!/bin/bash
set -eux -o pipefail

$BLENDER_PATH \
    ./data/blender_template/blank.blend -b -d --python-use-system-env --python ./blender_server.py ${1:-5556}
