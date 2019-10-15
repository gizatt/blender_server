#!/bin/bash
set -eux -o pipefail

$BLENDER_PATH \
    ./data/blender_template/blank.blend -b --python ./render_main_bsm.py
