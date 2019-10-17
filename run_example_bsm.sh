#!/bin/bash
set -eux -o pipefail

blender ./data/blender_template/blank.blend -b --python ./render_main_bsm.py
