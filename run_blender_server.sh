set -eux -o pipefail
$BLENDER_PATH \
    ./data/blender_template/blank.blend -b --python ./blender_server.py
