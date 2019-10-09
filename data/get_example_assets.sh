#!/bin/bash
set -eux -o pipefail

cd $(dirname $0)
wget http://people.csail.mit.edu/gizatt/data/blender_server/example_assets.tar.gz
tar -xvzf example_assets.tar.gz --strip-components=1
rm example_assets.tar.gz

./ycb_downloader.py
