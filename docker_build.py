#!/usr/bin/env python
from __future__ import print_function

import os

if __name__ == "__main__":

    print("building docker container . . . ")

    print("building docker image named blender_server")
    cmd = "docker build -t blender_server ."

    print("command = \n \n", cmd)
    print("")

    os.system(cmd)
