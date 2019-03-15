#!/usr/bin/env python
from __future__ import print_function

import argparse
import os
import getpass

if __name__=="__main__":
    user_name = getpass.getuser()
    parser = argparse.ArgumentParser()

    parser.add_argument("data_dir", type=str, help="Path to root of data folder containing assets that the server will use.")

    parser.add_argument("-p", "--port", type=int, default=5556, help="(optional) zmq port for server")

    parser.add_argument("-c", "--container", type=str, default="blender_server", help="(optional) name of the container")\

    parser.add_argument("-e", "--entrypoint", type=str, default=None, help="(optional) thing to run in container")

    parser.add_argument("--passthrough", type=str, default="", help="(optional) extra string that will be tacked onto the docker run command, allows you to pass extra options. Make sure to put this in quotes and leave a space before the first character")

    args = parser.parse_args()
    source_dir = os.getcwd()

    image_name = 'blender_server'
    print("running docker container derived from image %s" % image_name)

    cmd = "xhost +local:root \n"
    cmd += "nvidia-docker run "
    if args.container:
        cmd += " --name %(container_name)s " % {'container_name': args.container}

    cmd += " -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw "     # enable graphics 

    cmd += " -v %s:/blender_server/data " % args.data_dir
    # Port for the ZMQ server
    cmd += " -p %d:%d " % (args.port, args.port)
    if args.entrypoint is not None:
        cmd += " --entrypoint %s " % args.entrypoint
    cmd += " -e \"BLENDER_SERVER_PORT=%d\" " % (args.port)
    cmd += " " + args.passthrough + " "

    cmd += " --rm "  # remove the image when you exit

    cmd += " -it "
    cmd += image_name
    cmd_endxhost = "xhost -local:root"

    print("command = \n \n", cmd, "\n")
    print("")

    # run the docker image
    print("executing shell command")
    code = os.system(cmd)
    print("Executed with code ", code)
    os.system(cmd_endxhost)
    exit(code != 0)
