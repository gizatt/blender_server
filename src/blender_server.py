import bpy
import numpy as np
import os
import sys
import zmq

# Literally all this does right now is receive JSON
# message of the form
# {
#   command_name: <string>,
#   args: {arg dict}
# }
# and call bsm.command_name(**args)
# as a way of remotely operating
# the bsm.

# TODO(gizatt) Why is this required?
# Why isn't current launch dir included in sys.path?
print(sys.path)
sys.path.append("/home/gizatt/tools/blender_server/src/")
import blender_scripts.blender_scene_management as bsm

if __name__ == '__main__':
    bsm.initialize_scene()

    port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.bind("tcp://*:%s" % port)

    while True:
        try:
            msg = socket.recv_json()
            print("Acting on cmd: ")
            print(msg)

            cmd_name = msg["func"]
            args = msg["args"]

            getattr(bsm, cmd_name)(**args)

            socket.send(b"Success")
        
        except KeyError as e:
            print("KeyError: ", e)
            socket.send(b"Failure: " + str(e))


