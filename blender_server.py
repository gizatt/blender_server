import bpy
import numpy as np
import os
import sys
import zmq

'''
  The server provides minimal operating functionality:

  Remote command interface, for remotely piloting
  any function in the blender scene management module:
  Send a JSON dict of this form:
  {
    func: <string of the function name>,
    args: {arg dict}
  }

  returns "Success" if it works as a string,
  "Failure: " + error details otherwise. Error
  handling definitely not guaranteed...

  If you call "render_and_return_image_bytes", it'll
  return the image bytes (jpeg bytes) instead of "Success".
'''

import blender_scripts.blender_scene_management as bsm

if __name__ == '__main__':
    bsm.initialize_scene()

    print(sys.argv)
    if len(sys.argv) > 5:
        port = sys.argv[-1]
    else:
        port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:%s" % port)

    while True:
        try:
            msg = socket.recv_json()
            print("Acting on cmd: ")
            print(msg)

            cmd_name = msg["func"]
            args = msg["args"]

            ret = getattr(bsm, cmd_name)(**args)

            if cmd_name == "render_and_return_image_bytes":
                socket.send(ret)
            else:
                socket.send(b"Success")

        except zmq.ZMQError:
            pass
        except Exception as e:
            print("Unhandled exception from server: ", e)
            socket.send(b"Failure: " + str(e).encode('ascii'))
