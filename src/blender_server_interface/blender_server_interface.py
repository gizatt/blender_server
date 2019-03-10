import imageio
import io
import numpy as np
import os
import sys
import zmq


class BlenderServerInterface():
    def __init__(self, port=5556):
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.connect("tcp://localhost:%s" % str(self.port))

    def _construct_remote_call_json(self, func, **kwargs):
        return {
            "func": func,
            "args": kwargs
        }

    def send_remote_call(self, func, **kwargs):
        self.socket.send_json(
            self._construct_remote_call_json(
                func, **kwargs))
        print("Resp: ", self.socket.recv())

    def render_image(self, camera_name):
        self.socket.send_json(
            {"func": "render_and_return_image_bytes",
             "args": {"camera_name": camera_name}})
        im_buffer = self.socket.recv()
        return imageio.imread(io.BytesIO(im_buffer))
