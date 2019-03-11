import imageio
import io
import numpy as np
import os
import sys
import zmq


class BlenderServerInterface():
    def __init__(self, zmq_url="tcp://127.0.0.1:5556"):
        self.zmq_url = zmq_url
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.connect(self.zmq_url)

    def _construct_remote_call_json(self, func, **kwargs):
        return {
            "func": func,
            "args": kwargs
        }

    def send_remote_call(self, func, **kwargs):
        self.socket.send_json(
            self._construct_remote_call_json(
                func, **kwargs))
        resp = self.socket.recv()
        success = resp == "Success"
        if not success:
            print(resp)
        return success

    def render_image(self, camera_name, filepath=None):
        self.socket.send_json(
            {"func": "render_and_return_image_bytes",
             "args": {"camera_name": camera_name,
                      "filepath": filepath}})
        im_buffer = self.socket.recv()
        return imageio.imread(io.BytesIO(im_buffer))
