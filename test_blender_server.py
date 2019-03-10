import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import zmq
import sys
import time

from blender_server_interface.blender_server_interface import BlenderServerInterface

if __name__ == '__main__':
    bsi = BlenderServerInterface(port=5556)

    bsi.send_remote_call("initialize_scene")

    metal26_path = "./data/test_pbr_mats/Metal26/Metal26"
    bsi.send_remote_call(
        "register_material",
        name="metal26",
        material_type="CC0_texture",
        path=metal26_path)

    plane_path = "./data/test_objs/plane.obj"
    bsi.send_remote_call(
        "register_object",
        name="obj_table",
        path=plane_path,
        location=[0, 0, -0.35],
        scale=[0.1, 0.05, 0.1],
        material="metal26")


    object_classes = [
            "./data/test_objs/ycb/004_sugar_box/google_16k/",
            "./data/test_objs/ycb/035_power_drill/google_16k/"
    ]
    objs = []
    for i, obj_base_path in enumerate(object_classes):
        tex_path = obj_base_path + "texture_map.png"
        bsi.send_remote_call(
            "register_material",
            name="obj_%d_color" % i,
            material_type="color_texture",
            path=tex_path)
        for k in range(1):
            obj_name = "obj_%d_%d" % (i, k)
            bsi.send_remote_call(
                "register_object",
                name=obj_name,
                path=obj_base_path + "textured.obj",
                scale=[1., 1., 1.],
                material="obj_%d_color" % i)
            objs.append(obj_name)


    bsi.send_remote_call(
        "register_camera",
        name="cam_1",
        location=[-.54, -.54, .12],
        quaternion=[-0.677, -0.604, 0.242, 0.365])

    bsi.send_remote_call(
        "configure_rendering",
        camera_name='cam_1',
        resolution=[1920, 1200],
        file_format="JPEG")

    env_map_path = "./data/env_maps/aerodynamics_workshop_4k.hdr"
    bsi.send_remote_call(
        "set_environment_map",
        path=env_map_path)

    bsi.send_remote_call("save_current_scene", path="./out/save.blend")

    plt.figure()
    for i in range(1000):
        for obj_tmp in objs:
            loc = [np.random.uniform(-0.4, 0.4),
                   np.random.uniform(-0.4, 0.4),
                   0.]
            quat = np.random.uniform(-1, 1, size=4)
            quat /= np.linalg.norm(quat)
            bsi.send_remote_call(
                "update_parameters",
                name=obj_tmp,
                location=loc,
                rotation_mode='QUATERNION',
                rotation_quaternion=quat.tolist())

        bsi.send_remote_call(
            "configure_rendering",
            camera_name='cam_1', 
            filepath="./out/pic%0.2d.jpg" % i)
        im = bsi.render_image("cam_1")

        plt.imshow(im)
        plt.pause(0.01)