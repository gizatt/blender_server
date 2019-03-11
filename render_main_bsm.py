import bpy
import numpy as np
import os
import sys

# TODO(gizatt) Why is this required?
# Why isn't current launch dir included in sys.path?
print(sys.path)
sys.path.append("/home/gizatt/tools/blender_server/")
import blender_scripts.blender_scene_management as bsm

if __name__ == '__main__':
    bsm.initialize_scene()

    metal26_path = "./data/test_pbr_mats/Metal26/Metal26"
    bsm.register_material("metal26",
                          material_type="CC0_texture",
                          path=metal26_path)

    bsm.register_object("obj_table",
                        type="cube",
                        location=[0, 0, -0.45],
                        scale=[1.0, 1.0, 0.25],
                        material="metal26")


    object_classes = [
            "./data/test_objs/ycb/004_sugar_box/google_16k/",
            "./data/test_objs/ycb/035_power_drill/google_16k/"
    ]
    objs = []
    for i, obj_base_path in enumerate(object_classes):
        tex_path = obj_base_path + "texture_map.png"
        bsm.register_material("obj_%d_color" % i,
                              material_type="color_texture",
                              path=tex_path)
        for k in range(2):
            obj_name = "obj_%d_%d" % (i, k)
            bsm.register_object(obj_name,
                                type="obj",
                                path=obj_base_path + "textured.obj",
                                scale=[1., 1., 1.],
                                material="obj_%d_color" % i)
            objs.append(obj_name)


    bsm.register_camera("cam_1",
                        location=[-.54, -.54, .12],
                        quaternion=[-0.677, -0.604, 0.242, 0.365])

    bsm.configure_rendering(camera_name='cam_1',
                            resolution=[1920, 1200],
                            file_format="JPEG")

    env_map_path = "./data/env_maps/aerodynamics_workshop_4k.hdr"
    bsm.set_environment_map(path=env_map_path)

    bsm.save_current_scene('./out/save.blend')

    for i in range(10):
        for obj_tmp in objs:
            loc = [np.random.uniform(-0.4, 0.4),
                   np.random.uniform(-0.4, 0.4),
                   0.]
            quat = np.random.uniform(-1, 1, size=4)
            quat /= np.linalg.norm(quat)
            bsm.update_parameters(
                obj_tmp,
                location=loc,
                rotation_mode='QUATERNION',
                rotation_quaternion=quat)

        bsm.configure_rendering(
            camera_name='cam_1', 
            filepath="./out/pic%0.2d.jpg" % i)
        bsm.render("cam_1")
        print("# of image bytes: %d" % len(bsm.render_and_return_image_bytes("cam_1")))
