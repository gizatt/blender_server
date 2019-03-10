import bpy
import numpy as np
import os
import sys

# TODO(gizatt) Why is this required?
# Why isn't current launch dir included in sys.path?
print(sys.path)
sys.path.append("/home/gizatt/tools/blender_server/src/")
import blender_scripts.blender_scene_management as bsm

if __name__ == '__main__':
    bsm.initialize_scene()

    metal26_path = "../data/test_pbr_mats/Metal26/Metal26"
    bsm.register_material("metal26",
                          material_type="CC0_texture",
                          path=metal26_path)

    plane_path = "../data/objects/plane.obj"
    bsm.register_object("obj_table",
                        model_path=plane_path,
                        location=[0, 0, -0.35],
                        scale=[0.1, 0.05, 0.1],
                        material="metal26")

    objs = []
    for i, obj_base_path in enumerate([
            os.path.join(data_root, "test_objs/ycb/004_sugar_box/google_16k/"),
            os.path.join(data_root, "test_objs/ycb/035_power_drill/google_16k/")]):
        tex_path = obj_base_path + "texture_map.png"
        bsm.register_material("obj_%d_color" % i,
                              material_type="color_texture",
                              path=tex_path)
        bsm.register_object("obj_%d" % i,
                            path=obj_base_path + "textured.obj",
                            scale=[1., 1., 1.],
                            material="obj_%d_color" % i)
        objs.append("obj_%d" % i)

    bsm.register_camera("cam_1",
                        location=[-.54, -.54, .12],
                        quaternion=[-0.677, -0.604, 0.242, 0.365])

    bsm.configure_rendering(resolution_x=1920, resolution_y=1200,
                            file_format="JPEG")

    env_map_path = "../data/env_maps/aerodynamics_workshop_4k.hdr"
    bsm.register_environment_map("env_map", path=env_map_path)

    bsm.save_current_scene('./out/save.blend')

    for i in range(10):
        for obj_tmp in objs:
            bsm.update_parameters(
                obj_tmp,
                location=np.random.uniform(-0.4, 0.4),
                quaternion=np.random.uniform(-0.4, 0.4))

        bsm.configure_rendering(filepath="./out/pic%0.2d.jpg" % i)
        bsm.render()
