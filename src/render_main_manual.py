import sys
sys.path.append("/home/gizatt/tools/blender_server/src/")
import blender_scripts.object_manip as object_manip
import blender_scripts.utils as blender_utils
import blender_scripts.lighting_utils as lighting_utils
import blender_scripts.camera_utils as camera_utils
import blender_scripts.texture_utils as texture_utils
import blender_scripts.renderer_option as renderer_option
import blender_scripts.physics_utils as physics_utils
import os
import bpy
import numpy as np

if __name__ == '__main__':
    data_root = "/home/gizatt/tools/blender_server/data/"

    # The plane
    plane_path = os.path.join(data_root, 'test_objs/plane.obj')
    plane_obj = object_manip.import_obj_model(plane_path)
    object_manip.set_obj_location(plane_obj, 0.0, 0.0, -0.35)
    object_manip.set_obj_scale(plane_obj, 0.1, 0.05, 0.1)

    base_path = os.path.join(data_root, "test_pbr_mats/Metal26/Metal26")
    texture_utils.setup_cc0_texture_from_folder(plane_obj, base_path)


    #obj_path = os.path.join(data_root, 'test_objs/rlg_misc_models/meshes/visual/companion_cube.obj')
    #obj_path = os.path.join("/home/gizatt/companion_cube_with_shoddy_uvs.obj")
    #obj_path = os.path.join("/home/gizatt/shittybowl.obj")

    objs = []
    for k in range(5):
        for i, obj_base_path in enumerate([
                os.path.join(data_root, "test_objs/ycb/004_sugar_box/google_16k/"),
                #os.path.join(data_root, "test_objs/ycb/027_skillet/google_16k/"),
                os.path.join(data_root, "test_objs/ycb/035_power_drill/google_16k/")]):
            obj_1 = object_manip.import_obj_model(obj_base_path + "textured.obj")
            object_manip.set_obj_scale(obj_1, 1)
            #base_path = os.path.join(data_root, "test_pbr_mats/Metal07/Metal07")
            texture_utils.setup_diffuse_texture_from_single(obj_1, obj_base_path + "texture_map.png")
            #texture_utils.setup_cc0_texture_from_folder(obj_1, base_path)
            objs.append(obj_1)



    # Another plane
    #floor_obj = object_manip.import_obj_model(plane_path)
    #object_manip.set_obj_scale(floor_obj, 3.0)
    #object_manip.set_obj_location(floor_obj, 0.0, 0.0, -0.4)

    # The texture for plane
    #texture_utils.enable_shader_nodetree(plane_obj)
    #img_path = '/home/wei/Documents/code-ref/scenenet/dataset/shapenet_texture/texture_library/metal_plastic_glass/e58c136d8c4a4a49598a7de9196288be.jpg'
    #plane_img_node = texture_utils.image_node_from_file(plane_obj, img_path)
    #plane_uv_node = texture_utils.construct_uvmap_node(plane_obj)
    #texture_utils.setup_object_texture(plane_obj, plane_img_node, plane_uv_node)

    # The texture for object 1

    # Remove all light and add new light
    lighting_utils.remove_all_lights()
    #light_obj = lighting_utils.add_light()
    #lighting_utils.set_light_location(light_obj, 1.0, 2.0, 3.0)
    #lighting_utils.set_light_energy(light_obj, 1.0)

    # Set the camera location
    camera_utils.set_camera_location(-.54, -.54, .12)
    camera_utils.set_camera_quat(-0.677, -0.604, 0.242, 0.365)

    # The renderer option
    #renderer_config = renderer_option.CyclesRendererOption()
    #renderer_config.num_samples = 10
    #renderer_config.resolution_x = 1920
    #renderer_config.resolution_y = 1200
    #renderer_option.setup_and_use_cycles(renderer_config)
    
    renderer_config = renderer_option.EeveeRendererOption()
    renderer_config.resolution_x = 1920
    renderer_config.resolution_y = 1200
    renderer_option.setup_and_use_eevee(renderer_config)

    bpy.context.scene.world.use_nodes = True
    nt = bpy.data.worlds[bpy.context.scene.world.name].node_tree

    enode = bpy.context.scene.world.node_tree.nodes.new("ShaderNodeTexEnvironment")
    enode.image = bpy.data.images.load(os.path.join(data_root, "aerodynamics_workshop_4k.hdr"))
    nt.links.new(enode.outputs['Color'], nt.nodes['Background'].inputs['Color'])



    # The physical part
    #physics_utils.enable_physics_rigidbody(obj_1)
    #physics_utils.enable_physics_rigidbody(obj_0)
    #physics_utils.enable_physics_rigidbody(plane_obj, type='PASSIVE')
    #physics_utils.disable_physics_rigidbody(obj_1)

    blender_utils.save_current_scene('./tmp/save.blend')

    for i in range(10):
        for obj_1 in objs:
            object_manip.set_obj_location(
                obj_1, np.random.uniform(-0.4, 0.4),
                np.random.uniform(-0.4, 0.4),
                0.0)
            random_quat = np.random.uniform(-1, 1, size=4)
            random_quat /= np.linalg.norm(random_quat)
            object_manip.set_obj_quaternion(obj_1, random_quat[0],
                random_quat[1], random_quat[2], random_quat[3])
        bpy.context.scene.render.image_settings.file_format='JPEG'
        bpy.context.scene.render.filepath = "pic%0.2d.jpg"%i
        bpy.ops.render.render(use_viewport=False, write_still=True)
