import bpy
import numpy as np
import os
import time

import blender_scripts.object_manip as object_manip
import blender_scripts.utils as blender_utils
import blender_scripts.lighting_utils as lighting_utils
import blender_scripts.camera_utils as camera_utils
import blender_scripts.texture_utils as texture_utils
import blender_scripts.renderer_option as renderer_option
import blender_scripts.physics_utils as physics_utils

def initialize_scene():
    bpy.ops.wm.read_homefile(use_empty=True)
    # Add a world
    bpy.ops.world.new()
    bpy.context.scene.world = bpy.data.worlds[0]

def populate_image_node_from_file(nodes, path):
    image = bpy.data.images.load(path, check_existing=True)
    texture_img_node = nodes.new(type='ShaderNodeTexImage')
    texture_img_node.image = image
    texture_img_node.projection = 'FLAT'
    return texture_img_node

def register_material(name, material_type, path=None, color=None):
    '''
    Legal material_types:
        - "color", with color being a 3-element float vector
        ranging [0,1] (an RGB specification).
        - "color_texture": Path must be an image file
        that will be used as the base_color of a
        principled BRDF material.
        - "CC0_texture": Path must be base name (up to the
        material type specialization underscore) of a PBR
        material from CC0 textures, with completions including
        some set of:
            - base_name + "_col.jpg" (base_color)
            - etc, see source code
    '''
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    for node in nodes:
        nodes.remove(node)
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    links.new(output_node.inputs['Surface'], bsdf_node.outputs['BSDF'])

    if material_type == "color":
        print("Using base color ", color)
        bsdf_node.inputs["Base Color"].default_value = color
    elif material_type == "CC0_texture":
        possible_completions_for_pbsdf = {
            "Base Color": "_col.jpg",
            "Metallic": "_met.jpg",
            "Normal": "_nrm.jpg",
            "Roughness": "_rgh.jpg"
        }

        uvmap_node = nodes.new("ShaderNodeUVMap")
        uvmap_node.uv_map = 'UVMap'
        for input_name in possible_completions_for_pbsdf.keys():
            texture_full_path = path + possible_completions_for_pbsdf[input_name]
            if os.path.exists(texture_full_path):
                print("Using path %s" % texture_full_path)
                texture_image_node = populate_image_node_from_file(
                    nodes, texture_full_path)
                links.new(bsdf_node.inputs[input_name],
                          texture_image_node.outputs['Color'])
                links.new(texture_image_node.inputs['Vector'],
                          uvmap_node.outputs['UV'])
        
            else:
                print("Not using path %s" % texture_full_path)
    elif material_type == "color_texture":
        print("Using path %s" % path)
        texture_image_node = populate_image_node_from_file(
            nodes, path)
        uvmap_node = nodes.new("ShaderNodeUVMap")
        uvmap_node.uv_map = 'UVMap'
        links.new(bsdf_node.inputs["Base Color"],
                  texture_image_node.outputs['Color'])
        links.new(texture_image_node.inputs['Vector'],
                  uvmap_node.outputs['UV'])

    else:
        raise IllegalArgumentException(
            "Invalid material_type %s" % material_type)

def register_object(name, path,
                    location=None,
                    quaternion=None,
                    scale=None,
                    material=None,
                    **kwargs):
    object_manip.import_obj_model(path, name=name)
    obj = bpy.context.scene.objects[name]
    if location is not None:
        obj.location = location
    if quaternion is not None:
        obj.rotation_mode = 'QUATERNION'
        obj.rotation_quaternion = quaternion
    if scale is not None:
        if isinstance(scale, list):
            assert(len(scale) == 3)
            obj.scale = scale
        else:
            assert(isinstance(scale, float))
            obj.scale = [scale]*3
    if material is not None:
        obj.active_material = bpy.data.materials[material].copy()
    for key, value in kwargs.items():
        setattr(obj, key, value)

def update_parameters(name,
                      **kwargs):
    try:
        obj = bpy.context.scene.objects[name]
    except Exception as e:
        raise e

    for arg_name, arg_value in kwargs.items():
        try:
            setattr(obj, arg_name, arg_value)
        except ValueError as e:
            print("ValueError setting object [%s]'s attr [%s] to [%s]." % 
                  (name, arg_name, str(arg_value)))
            print("ValueError details: ", e)

def set_environment_map(path):
    # Overwrite the world background node input.
    nodes = bpy.context.scene.world.node_tree.nodes
    links = bpy.context.scene.world.node_tree.links

    enode = nodes.new("ShaderNodeTexEnvironment")
    enode.image = bpy.data.images.load(path)
    links.new(enode.outputs['Color'], nodes['Background'].inputs['Color'])


def register_camera(name,
                    location=None,
                    quaternion=None):
    bpy.ops.object.camera_add()
    cam = bpy.context.active_object
    cam.name = name
    print(cam, cam.name)
    if location is not None:
        cam.location = location
    if quaternion is not None:
        cam.rotation_mode = 'QUATERNION'
        cam.rotation_quaternion = quaternion

def configure_rendering(camera_name,
                        resolution=None,
                        file_format=None,
                        filepath=None):
    if resolution is not None:
        resolution_x, resolution_y = resolution
        renderer_config = renderer_option.EeveeRendererOption()
        renderer_config.resolution_x = resolution_x
        renderer_config.resolution_y = resolution_y
        renderer_option.setup_and_use_eevee(renderer_config,
            camera_name=camera_name)

    if file_format is not None:
        bpy.context.scene.render.image_settings.file_format='JPEG'

    if filepath is not None:
        bpy.context.scene.render.filepath = filepath

def save_current_scene(path):
    prefix_except_file = os.path.split(path)[0]
    os.system("mkdir -p %s" % prefix_except_file)
    blender_utils.save_current_scene(path)

def render(camera_name, write_still=True):
    bpy.context.scene.camera = bpy.context.scene.objects[camera_name]
    bpy.ops.render.render(use_viewport=False, write_still=write_still)

def render_and_return_image_bytes(camera_name):
    bpy.context.scene.camera = bpy.context.scene.objects[camera_name]
    
    output_file = "/tmp/blender_server_%d.jpg" % (time.time() * 1000 * 1000)
    old_filepath = bpy.context.scene.render.filepath
    bpy.context.scene.render.filepath = output_file
    bpy.ops.render.render(use_viewport=False, write_still=True)
    bpy.context.scene.render.filepath = old_filepath

    return open(output_file, 'rb').read()