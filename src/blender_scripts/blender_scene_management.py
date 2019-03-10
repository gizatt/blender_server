import bpy
import os

import blender_scripts.object_manip as object_manip
import blender_scripts.utils as blender_utils
import blender_scripts.lighting_utils as lighting_utils
import blender_scripts.camera_utils as camera_utils
import blender_scripts.texture_utils as texture_utils
import blender_scripts.renderer_option as renderer_option
import blender_scripts.physics_utils as physics_utils

def initialize_scene():
    lighting_utils.remove_all_lights()

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
        for input_name in possible_completions_for_pbsdf.keys():
            texture_full_path = path + possible_completions_for_pbsdf[input_name]
            if os.path.exists(texture_full_path):
                print("Using path %s" % texture_full_path)
                texture_image_node = populate_image_node_from_file(
                    nodes, texture_full_path)
                links.new(bsdf_node.inputs[input_name],
                          texture_image_node.outputs['Color'])
            else:
                print("Not using path %s" % texture_full_path)
    elif material_type == "color_texture":
        print("Using path %s" % path)
        texture_image_node = populate_image_node_from_file(
            nodes, path)
        links.new(bsdf_node.inputs[input_name],
                  texture_image_node.outputs['Color'])

    else:
        raise IllegalArgumentException(
            "Invalid material_type %s" % material_type)
