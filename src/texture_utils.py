import bpy
import blender_scripts.object_manip as object_manip
import os


def enable_global_shadeless():
    """
    Disable shade on old version
    """
    from blender_scripts.utils import is_old_api
    for item in bpy.data.materials:
        if is_old_api():
            item.use_shadeless = True


def enable_shader_nodetree(name_key: str):
    """
    By default the node tree is not enabled, this method
    enable it and clear all the nodes
    :param name_key: The key for the object
    :return: no return
    """
    object = object_manip.get_bpyobj_by_name(name_key)
    material = object.active_material
    assert material is not None
    material.use_nodes = True

    # Remove current node
    nodes = get_shader_nodes(name_key)
    for node in nodes:
        nodes.remove(node)


def get_shader_nodetree(name_key: str):
    """
    Given the object name key, return the node
    tree of the object. The nodetree must be enabled
    :param name_key:
    :return: The node tree of this object material
    """
    object = object_manip.get_bpyobj_by_name(name_key)
    material = object.active_material
    assert material is not None
    assert material.use_nodes

    # Ok
    return material.node_tree


def get_shader_nodes(name_key: str):
    return get_shader_nodetree(name_key).nodes


def get_shader_node_links(name_key: str):
    return get_shader_nodetree(name_key).links


def construct_uvmap_node(name_key: str):
    """
    Given the object model, construct the uv map node that
    will be used in the node tree.
    The .obj file must specifiy the texture coordinate
    :param name_key: The key to find the body
    :return:
    """
    nodes = get_shader_nodes(name_key)
    uvmap_node = nodes.new("ShaderNodeUVMap")
    uvmap_node.uv_map = 'UVMap'
    return uvmap_node


def image_node_for_textured_obj(name_key: str):
    """
    For object model with bounded image texture, return the
    image texture node that will be used in node tree
    :param name_key:
    :return:
    """
    # Get the image
    assert object_manip.object_has_image_texture(name_key)
    img_key = object_manip.get_object_image_key(name_key)
    image = bpy.data.images[img_key]

    # Construct the image node
    nodes = get_shader_nodes(name_key)
    texture_img_node = nodes.new(type='ShaderNodeTexImage')
    texture_img_node.image = image
    texture_img_node.projection = 'FLAT'

    # OK
    return texture_img_node


def image_node_from_file(name_key: str, filepath: str):
    # Load the image
    image = bpy.data.images.load(filepath, check_existing=True)

    # Construct the image node
    nodes = get_shader_nodes(name_key)
    texture_img_node = nodes.new(type='ShaderNodeTexImage')
    texture_img_node.image = image
    texture_img_node.projection = 'FLAT'

    # OK
    return texture_img_node


def setup_object_texture(name_key: str, texture_image_node, uvmap_node):
    # Get the nodes
    nodes = get_shader_nodes(name_key)

    # Construct the output
    output_node = nodes.new(type='ShaderNodeOutputMaterial')

    # The bsdf node
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')

    # Link them
    links = get_shader_node_links(name_key)
    links.new(output_node.inputs['Surface'], bsdf_node.outputs['BSDF'])
    links.new(texture_image_node.inputs['Vector'], uvmap_node.outputs['UV'])
    links.new(bsdf_node.inputs['Base Color'], texture_image_node.outputs['Color'])


def setup_diffuse_texture_from_single(obj_1, path):
    enable_shader_nodetree(obj_1)
    nodes = get_shader_nodes(obj_1)
    uvmap_node = construct_uvmap_node(obj_1)

    # Construct the output
    output_node = nodes.new(type='ShaderNodeOutputMaterial')

    # The bsdf node
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')

    # Link them
    links = get_shader_node_links(obj_1)
    links.new(output_node.inputs['Surface'], bsdf_node.outputs['BSDF'])

    texture_image_node = image_node_from_file(obj_1, path)
    links.new(texture_image_node.inputs['Vector'], uvmap_node.outputs['UV'])
    links.new(bsdf_node.inputs["Base Color"],
              texture_image_node.outputs['Color'])


def setup_cc0_texture_from_folder(obj_1, base_path):
    enable_shader_nodetree(obj_1)
    nodes = get_shader_nodes(obj_1)
    uvmap_node = construct_uvmap_node(obj_1)

    possible_completions_for_pbsdf = {
        "Base Color": "_col.jpg",
        "Metallic": "_met.jpg",
        "Normal": "_nrm.jpg",
        "Roughness": "_rgh.jpg"
    }

    # Construct the output
    output_node = nodes.new(type='ShaderNodeOutputMaterial')

    # The bsdf node
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')

    # Link them
    links = get_shader_node_links(obj_1)
    links.new(output_node.inputs['Surface'], bsdf_node.outputs['BSDF'])

    # Link up whatever images we can find for the 
    for input_name in possible_completions_for_pbsdf.keys():
        texture_full_path = base_path + possible_completions_for_pbsdf[input_name]
        if os.path.exists(texture_full_path):
            print("Using path %s" % texture_full_path)
            texture_image_node = image_node_from_file(obj_1, texture_full_path)
            links.new(texture_image_node.inputs['Vector'], uvmap_node.outputs['UV'])
            links.new(bsdf_node.inputs[input_name],
                      texture_image_node.outputs['Color'])
        else:
            print("Not using path %s" % texture_full_path)

    # Support for this is only when using Cycles.
    # No displacement mapping in Eevee yet.
    displacement_full_path = base_path + "_disp.jpg"
    if os.path.exists(displacement_full_path):
        print("Using path %s" % displacement_full_path)
        displacement_image_node = image_node_from_file(
            obj_1, displacement_full_path)
        links.new(displacement_image_node.inputs['Vector'],
                  uvmap_node.outputs['UV'])
        links.new(output_node.inputs["Displacement"],
                  displacement_image_node.outputs['Color'])

    else:
        print("Not using path %s" % displacement_full_path)
