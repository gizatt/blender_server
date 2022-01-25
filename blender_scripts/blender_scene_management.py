import bpy
import numpy as np
import os
import time

import blender_scripts.utils as blender_utils
import blender_scripts.renderer_option as renderer_option

'''

This file provides the set of high-level scene management functions
necessary to setup and render scenes in Blender. Each of these functions
is in scope to be called remotely via the blender server interface: e.g.,
after setting up a Blender server with

bsi = BlenderServerInterface(zmq_url="tcp://127.0.0.1:5556")

You can initialize a scene by calling

success = bsi.send_remote_call("initialize_scene")

and then setup a texture with

metal26_path = "./data/test_pbr_mats/Metal26/Metal26"
bsi.send_remote_call(
    "register_material",
    name="metal26",
    material_type="CC0_texture",
    path=metal26_path)

and so on until you have a complete scene, at which point you can
submit a request to render and finally get an image.
'''


loaded_environment_nodes = {}
def initialize_scene():
    '''
        Setup a clean, empty scene.
    '''
    print("\n\n**************Before init: ")
    bpy.ops.wm.memory_statistics()
    global loaded_environment_nodes
    loaded_environment_nodes = {}

    bpy.ops.wm.read_homefile(use_empty=True)
    print("**************After init: ")
    bpy.ops.wm.memory_statistics()

    # Add a world
    bpy.ops.world.new()
    bpy.context.scene.world = bpy.data.worlds[0]
    # Default background color: flat black
    bpy.context.scene.world.node_tree.nodes["Background"].inputs[0].default_value = [0, 0, 0, 1.]

def register_material(name, material_type, path=None, color=None):
    '''
    Register a material type (specified by string) under the given
    unique name.

    Legal material_types:

        Emission node type:
        - "emission", with color being a 3-element float vector
        ranging [0,1] (an RGB specification).

        Principled BSDF:
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

    # Principled BSDF types
    if material_type == "color":
        mat_node = nodes.new(type='ShaderNodeBsdfPrincipled')
        print("Using base color ", color)
        mat_node.inputs["Base Color"].default_value = color
        mat_node_output = mat_node.outputs['BSDF']

    elif material_type == "CC0_texture":
        mat_node = nodes.new(type='ShaderNodeBsdfPrincipled')
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
                links.new(texture_image_node.inputs['Vector'],
                          uvmap_node.outputs['UV'])
                if input_name is "Normal":
                    # Needs to be remapped into tangent space
                    nmap_node = nodes.new(type='ShaderNodeNormalMap')
                    texture_image_node.image.colorspace_settings.name = 'Non-Color'
                    links.new(mat_node.inputs[input_name],
                              nmap_node.outputs['Normal'])
                    links.new(nmap_node.inputs['Color'],
                              texture_image_node.outputs['Color'])
                else:
                    links.new(mat_node.inputs[input_name],
                              texture_image_node.outputs['Color'])
        
            else:
                print("Not using path %s" % texture_full_path)
        mat_node_output = mat_node.outputs['BSDF']
    elif material_type == "color_texture":
        mat_node = nodes.new(type='ShaderNodeBsdfPrincipled')
        print("Using path %s" % path)
        texture_image_node = populate_image_node_from_file(
            nodes, path)
        uvmap_node = nodes.new("ShaderNodeUVMap")
        uvmap_node.uv_map = 'UVMap'
        links.new(mat_node.inputs["Base Color"],
                  texture_image_node.outputs['Color'])
        links.new(texture_image_node.inputs['Vector'],
                  uvmap_node.outputs['UV'])
        mat_node_output = mat_node.outputs['BSDF']

    # emission type
    elif material_type == "emission":
        mat_node = nodes.new(type='ShaderNodeEmission')
        print("Using base color for emission", color)
        mat_node.inputs[0].default_value = color
        mat_node_output = mat_node.outputs['Emission']

    else:
        raise IllegalArgumentException(
            "Invalid material_type %s" % material_type)

    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    links.new(output_node.inputs['Surface'], mat_node_output)

def register_object(name, type,
                    path=None,
                    location=None,
                    quaternion=None,
                    scale=None,
                    material=None,
                    **kwargs):
    '''
        Register an object of a specified type with the given
        unique name.

        Type can be "obj", "cube", "sphere", or "cylinder."
        If type is "obj", must provide a path to an obj file.

        Location should be a 3-element list of floats, if provided.
        Quaternion should be a 4-element list of floats, if provided.
        Scale should be a 3-element list of floats, if provided.
        Material should be the unique name of a registered material,
        if provided.
    '''
    if type == "obj":
        assert(path is not None)
        bpy.ops.import_scene.obj(filepath=path)
    elif type == "cube":
        bpy.ops.mesh.primitive_cube_add()
    elif type == "sphere":
        bpy.ops.mesh.primitive_uv_sphere_add()
    elif type == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add()
    else:
        raise NotImplementedException("Unsupported shape %s" % type)
    obj = bpy.context.selected_objects[0]
    obj.name = name
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
        #obj.active_material = bpy.data.materials[material]
        if obj.data.materials:
            obj.data.materials[0] = bpy.data.materials[material]
        else:
            obj.data.materials.append(bpy.data.materials[material])
    for key, value in kwargs.items():
        setattr(obj, key, value)

def apply_modifier_to_object(name, type, **kwargs):
    '''
        Applied the given modifier type to the object.
        Legitimate object modifiers are defined by Blender, and can be one
        of: ["WIREFRAME", TODO(gizatt)(What *are* the others, anyway?)]
    '''

    if len(bpy.data.objects[name].modifiers) != 0:
        raise NotImplementedException("In-flight modifier never cleared. This shouldn't happen.")

    bpy.data.objects[name].modifiers.new(name='mod', type=type)
    for arg_name, arg_value in kwargs.items():
        setattr(bpy.data.objects[name].modifiers['mod'], arg_name, arg_value)
    bpy.ops.object.modifier_apply(modifier='mod')

def register_light(name,
                   type="POINT",
                   location=None,
                   quaternion=None,
                   energy=None):
    '''
        Registers a light of the specified type under the given unique name.
        Location should be a 3-element list of floats, if provided.
        Quaternion should be a 4-element list of floats, if provided
        Eenrgy should be a float, if provided.
    '''
    bpy.ops.object.light_add(type=type)
    obj = bpy.context.selected_objects[0]
    obj.name = name
    obj.data.use_contact_shadow = True
    if location is not None:
        obj.location = location
    if quaternion is not None:
        obj.rotation_mode = 'QUATERNION'
        obj.rotation_quaternion = quaternion
    if energy is not None:
        obj.data.energy = energy

def update_object_parameters(name,
                             **kwargs):
    '''
        Gets the object of the given name from the Blender scene and sets
        its attributes according to the key:value pairs in kwargs.

        Most useful for updating object transformations by invoking like:
            bsi.send_remote_call(
                "update_object_parameters",
                name=<object name>.
                location=[0., 1., 2.],
                rotation_mode='QUATERNION',
                rotation_quaternion=[1., 0., 0., 0.]
            )

        or hiding an object like:
            bsi.send_remote_call(
                "update_object_parameters",
                name=<object name>,
                hide_render=True
            )
    '''
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

def update_material_parameters(name,
                               type=None,
                               **kwargs):
    '''
        Updates the attributes of a material of of the given name
        and corresponding type using the given key-value pairs in
        kwargs.

        Hacky, see code for why. The recipes listed here should
        work, but see implementation to understand what this is
        actually doing on the blender side.

        Useful for updating the color of a material like:
            bsi.send_remote_call(
                "update_material_parameters",
                type="Principled BSDF",
                name=<material name>,
                **{"Base Color": [1., 0.5, 0.25],
                   "Specular": 0.0,
                    "blend_method": "ADD"}
            )
    '''
    # Much hackier than objects, as material live in a node tree...
    try:
        obj = bpy.data.materials[name]
    except Exception as e:
        raise e

    for arg_name, arg_value in kwargs.items():
        if type:
            obj.node_tree.nodes[type].inputs[arg_name].default_value = arg_value
        else:
            setattr(obj, arg_name, arg_value)

# None to detach, otherwise provide a path to an env map file.
def set_environment_map(path):
    '''
        Sets the environment map to the one at the given path.
    '''
    global loaded_environment_nodes
    # Overwrite the world background node input.
    nodes = bpy.context.scene.world.node_tree.nodes
    links = bpy.context.scene.world.node_tree.links

    for l in nodes['Background'].inputs['Color'].links:
        links.remove(l)

    if path is None:
        return

    if path not in loaded_environment_nodes.keys():
        enode = nodes.new("ShaderNodeTexEnvironment")
        enode.image = bpy.data.images.load(path, check_existing=True)
        loaded_environment_nodes[path] = enode

    links.new(loaded_environment_nodes[path].outputs['Color'],
              nodes['Background'].inputs['Color'])


def register_camera(name,
                    location=None,
                    quaternion=None,
                    angle=None):
    '''
        Registers a camera under the given unique name.
        Sets the camera location to the given 3-float list, if provided.
        Sets camera rotation to the given 4-float list, if provided.
        Sets the camera diagonal angle (FOV) to the given float, if provided.
    '''
    bpy.ops.object.camera_add()
    cam = bpy.context.active_object
    cam.name = name
    cam.data.name = name
    print(cam, cam.name)
    if location is not None:
        cam.location = location
    if quaternion is not None:
        cam.rotation_mode = 'QUATERNION'
        cam.rotation_quaternion = quaternion
    if angle is not None:
        cam.data.angle = angle
        

def configure_rendering(camera_name,
                        resolution=None,
                        file_format=None,
                        filepath=None,
                        configure_for_masks=False,
                        taa_render_samples=None,
                        cycles=False):
    '''
        Sets up rendering for the specified already-registered camera.

        resolution: 2-element list of ints, [width, height]
        file_format: File format string
        filepath: Output image file path
        configure_for_masks: bool, whether to render label image masks.
        taa_render_samples: int
        cycles: bool, whether to use cycles or eevee.
    '''

    if configure_for_masks:
        cycles = False
    if cycles:
        renderer_config = renderer_option.CyclesRendererOption()
    else:
        renderer_config = renderer_option.EeveeRendererOption()


    if resolution is not None:
        resolution_x, resolution_y = resolution
        renderer_config.resolution_x = resolution_x
        renderer_config.resolution_y = resolution_y

    if taa_render_samples:
        renderer_config.taa_render_samples = taa_render_samples

    if configure_for_masks:
        renderer_config.use_ssr = False
        renderer_config.use_soft_shadow = False
        renderer_config.use_ambient_occlusion = False
        bpy.data.scenes["Scene"].display_settings.display_device = "None"
        bpy.data.scenes["Scene"].sequencer_colorspace_settings.name = "Non-Color"
    else:
        renderer_config.use_ssr = True
        renderer_config.use_soft_shadow = True
        renderer_config.use_ambient_occlusion = True
        bpy.data.scenes["Scene"].display_settings.display_device = "sRGB"
        bpy.data.scenes["Scene"].sequencer_colorspace_settings.name = "Linear"

    if cycles:
        renderer_option.setup_and_use_cycles(
            renderer_config, camera_name=camera_name)
    else:
        renderer_option.setup_and_use_eevee(
            renderer_config, camera_name=camera_name)

    if file_format is not None:
        bpy.context.scene.render.image_settings.file_format = file_format

    if filepath is not None:
        bpy.context.scene.render.filepath = filepath


def save_current_scene(path):
    '''
        Saves the current scene as a .blend at the given path.
    '''
    prefix_except_file = os.path.split(path)[0]
    os.system("mkdir -p %s" % prefix_except_file)
    blender_utils.save_current_scene(path)


def render(camera_name, write_still=True, filepath=None):
    '''
        Renders the scene to the specified filepath.
    '''
    if filepath is not None:
        old_filepath = bpy.context.scene.render.filepath
        bpy.context.scene.render.filepath = filepath
    bpy.context.scene.camera = bpy.context.scene.objects[camera_name]
    bpy.ops.render.render(use_viewport=False, write_still=write_still)
    if filepath is not None:
        bpy.context.scene.render.filepath = old_filepath


def render_and_return_image_bytes(camera_name, filepath=None):
    '''
        Renders to the given filepath and returns the image bytes
        by reading from a saved image file.

        TODO(gizatt) This is horribly inefficient because of the file
        writing, but getting an image from blender has proven tricky.
        Must improve here.
    '''
    bpy.context.scene.camera = bpy.context.scene.objects[camera_name]

    if filepath is None:
        output_file = "/tmp/blender_server_%d.jpg" % (
            time.time() * 1000 * 1000)
    else:
        output_file = filepath
    old_filepath = bpy.context.scene.render.filepath
    bpy.context.scene.render.filepath = output_file
    bpy.ops.render.render(use_viewport=False, write_still=True)
    bpy.context.scene.render.filepath = old_filepath

    with open(output_file, 'rb') as f:
        output_stuff = f.read()
    return output_stuff


'''
    Everything below here are internal functions used by the scene
    management functions, and shouldn't be called through the server
    interface.
'''

def populate_image_node_from_file(nodes, path):
    image = bpy.data.images.load(path, check_existing=True)
    texture_img_node = nodes.new(type='ShaderNodeTexImage')
    texture_img_node.image = image
    texture_img_node.projection = 'FLAT'
    return texture_img_node


