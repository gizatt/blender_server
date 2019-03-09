import bpy
import blender_scripts.utils as utils


def remove_all_lights():
    if utils.is_new_api():
        bpy.ops.object.select_by_type(type='LIGHT')
    else:
        bpy.ops.object.select_by_type(type='LAMP')
    bpy.ops.object.delete(use_global=False)


def add_light(light_type: str = 'POINT') -> str:
    """
    Add a light of the given type to the scene, return
    the name key of the newly added light
    :param light_type:
    :return: The named key used to index the object
    """
    if utils.is_new_api():
        bpy.ops.object.light_add(type=light_type)
    else:
        bpy.ops.object.lamp_add(type=light_type)
    light_obj = bpy.context.selected_objects[0]

    # Enable contact shadows
    if utils.is_new_api():
        light_obj.data.use_contact_shadow = True

    # Return the name
    return light_obj.name


def get_lightobj_by_name(name_key: str):
    """
    Given the name_key returned by add_light, find the object
    for the light
    """
    import blender_scripts.object_manip as object_manip
    return object_manip.get_bpyobj_by_name(name_key)


# Below are a series of method to setup and properties of the light objects
def set_light_location(name_key: str, x: float, y: float, z: float):
    light_obj = get_lightobj_by_name(name_key)
    light_obj.location[0] = x
    light_obj.location[1] = y
    light_obj.location[2] = z


def set_light_energy(name_key: str, energy: float = 1.0):
    light_obj = get_lightobj_by_name(name_key).data
    light_obj.energy = energy
