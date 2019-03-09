import bpy
import blender_scripts.object_manip as object_manip
import blender_scripts.utils as utils
import attr


@attr.s
class RigidBodyProperties:
    """
    The rigid body properties for rigid body
    """
    use_margin = True
    collision_margin = 0
    # These are for active body only
    linear_damping = 0.9
    angular_damping = 0.9


def set_as_active(name_key: str):
    """
    Remove all other object and set the given one
    as the active object
    :param name_key: The key of the object
    :return:
    """
    obj = object_manip.get_bpyobj_by_name(name_key)
    if utils.is_new_api():
        bpy.context.view_layer.objects.active = obj
    else:
        bpy.context.scene.objects.active = obj


def enable_physics_rigidbody(name_key: str, type='ACTIVE', properties = RigidBodyProperties()):
    """
    Given the object, enable physical simulation for that
    object for either active or passive
    :param name_key:
    :param type: 'ACTIVE', 'PASSIVE'
    :param properties: control the virtual boundary for collision detection
    :return:
    """
    set_as_active(name_key)
    bpy.ops.rigidbody.object_add(type=type)

    # Get the rigid body and setup properties
    obj = object_manip.get_bpyobj_by_name(name_key)
    obj.rigid_body.collision_margin = properties.collision_margin
    obj.rigid_body.linear_damping = properties.linear_damping
    obj.rigid_body.angular_damping = properties.angular_damping


def disable_physics_rigidbody(name_key: str):
    """
    Given the object, remove that body from
    rigid body simulation engine.
    :param name_key:
    :return:
    """
    set_as_active(name_key)
    bpy.ops.rigidbody.object_remove()