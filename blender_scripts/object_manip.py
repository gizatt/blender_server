import bpy
import sys
from typing import Set


class ObjectMap:
    """
    The new blender interface doesn't provide easy access
    from object to their bounded image (texture).
    Thus do it ourselves
    """
    def __init__(self):
        self._obj2image = {}

    @staticmethod
    def _get_current_images() -> Set[str]:
        current_images = set()
        for i in range(len(bpy.data.images)):
            image_i = bpy.data.images[i].name
            current_images.add(image_i)
        return current_images

    def add_object_to_scene(self, obj_path: str, name=None) -> str:
        current_images = ObjectMap._get_current_images()

        # Add the object using bpy methods
        bpy.ops.import_scene.obj(filepath=obj_path)
        imported_obj = bpy.context.selected_objects[0]
        if name is not None:
            imported_obj.name = name

        # Check the new image sets
        if len(bpy.data.images) == len(current_images):
            pass
        else:
            assert len(bpy.data.images) == len(current_images) + 1
            # Get the new image
            new_img = ''
            for i in range(len(bpy.data.images)):
                image_i = bpy.data.images[i].name
                if image_i not in current_images:
                    new_img = image_i
                    break

            # Insert to our map
            self._obj2image[imported_obj.name] = new_img

        # The global return
        return imported_obj.name

    def get_image_key(self, name_key: str) -> str:
        if name_key in self._obj2image:
            return self._obj2image[name_key]
        else:
            return None

    def object_has_image_texture(self, name_key: str) -> bool:
        return name_key in self._obj2image


# The singleton
object_map = ObjectMap()


def import_obj_model(obj_path: str, name=None) -> str:
    """
    :param obj_path: The path to the obj model
    :return: The name of the object as an index
    """
    return object_map.add_object_to_scene(obj_path, name=name)


def object_has_image_texture(name_key: str) -> bool:
    """
    Does this object is created with texture image and
    uv map.
    """
    return object_map.object_has_image_texture(name_key)


def get_object_image_key(object_key: str) -> str:
    """
    Return the key to index the image that the object is bounded with
    :param object_key: The key of the object
    :return: The key used in bpy.data.images
    """
    return object_map.get_image_key(object_key)


def get_bpyobj_by_name(name_key: str):
    """
    Using the name key to retrieve the object
    :param name_key: The key assigned using import_obj_model
    :return: bpy.data.objects[name_key]
    """
    obj_collect = bpy.context.scene.objects
    if name_key in obj_collect:
        return obj_collect[name_key]
    else:
        sys.exit(-1)


def set_obj_location(name_key: str, x: float, y: float, z: float):
    """
    Set the location of the object in world frame
    :param name_key:
    :param x:
    :param y:
    :param z:
    :return:
    """
    obj = get_bpyobj_by_name(name_key)
    obj.location[0] = x
    obj.location[1] = y
    obj.location[2] = z


def set_obj_quaternion(name_key: str, qx: float, qy: float, qz: float, qw: float):
    obj = get_bpyobj_by_name(name_key)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = (qx, qy, qz, qw)


def set_obj_scale(name_key: str,
                  scale_x: float,
                  scale_y: float = -1, scale_z: float = -1):
    """
    :param name_key:
    :param scale_x:
    :param scale_y:
    :param scale_z:
    :return:
    """
    if scale_y < 0:
        scale_y = scale_x
    if scale_z < 0:
        scale_z = scale_x
    obj = get_bpyobj_by_name(name_key)
    obj.scale[0] = scale_x
    obj.scale[1] = scale_y
    obj.scale[2] = scale_z
