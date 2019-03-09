import bpy
import os


def save_current_scene(save_path: str):
    """
    Save the current scene configuration at the given directory
    """
    bpy.ops.file.autopack_toggle()
    bpy.ops.wm.save_as_mainfile(filepath=save_path)


def get_genda_root() -> str:
    """
    :return: The absolute path to the genda directory
    """
    exe_path = os.path.dirname(os.path.abspath(__file__))
    proj_path = exe_path
    for _ in range(3):
        proj_path = os.path.abspath(os.path.join(proj_path, os.pardir))
    return proj_path


def get_data_root() -> str:
    return os.path.join(get_genda_root(), 'data')


def is_old_api() -> bool:
    return bpy.app.version < (2, 80, 0)


def is_new_api() -> bool:
    return not is_old_api()