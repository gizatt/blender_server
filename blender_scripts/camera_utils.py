import bpy


def get_camera_obj():
    # Check that there is only one camera in the scene
    bpy.ops.object.select_by_type(type='CAMERA')
    assert len(bpy.context.selected_objects) is 1

    # Return the camera object
    return bpy.data.objects['Camera']


def set_camera_location(x: float, y: float, z: float):
    camera = get_camera_obj()
    camera.location[0] = x
    camera.location[1] = y
    camera.location[2] = z


def set_camera_quat(qx: float, qy: float, qz: float, qw: float):
    camera = get_camera_obj()
    camera.rotation_mode = 'QUATERNION'
    camera.rotation_quaternion = (qx, qy, qz, qw)