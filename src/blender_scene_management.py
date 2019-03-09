import bpy

import object_manip
import utils
import lighting_utils
import camera_utils
import texture_utils
import renderer_option
import physics_utils

def initialize_scene():

    lighting_utils.remove_all_lights()
