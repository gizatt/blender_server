import attr
import bpy
from blender_scripts.utils import is_old_api
from blender_scripts.object_manip import get_bpyobj_by_name


def remove_from_renderer(obj_name_key: str):
    obj = get_bpyobj_by_name(obj_name_key)
    obj.hide_render = True


def add_to_renderer(obj_name_key: str):
    obj = get_bpyobj_by_name(obj_name_key)
    obj.hide_render = False


@attr.s
class RendererOption:
    resolution_x = 640
    resolution_y = 480
    focal_x_mm = 1.0
    focal_y_mm = 1.0
    principal_x = 640
    principal_y = 480

    def rows(self):
        return self.resolution_y

    def cols(self):
        return self.resolution_x

    def sensor_width_bpy(self):
        return (self.focal_y_mm * self.principal_x) / (self.focal_x_mm * self.principal_y)

    def focal_bpy(self):
        s_u = self.resolution_x / self.sensor_width_bpy()
        return self.focal_x_mm / s_u


def setup_renderer_resolution(option: RendererOption, camera_name='Camera'):
    bpy.data.scenes['Scene'].render.resolution_x = option.resolution_x
    bpy.data.scenes['Scene'].render.resolution_y = option.resolution_y
    bpy.data.scenes['Scene'].render.resolution_percentage = 100

    # Set up the camera intrsinsic
    camera_obj = bpy.context.scene.objects[camera_name]
    camera_obj.data.type = 'PERSP'
    camera_obj.data.lens_unit = 'MILLIMETERS'
    camera_obj.data.lens = option.focal_bpy()
    camera_obj.data.sensor_width = option.sensor_width_bpy()


@attr.s
class CyclesRendererOption(RendererOption):
    # The number of samples per pixel
    num_samples = 64
    use_denoising = True

    # Parameter related to performance but not output
    renderer_tile_x = 128
    renderer_tile_y = 128
    renderer_device = 'GPU'


def setup_and_use_cycles(option: CyclesRendererOption):
    # The general setup code
    setup_renderer_resolution(option)
    bpy.context.scene.render.engine = 'CYCLES'
    if is_old_api():
        bpy.context.scene.render.use_raytrace = True
        bpy.context.scene.render.use_shadows = True

    # The number of samples and denosing
    bpy.data.scenes['Scene'].cycles.samples = option.num_samples
    if is_old_api():
        bpy.context.scene.render.layers[0].cycles.use_denoising = True
    else:
        bpy.context.view_layer.cycles.use_denoising = True

    # Performance parameters
    bpy.data.scenes['Scene'].render.tile_x = option.renderer_tile_x
    bpy.data.scenes['Scene'].render.tile_y = option.renderer_tile_y
    bpy.data.scenes['Scene'].cycles.device = option.renderer_device


@attr.s
class EeveeRendererOption(RendererOption):
    # The member related to ambient occlusion
    use_ambient_occlusion = True

    # The member related to shadow
    use_soft_shadow = True
    shadow_method = 'VSM'
    shadow_cube_size = '1024'


def setup_and_use_eevee(option: EeveeRendererOption, camera_name='Camera'):
    # The general setup code
    assert not is_old_api()
    setup_renderer_resolution(option, camera_name=camera_name)
    bpy.context.scene.render.engine = 'BLENDER_EEVEE'

    # The setup code for ambient occulusion
    bpy.context.scene.eevee.use_gtao = option.use_ambient_occlusion

    # the setup code for shadow
    bpy.context.scene.eevee.shadow_method = option.shadow_method
    bpy.context.scene.eevee.use_soft_shadows = option.use_soft_shadow
    bpy.context.scene.eevee.shadow_cube_size = option.shadow_cube_size
    bpy.context.scene.eevee.taa_render_samples = 16
    bpy.context.scene.eevee.use_ssr = True
