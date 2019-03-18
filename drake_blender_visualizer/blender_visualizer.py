"""
Visualizes SceneGraph state using Blender Server.
"""
from __future__ import print_function
import argparse
from copy import deepcopy
import math
import os
import pickle
import re
import time
import warnings
import yaml

import matplotlib.pyplot as plt
import numpy as np

from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.systems.framework import (
    AbstractValue, LeafSystem, PublishEvent, TriggerType
)
from pydrake.systems.rendering import PoseBundle
from pydrake.multibody.plant import ContactResults

from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import (
    ManipulationStation,
    CreateDefaultYcbObjectList)
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import (
    FirstOrderLowPassFilter, TrajectorySource)
from pydrake.trajectories import PiecewisePolynomial
from pydrake.util.eigen_geometry import Isometry3


from blender_server.blender_server_interface.blender_server_interface import BlenderServerInterface


class BoundingBoxBundle(object):
    def __init__(self, num_bboxes=0):
        self.reset(num_bboxes)

    def MakeCopy(self):
        new = BoundingBoxBundle(self.num_bboxes)
        new.scales = deepcopy(self.scales)
        new.colors = deepcopy(self.colors)
        new.poses = [Isometry3(tf) for tf in self.poses]
        return new

    def reset(self, num_bboxes):
        self.num_bboxes = num_bboxes
        self.scales = [[1., 1., 1] for n in range(num_bboxes)]
        self.poses = [Isometry3() for n in range(num_bboxes)]
        self.colors = [[1., 1., 1., 1.] for n in range(num_bboxes)]

    def set_bbox_attributes(
            self, box_i, scale=None, pose=None,
            color=None):
        if scale:
            self.scales[box_i] = scale
        if pose:
            self.poses[box_i] = pose
        if color:
            self.colors[box_i] = color

    def get_num_bboxes(self):
        return self.num_bboxes

    def get_bbox_scale(self, box_i):
        return self.scales[box_i]

    def get_bbox_pose(self, box_i):
        return self.poses[box_i]

    def get_bbox_color(self, box_i):
        return self.colors[box_i]


class BoundingBoxBundleTestSource(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        self.iter = 0
        self.set_name('dummy bbox publisher')
        self._DeclarePeriodicPublish(0.03333, 0.0)

        self.bbox_bundle_output_port = \
            self._DeclareAbstractOutputPort(
                self._DoAllocBboxBundle,
                self._DoCalcAbstractOutput)

    def _DoAllocBboxBundle(self):
        return AbstractValue.Make(BoundingBoxBundle)

    def _DoCalcAbstractOutput(self, context, y_data):
        bbox_bundle = BoundingBoxBundle(2)
        bbox_bundle.set_bbox_attributes(
            0, scale=[0.1, 0.15, 0.2],
            pose=RigidTransform(p=[0.5, 0., 0.1]).GetAsIsometry3(),
            color=[1., 0., 0., 1.])
        bbox_bundle.set_bbox_attributes(
            1, scale=[0.05, 0.05, 0.05],
            pose=RigidTransform(p=[0.4, 0.1, 0.1]).GetAsIsometry3(),
            color=[0., 1., 1., 1.])
        y_data.set_value(bbox_bundle)


class BoundingBoxBundleYamlSource(LeafSystem):
    def __init__(self, log_file, publish_period=0.033333):
        LeafSystem.__init__(self)

        self.iter = 0
        self.set_name('ycb yaml log bbox publisher')
        self.publish_period = publish_period
        self._DeclarePeriodicPublish(self.publish_period, 0.0)

        self.bbox_bundle_output_port = \
            self._DeclareAbstractOutputPort(
                self._DoAllocBboxBundle,
                self._DoCalcAbstractOutput)

        # Load in log and prepare ordered list of the bundles
        # across time
        with open(log_file, "r") as f:
            log_yaml = yaml.load(f)

        self.colors_by_obj = log_yaml["colors"]
        self.sizes_by_obj = log_yaml["sizes"]
        self.bbox_bundle_times = []
        self.bbox_bundle_durations = []
        self.bbox_bundles = []
        for detection_event in log_yaml["detection_events"]:
            t = detection_event["t"]
            self.bbox_bundle_times.append(t)
            self.bbox_bundle_durations.append(detection_event["duration"])
            detected_objs = detection_event["detections"].keys()
            bbox_bundle = BoundingBoxBundle(len(detected_objs))
            for i, obj_name in enumerate(detected_objs):
                assert(obj_name in self.colors_by_obj.keys() and
                       obj_name in self.sizes_by_obj.keys())
                color_float = [float(x)/255. for x
                               in self.colors_by_obj[obj_name]]
                scale = [eval(x) for x in self.sizes_by_obj[obj_name]]
                bbox_bundle.set_bbox_attributes(
                    i, scale=scale,
                    color=color_float,
                    pose=RigidTransform(np.array(
                        detection_event["detections"][obj_name]))
                    .GetAsIsometry3())
            self.bbox_bundles.append((bbox_bundle))
        self.bbox_bundle_times = np.array(self.bbox_bundle_times)
        self.bbox_bundle_durations = np.array(self.bbox_bundle_durations)

    def _DoAllocBboxBundle(self):
        return AbstractValue.Make(BoundingBoxBundle)

    def _DoCalcAbstractOutput(self, context, y_data):
        t = context.get_time()

        fade_time = 0.5
        bundle_absolute_end_times = (
            self.bbox_bundle_times + self.bbox_bundle_durations + fade_time)

        if t > bundle_absolute_end_times[-1]:
            now_ind = len(bundle_absolute_end_times)-1
        else:
            now_ind = np.argmax(bundle_absolute_end_times >= t)

        start_time = self.bbox_bundle_times[now_ind]
        end_time = start_time + self.bbox_bundle_durations[now_ind]

        if t < (start_time - fade_time) or t > (end_time + fade_time):
            y_data.set_value(BoundingBoxBundle(0))
            return
        elif t >= start_time and t <= end_time:
            y_data.set_value(self.bbox_bundles[now_ind])
            return
        elif t > (end_time):
            fade_amt = 1. - (t - end_time) / fade_time
        elif t < start_time:
            fade_amt = 1. - (start_time - t) / fade_time

        faded_bundle = self.bbox_bundles[now_ind].MakeCopy()
        for k in range(faded_bundle.get_num_bboxes()):
            this_color = faded_bundle.get_bbox_color(k)
            this_color = [x * fade_amt for x in this_color]
            faded_bundle.set_bbox_attributes(k, color=this_color)
        y_data.set_value(faded_bundle)


class BlenderColorCamera(LeafSystem):
    """
    BlenderColorCamera is a System block that connects to the pose bundle output
    port of a SceneGraph and uses BlenderServer to render a color camera image.
    """

    def __init__(self,
                 scene_graph,
                 zmq_url="default",
                 draw_period=0.033333,
                 camera_tfs=[Isometry3()],
                 material_overrides=[],
                 global_transform=Isometry3(),
                 out_prefix=None,
                 show_figure=False):
        """
        Args:
            scene_graph: A SceneGraph object.
            draw_period: The rate at which this class publishes rendered
                images.
            zmq_url: Optionally set a url to connect to the blender server.
                Use zmp_url="default" to the value obtained by running a
                single blender server in another terminal.
                TODO(gizatt): Use zmp_url=None or zmq_url="new" to start a
                new server (as a child of this process); a new web browser
                will be opened (the url will also be printed to the console).
                Use e.g. zmq_url="tcp://127.0.0.1:5556" to specify a
                specific address.
            camera tfs: List of Isometry3 camera tfs.
            material_overrides: A list of tuples of regex rules and
                material override arguments to be passed into a
                a register material call, not including names. (Those are
                assigned automatically by this class.)
            global transform: Isometry3 that gets premultiplied to every object.

        Note: This call will not return until it connects to the
              Blender server.
        """
        LeafSystem.__init__(self)

        if out_prefix is not None:
            self.out_prefix = out_prefix
        else:
            self.out_prefix = "/tmp/drake_blender_vis_"
        self.current_publish_num = 0
        self.set_name('blender_color_camera')
        self._DeclarePeriodicPublish(draw_period, 0.0)
        self.draw_period = draw_period

        # Pose bundle (from SceneGraph) input port.
        self._DeclareAbstractInputPort(
            "lcm_visualization",
            AbstractValue.Make(PoseBundle(0)))

        # Optional pose bundle of bounding boxes.
        self._DeclareAbstractInputPort(
            "bounding_box_bundle",
            AbstractValue.Make(BoundingBoxBundle(0)))

        if zmq_url == "default":
            zmq_url = "tcp://127.0.0.1:5556"
        elif zmq_url == "new":
            raise NotImplementedError("TODO")

        if zmq_url is not None:
            print("Connecting to blender server at zmq_url=" + zmq_url + "...")
        self.bsi = BlenderServerInterface(zmq_url=zmq_url)
        print("Connected to Blender server.")
        self._scene_graph = scene_graph

        # Compile regex for the material overrides
        self.material_overrides = [
            (re.compile(x[0]), x[1]) for x in material_overrides]
        self.global_transform = global_transform
        self.camera_tfs = camera_tfs

        def on_initialize(context, event):
            self.load()

        self._DeclareInitializationEvent(
            event=PublishEvent(
                trigger_type=TriggerType.kInitialization,
                callback=on_initialize))

        self.show_figure = show_figure
        if self.show_figure:
            plt.figure()

    def _parse_name(self, name):
        # Parse name, split on the first (required) occurrence of `::` to get
        # the source name, and let the rest be the frame name.
        # TODO(eric.cousineau): Remove name parsing once #9128 is resolved.
        delim = "::"
        assert delim in name
        pos = name.index(delim)
        source_name = name[:pos]
        frame_name = name[pos + len(delim):]
        return source_name, frame_name

    def _format_geom_name(self, source_name, frame_name, model_id, i):
        return "{}::{}::{}::{}".format(source_name, model_id, frame_name, i)

    def load(self):
        """
        Loads all visualization elements in the Blender server.

        @pre The `scene_graph` used to construct this object must be part of a
        fully constructed diagram (e.g. via `DiagramBuilder.Build()`).
        """
        self.bsi.send_remote_call("initialize_scene")

        # Keeps track of registered bounding boxes, to keep us from
        # having to register + delete brand new objects every cycle.
        # If more than this number is needed in a given cycle,
        # more are registered and this number is increased.
        # If less are needed, the unused ones are made invisible.
        # The attributes of all bounding boxes are updated every cycle.
        self.num_registered_bounding_boxes = 0
        self.num_visible_bounding_boxes = 0

        # Intercept load message via mock LCM.
        mock_lcm = DrakeMockLcm()
        DispatchLoadMessage(self._scene_graph, mock_lcm)
        load_robot_msg = lcmt_viewer_load_robot.decode(
            mock_lcm.get_last_published_message("DRAKE_VIEWER_LOAD_ROBOT"))
        # Load all the elements over on the Blender side.
        self.num_link_geometries_by_link_name = {}
        self.link_subgeometry_local_tfs_by_link_name = {}
        for i in range(load_robot_msg.num_links):
            link = load_robot_msg.link[i]
            [source_name, frame_name] = self._parse_name(link.name)
            self.num_link_geometries_by_link_name[link.name] = link.num_geom

            tfs = []
            for j in range(link.num_geom):
                geom = link.geom[j]

                # MultibodyPlant currently sets alpha=0 to make collision
                # geometry "invisible".  Ignore those geometries here.
                if geom.color[3] == 0:
                    continue

                geom_name = self._format_geom_name(source_name, frame_name, link.robot_num, j)

                tfs.append(RigidTransform(
                    RotationMatrix(Quaternion(geom.quaternion)),
                    geom.position).GetAsMatrix4())

                # It will have a material with this key.
                # We will set it up after deciding whether it's
                # a mesh with a texture or not...
                material_key = "material_" + geom_name
                material_key_assigned = False

                # Check overrides
                for override in self.material_overrides:
                    if override[0].match(geom_name):
                        print("Using override ", override[0].pattern, " on name ", geom_name, " with applied args ", override[1])
                        self.bsi.send_remote_call(
                            "register_material",
                            name=material_key,
                            **(override[1]))
                        material_key_assigned = True

                if geom.type == geom.BOX:
                    assert geom.num_float_data == 3
                    # Blender cubes are 2x2x2 by default
                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="cube",
                        scale=[x*0.5 for x in geom.float_data[:3]],
                        location=geom.position,
                        quaternion=geom.quaternion,
                        material=material_key)

                elif geom.type == geom.SPHERE:
                    assert geom.num_float_data == 1
                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="sphere",
                        scale=geom.float_data[0],
                        location=geom.position,
                        quaternion=geom.quaternion,
                        material=material_key)

                elif geom.type == geom.CYLINDER:
                    assert geom.num_float_data == 2
                    # Blender cylinders are r=1, h=2 by default
                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="cylinder",
                        scale=[geom.float_data[0], geom.float_data[0], 0.5*geom.float_data[1]],
                        location=geom.position,
                        quaternion=geom.quaternion,
                        material=material_key)

                elif geom.type == geom.MESH:
                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="obj",
                        path=geom.string_data[0:-3] + "obj",
                        scale=geom.float_data[:3],
                        location=geom.position,
                        quaternion=geom.quaternion,
                        material=material_key)

                    # Attempt to find a texture for the object by looking for an
                    # identically-named *.png next to the model.
                    # TODO(gizatt): In the long term, this kind of material information
                    # should be gleaned from the SceneGraph constituents themselves, so
                    # that we visualize what the simulation is *actually* reasoning about
                    # rather than what files happen to be present.
                    candidate_texture_path_png = geom.string_data[0:-3] + "png"
                    if not material_key_assigned and os.path.exists(candidate_texture_path_png):
                        material_key_assigned = self.bsi.send_remote_call(
                            "register_material",
                            name=material_key,
                            material_type="color_texture",
                            path=candidate_texture_path_png)

                else:
                    print("UNSUPPORTED GEOMETRY TYPE {} IGNORED".format(
                          geom.type))
                    continue

                if not material_key_assigned:
                    material_key_assigned = self.bsi.send_remote_call(
                        "register_material",
                        name=material_key,
                        material_type="color",
                        color=geom.color[:4])

                # Finally actually load the geometry now that the material
                # is registered.
                do_load_geom()

            self.link_subgeometry_local_tfs_by_link_name[link.name] = tfs

        for i, camera_tf in enumerate(self.camera_tfs):
            camera_tf_post = self.global_transform.multiply(camera_tf)
            self.bsi.send_remote_call(
                "register_camera",
                name="cam_%d" % i,
                location=camera_tf_post.translation().tolist(),
                quaternion=camera_tf_post.quaternion().wxyz().tolist(),
                angle=90)

            self.bsi.send_remote_call(
                "configure_rendering",
                camera_name='cam_%d' % i,
                resolution=[640*2, 480*2],
                file_format="JPEG")

        env_map_path = "/home/gizatt/tools/blender_server/data/env_maps/aerodynamics_workshop_4k.hdr"
        self.bsi.send_remote_call(
            "set_environment_map",
            path=env_map_path)

    def _DoPublish(self, context, event):
        # TODO(russt): Change this to declare a periodic event with a
        # callback instead of overriding _DoPublish, pending #9992.
        LeafSystem._DoPublish(self, context, event)

        pose_bundle = self.EvalAbstractInput(context, 0).get_value()
        bbox_bundle = self.EvalAbstractInput(context, 1)
        if bbox_bundle:
            bbox_bundle = bbox_bundle.get_value()
            for k in range(bbox_bundle.get_num_bboxes()):
                pose = self.global_transform.multiply(
                    bbox_bundle.get_bbox_pose(k))
                if (k + 1) > self.num_registered_bounding_boxes:
                    # Register a new one
                    color = bbox_bundle.get_bbox_color(k)
                    self.bsi.send_remote_call(
                            "register_material",
                            name="mat_bb_%d" % k,
                            material_type="color",
                            color=bbox_bundle.get_bbox_color(k))
                    self.bsi.send_remote_call(
                            "update_material_parameters",
                            type="Principled BSDF",
                            name="mat_bb_%d" % k,
                            **{"Specular": 0.0})
                    self.bsi.send_remote_call(
                            "update_material_parameters",
                            type=None,
                            name="mat_bb_%d" % k,
                            **{"blend_method": "ADD"})
                    self.bsi.send_remote_call(
                        "register_object",
                        name="obj_bb_%d" % k,
                        type="cube",
                        scale=[x/2 for x in bbox_bundle.get_bbox_scale(k)],
                        location=pose.translation().tolist(),
                        quaternion=pose.quaternion().wxyz().tolist(),
                        material="mat_bb_%d" % k)
                    self.bsi.send_remote_call(
                        "apply_modifier_to_object",
                        name="obj_bb_%d" % k,
                        type="WIREFRAME",
                        thickness=0.1)
                    self.num_registered_bounding_boxes = k + 1
                    self.num_visible_bounding_boxes = k + 1
                else:
                    # Update parameters of existing bbox + its material
                    self.bsi.send_remote_call(
                            "update_material_parameters",
                            type="Principled BSDF",
                            name="mat_bb_%d" % k,
                            **{"Base Color": bbox_bundle.get_bbox_color(k)})
                    self.bsi.send_remote_call(
                            "update_object_parameters",
                            name="obj_bb_%d" % k,
                            scale=[x/2 for x in bbox_bundle.get_bbox_scale(k)],
                            location=pose.translation().tolist(),
                            rotation_mode='QUATERNION',
                            rotation_quaternion=pose.quaternion().
                            wxyz().tolist(),
                            hide_render=False)
                    self.num_visible_bounding_boxes = k + 1
            for k in range(bbox_bundle.get_num_bboxes(),
                           self.num_visible_bounding_boxes):
                self.bsi.send_remote_call(
                        "update_object_parameters",
                        name="obj_bb_%d" % k,
                        hide_render=True)

        for frame_i in range(pose_bundle.get_num_poses()):
            link_name = pose_bundle.get_name(frame_i)
            [source_name, frame_name] = self._parse_name(link_name)
            model_id = pose_bundle.get_model_instance_id(frame_i)
            pose_matrix = pose_bundle.get_pose(frame_i)
            for j in range(self.num_link_geometries_by_link_name[link_name]):
                offset = Isometry3(
                    self.global_transform.matrix().dot(
                        pose_matrix.matrix().dot(
                            self.link_subgeometry_local_tfs_by_link_name[
                                link_name][j])))
                location = offset.translation()
                quaternion = offset.quaternion().wxyz()
                geom_name = self._format_geom_name(
                    source_name, frame_name, model_id, j)
                self.bsi.send_remote_call(
                    "update_object_parameters",
                    name="obj_" + geom_name,
                    location=location.tolist(),
                    rotation_mode='QUATERNION',
                    rotation_quaternion=quaternion.tolist())

        n_cams = len(self.camera_tfs)
        for i in range(len(self.camera_tfs)):
            out_filepath = self.out_prefix + "%02d_%08d.jpg" % (
                i, self.current_publish_num)
            im = self.bsi.render_image(
                "cam_%d" % i, filepath=out_filepath)
            if self.show_figure:
                plt.subplot(1, n_cams, i+1)
                plt.imshow(np.transpose(im, [1, 0, 2]))

        if self.current_publish_num == 0:
            scene_filepath = self.out_prefix + "_scene.blend"
            self.bsi.send_remote_call(
                "save_current_scene", path=scene_filepath)
            self.published_scene = True
        self.current_publish_num += 1

        if self.show_figure:
            plt.pause(0.01)
        print("Rendered a frame at %f seconds sim-time." % (
            context.get_time()))


def _xyz_rpy(p, rpy):
    return RigidTransform(rpy=RollPitchYaw(rpy), p=p)


def CreateYcbObjectClutter():
    ycb_object_pairs = []

    X_WCracker = _xyz_rpy([0.35, 0.14, 0.09], [0, -1.57, 4])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/003_cracker_box.sdf", X_WCracker))

    # The sugar box pose.
    X_WSugar = _xyz_rpy([0.28, -0.17, 0.03], [0, 1.57, 3.14])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/004_sugar_box.sdf", X_WSugar))

    # The tomato soup can pose.
    X_WSoup = _xyz_rpy([0.40, -0.07, 0.03], [-1.57, 0, 3.14])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/005_tomato_soup_can.sdf", X_WSoup))

    # The mustard bottle pose.
    X_WMustard = _xyz_rpy([0.44, -0.16, 0.09], [-1.57, 0, 3.3])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf",
         X_WMustard))

    # The gelatin box pose.
    #X_WGelatin = _xyz_rpy([0.35, -0.32, 0.1], [-1.57, 0, 2.5])
    #ycb_object_pairs.append(
    #    ("drake/manipulation/models/ycb/sdf/009_gelatin_box.sdf", X_WGelatin))

    # The potted meat can pose.
    X_WMeat = _xyz_rpy([0.35, -0.32, 0.03], [-1.57, 0, 2.5])
    ycb_object_pairs.append(
        ("drake/manipulation/models/ycb/sdf/010_potted_meat_can.sdf", X_WMeat))

    return ycb_object_pairs


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Demo of the Blender visualizer functionality "
                    "on the Drake ManipulationStation example.")
    parser.add_argument(
        "--duration", type=float, default=np.inf,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--test", action='store_true',
        help="Disable open_ing the gui window for testing.")
    parser.add_argument(
        "--log", type=str, default=None,
        help="Pickle file of logged run to play back. If not supplied, "
             " a teleop panel will appear instead.")
    parser.add_argument(
        "--log_bbox", type=str, default=None,
        help="Optional yaml file of logged bbox detections to play back.")
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    builder = DiagramBuilder()

    # Create the ManipulationStation.
    station = builder.AddSystem(ManipulationStation())
    station.SetupDefaultStation()
    ycb_objects = CreateYcbObjectClutter()
    for model_file, X_WObject in ycb_objects:
        station.AddManipulandFromFile(model_file, X_WObject)
    station.Finalize()

    if args.meshcat:
            meshcat = builder.AddSystem(MeshcatVisualizer(
                station.get_scene_graph(), zmq_url=args.meshcat))
            builder.Connect(station.GetOutputPort("pose_bundle"),
                            meshcat.get_input_port(0))


    #cam_quat_base = RollPitchYaw(
    #    81.8*np.pi/180.,
    #    -5.*np.pi/180,
    #    (129.+90)*np.pi/180.).ToQuaternion().wxyz()
    #cam_trans_base = [-0.196, 0.816, 0.435]
    #cam_tf_base = Isometry3(translation=cam_trans_base,
    #                        quaternion=cam_quat_base)
    #cam_tfs = [cam_tf_base]

    cam_tfs = []
    for cam_name in [u"0", u"1", u"2"]:
        cam_tf_base = station.GetStaticCameraPosesInWorld()[cam_name].GetAsIsometry3()
        # Rotate cam to get it into blender +y up, +x right, -z forward
        cam_tf_base.set_rotation(cam_tf_base.matrix()[:3, :3].dot(
            RollPitchYaw([-np.pi/2, 0., np.pi/2]).ToRotationMatrix().matrix()))
        cam_tfs.append(cam_tf_base)

    offset_quat_base = RollPitchYaw(0., 0., np.pi/4).ToQuaternion().wxyz()
    os.system("rm -r /tmp/manipulation_station_ycb && mkdir -p /tmp/manipulation_station_ycb")
    blender_cam = builder.AddSystem(BlenderColorCamera(
        station.get_scene_graph(),
        show_figure=False,
        draw_period=0.03333333,
        camera_tfs=cam_tfs,
        material_overrides=[
            (".*amazon_table.*",
                {"material_type": "CC0_texture",
                 "path": "data/test_pbr_mats/Metal09/Metal09"}),
            (".*cupboard_body.*",
                {"material_type": "CC0_texture",
                 "path": "data/test_pbr_mats/Wood15/Wood15"}),
            (".*top_and_bottom.*",
                {"material_type": "CC0_texture",
                 "path": "data/test_pbr_mats/Wood15/Wood15"}),
            (".*cupboard.*door.*",
                {"material_type": "CC0_texture",
                 "path": "data/test_pbr_mats/Wood08/Wood08"})
        ],
        global_transform=Isometry3(translation=[0, 0, 0],
                                   quaternion=Quaternion(offset_quat_base)),
        out_prefix="/tmp/manipulation_station_ycb/"
    ))
    builder.Connect(station.GetOutputPort("pose_bundle"),
                    blender_cam.get_input_port(0))

    if args.log_bbox:
        bbox_source = builder.AddSystem(BoundingBoxBundleYamlSource(
            args.log_bbox, publish_period=blender_cam.draw_period))
        builder.Connect(bbox_source.get_output_port(0),
                        blender_cam.get_input_port(1))

    if args.log:
        def buildTrajectorySource(ts, knots):
            #good_steps = np.diff(np.hstack([ts_raw, np.array(ts_raw[-1])])) >= 0.001
            #ts = ts_raw[good_steps]
            #knots = knots_raw[:, good_steps]
            ppt = PiecewisePolynomial.FirstOrderHold(
                ts, knots)
            return TrajectorySource(
                trajectory=ppt, output_derivative_order=0,
                zero_derivatives_beyond_limits=True)

        with open(args.log, "rb") as f:
            input_dict = pickle.load(f)
        iiwa_position_trajsource = builder.AddSystem(
            buildTrajectorySource(input_dict["iiwa_position_t"],
                                  input_dict["iiwa_position_data"]))
        wsg_position_trajsource = builder.AddSystem(
            buildTrajectorySource(input_dict["wsg_position_t"],
                                  input_dict["wsg_position_data"]))
        builder.Connect(iiwa_position_trajsource.get_output_port(0),
                        station.GetInputPort("iiwa_position"))
        builder.Connect(wsg_position_trajsource.get_output_port(0),
                        station.GetInputPort("wsg_position"))
        duration = input_dict["iiwa_position_t"][-1]
    else:
        teleop = builder.AddSystem(JointSliders(station.get_controller_plant(),
                                                length=800))
        filter = builder.AddSystem(FirstOrderLowPassFilter(time_constant=2.0,
                                                           size=7))
        builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))
        builder.Connect(filter.get_output_port(0),
                        station.GetInputPort("iiwa_position"))

        wsg_buttons = builder.AddSystem(SchunkWsgButtons(teleop.window))
        builder.Connect(wsg_buttons.GetOutputPort("position"), station.GetInputPort(
            "wsg_position"))
        builder.Connect(wsg_buttons.GetOutputPort("force_limit"),
                        station.GetInputPort("wsg_force_limit"))
        duration = args.duration

    diagram = builder.Build()
    simulator = Simulator(diagram)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station_context.FixInputPort(station.GetInputPort(
        "iiwa_feedforward_torque").get_index(), np.zeros(7))
    if args.log:
        station_context.FixInputPort(station.GetInputPort(
            "wsg_force_limit").get_index(), np.zeros(1)+40)
        station.SetIiwaPosition(station_context, input_dict["q0"])
    else:
        # Eval the output port once to read the initial positions of the IIWA.
        q0 = station.GetOutputPort("iiwa_position_measured").Eval(
            station_context)
        q0[:] = [0., 0.5, 0., -1.75, 0., 1., 0.]
        teleop.set_position(q0)
        filter.set_initial_output_value(diagram.GetMutableSubsystemContext(
            filter, simulator.get_mutable_context()), q0)

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    simulator.set_target_realtime_rate(1.0)
    print("Starting simulation for %f seconds." % duration)
    simulator.StepTo(duration)
