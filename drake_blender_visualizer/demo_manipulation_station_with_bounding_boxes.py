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
    CreateManipulationClassYcbObjectList)
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import (
    FirstOrderLowPassFilter, TrajectorySource)
from pydrake.trajectories import PiecewisePolynomial
from pydrake.common.eigen_geometry import Isometry3


from blender_server.drake_blender_visualizer.blender_visualizer import (
    BlenderCamera,
    BoundingBoxBundle
)


class BoundingBoxBundleYamlSource(LeafSystem):
    def __init__(self, log_file, publish_period=0.033333):
        LeafSystem.__init__(self)

        self.iter = 0
        self.set_name('ycb yaml log bbox publisher')
        self.publish_period = publish_period
        self.DeclarePeriodicPublish(self.publish_period, 0.0)

        self.bbox_bundle_output_port = \
            self.DeclareAbstractOutputPort(
                self.DoAllocBboxBundle,
                self.DoCalcAbstractOutput)

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

    def DoAllocBboxBundle(self):
        return AbstractValue.Make(BoundingBoxBundle)

    def DoCalcAbstractOutput(self, context, y_data):
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
    station.SetupManipulationClassStation()
    ycb_objects = CreateManipulationClassYcbObjectList()
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
    blender_cam = builder.AddSystem(BlenderCamera(
        station.get_scene_graph(),
        show_figure=False,
        draw_period=0.03333,
        camera_tfs=cam_tfs,
        env_map_path="/home/gizatt/tools/blender_server/data/env_maps/aerodynamics_workshop_4k.hdr",
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
