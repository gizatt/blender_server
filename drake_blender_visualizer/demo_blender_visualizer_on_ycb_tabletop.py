"""
Visualizes SceneGraph state using Blender Server.
"""
from __future__ import print_function
import argparse
import codecs
import math
import os
import random
import re
import time
import warnings

import matplotlib.pyplot as plt
import numpy as np

from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import Box, DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.systems.framework import (
    AbstractValue, LeafSystem, PublishEvent, TriggerType
)
from pydrake.systems.rendering import PoseBundle
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
    MultibodyPlant
)
from pydrake.multibody.tree import (
    SpatialInertia,
    UniformGravityFieldElement,
    UnitInertia,
    BodyIndex
)

from pydrake import getDrakePath
from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import (
    ManipulationStation)
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.util.eigen_geometry import Isometry3

from blender_server.drake_blender_visualizer.blender_visualizer import (
    BlenderColorCamera,
    BlenderLabelCamera
)


def RegisterVisualAndCollisionGeometry(
        mbp, body, pose, shape, name, color, friction):
    mbp.RegisterVisualGeometry(body, pose, shape, name + "_vis", color)
    mbp.RegisterCollisionGeometry(body, pose, shape, name + "_col",
                                  friction)


if __name__ == "__main__":
    np.random.seed(int(codecs.encode(os.urandom(4), 'hex'), 32) & (2**32 - 1))
    random.seed(os.urandom(4))
    for scene_k in range(25):
        builder = DiagramBuilder()
        mbp, scene_graph = AddMultibodyPlantSceneGraph(
            builder, MultibodyPlant(time_step=0.005))

        # Add "tabletop" (ground)
        world_body = mbp.world_body()
        ground_shape = Box(1., 1., 1.)
        ground_body = mbp.AddRigidBody("ground", SpatialInertia(
            mass=10.0, p_PScm_E=np.array([0., 0., 0.]),
            G_SP_E=UnitInertia(1.0, 1.0, 1.0)))
        mbp.WeldFrames(world_body.body_frame(), ground_body.body_frame(),
                       RigidTransform())
        RegisterVisualAndCollisionGeometry(
            mbp, ground_body,
            RigidTransform(p=[0, 0, -0.5]),
            ground_shape, "ground", np.array([0.5, 0.5, 0.5, 1.]),
            CoulombFriction(0.9, 0.8))

        ycb_object_dir = os.path.join(
            getDrakePath(), "manipulation/models/ycb/sdf/")
        ycb_object_sdfs = os.listdir(ycb_object_dir)
        ycb_object_sdfs = [os.path.join(ycb_object_dir, path)
                           for path in ycb_object_sdfs]

        # Add random objects
        parser = Parser(mbp, scene_graph)
        n_objects = np.random.randint(1, 12)
        obj_ids = []
        for k in range(n_objects):
            obj_i = np.random.randint(len(ycb_object_sdfs))
            parser.AddModelFromFile(
                file_name=ycb_object_sdfs[obj_i],
                model_name="obj_ycb_%03d" % k)
            obj_ids.append(k+2)

        # mbp.AddForceElement(UniformGravityFieldElement())
        mbp.Finalize()

        #meshcat = builder.AddSystem(
        #    MeshcatVisualizer(scene_graph, draw_period=0.0333))
        #builder.Connect(scene_graph.get_pose_bundle_output_port(),
        #                meshcat.get_input_port(0))

        cam_quat_base = RollPitchYaw(
            68.*np.pi/180.,
            0.*np.pi/180,
            38.6*np.pi/180.).ToQuaternion()
        cam_trans_base = np.array([0.47, -0.54, 0.31])
        cam_tf_base = Isometry3(quaternion=cam_quat_base,
                                translation=cam_trans_base)
        # Rotate camera around origin
        cam_additional_rotation = Isometry3(quaternion=RollPitchYaw(0., 0., np.random.uniform(0., np.pi*2.)).ToQuaternion(),
                                            translation=[0, 0, 0])
        cam_tf_base = cam_additional_rotation.multiply(cam_tf_base)
        cam_tfs = [cam_tf_base]

        offset_quat_base = RollPitchYaw(0., 0., 0.).ToQuaternion().wxyz()
        os.system("mkdir -p /tmp/ycb_scene_%03d" % scene_k)
        blender_color_cam = builder.AddSystem(BlenderColorCamera(
            scene_graph,
            draw_period=0.03333/2.,
            camera_tfs=cam_tfs,
            zmq_url="tcp://127.0.0.1:5556",
            env_map_path="/home/gizatt/tools/blender_server/data/env_maps/aerodynamics_workshop_4k.hdr",
            material_overrides=[
                (".*ground.*",
                    {"material_type": "CC0_texture",
                     "path": "data/test_pbr_mats/Wood15/Wood15"}),
            ],
            global_transform=Isometry3(translation=[0, 0, 0],
                                       quaternion=Quaternion(offset_quat_base)),
            out_prefix="/tmp/ycb_scene_%03d/" % scene_k
        ))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        blender_color_cam.get_input_port(0))

        blender_label_cam = builder.AddSystem(BlenderLabelCamera(
            scene_graph,
            draw_period=0.03333/2.,
            camera_tfs=cam_tfs,
            zmq_url="tcp://127.0.0.1:5557",
            global_transform=Isometry3(translation=[0, 0, 0],
                                       quaternion=Quaternion(offset_quat_base)),
            out_prefix="/tmp/ycb_scene_%03d/" % scene_k
        ))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        blender_label_cam.get_input_port(0))
        diagram = builder.Build()

        diagram_context = diagram.CreateDefaultContext()
        mbp_context = diagram.GetMutableSubsystemContext(
            mbp, diagram_context)
        for obj_id in obj_ids:
            mbp.SetFreeBodyPose(
                mbp_context, body=mbp.get_body(BodyIndex(obj_id)),
                X_WB=Isometry3(translation=[np.random.randn()*0.1,
                                            np.random.randn()*0.1,
                                            np.random.uniform(0.1, 0.3)],
                               quaternion=RollPitchYaw(
                                    np.random.uniform(0, 2*np.pi, size=3)).ToQuaternion()))

        # Project to nonpenetration to seed sim
        q0 = mbp.GetPositions(mbp_context).copy()
        ik = InverseKinematics(mbp, mbp_context)
        q_dec = ik.q()
        prog = ik.prog()

        constraint = ik.AddMinimumDistanceConstraint(0.01)
        prog.AddQuadraticErrorCost(np.eye(q0.shape[0])*1.0, q0, q_dec)
        mbp.SetPositions(mbp_context, q0)

        prog.SetInitialGuess(q_dec, q0)
        print("Solving")
        print("Initial guess: ", q0)
        res = Solve(prog)
        #print(prog.GetSolverId().name())
        q0_proj = res.GetSolution(q_dec)
        print("Final: ", q0_proj)

        mbp.SetPositions(mbp_context, q0_proj)

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.set_target_realtime_rate(1.0)
        try:
            simulator.AdvanceTo(2.0)
        except Exception as e:
            print("Exception in sim: ", e)
            scene_k -= 1
