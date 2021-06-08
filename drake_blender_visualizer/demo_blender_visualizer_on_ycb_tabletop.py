"""

Visualizes Drake scene graph state using the BlenderColorCamera
and BlenderLabelCamera wrappers of the blender server. These are
relatively standard Drake visualizer blocks that connect to a
SceneGraph and visualizes its state.

To run, make sure you've set up *two* blender servers -- one for
the color images, and one for the label images:

```
./run_blender_server.sh 5556 &
./run_blender_server.sh 5557
```

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

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    BodyIndex,
    Box,
    CoulombFriction,
    DiagramBuilder,
    FindResourceOrThrow,
    getDrakePath,
    InverseKinematics,
    MeshcatVisualizer,
    MultibodyPlant,
    Parser,
    Quaternion,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
    Solve,
    SpatialInertia,
    UnitInertia
)
from blender_server.drake_blender_visualizer.blender_visualizer import (
    BlenderColorCamera,
    BlenderLabelCamera
)


def RegisterVisualAndCollisionGeometry(
        mbp, body, pose, shape, name, color, friction):
    '''
        Adds the given Body to the MBP as both visual and collision geometry.
    '''
    mbp.RegisterVisualGeometry(body, pose, shape, name + "_vis", color)
    mbp.RegisterCollisionGeometry(body, pose, shape, name + "_col",
                                  friction)


if __name__ == "__main__":
    # Random seed setup for reproducibility.
    np.random.seed(int(codecs.encode(os.urandom(4), 'hex'), 32) & (2**32 - 1))
    random.seed(os.urandom(4))

    for scene_k in range(25):

        # Set up a new Drake scene from scratch.
        builder = DiagramBuilder()
        mbp, scene_graph = AddMultibodyPlantSceneGraph(
            builder, MultibodyPlant(time_step=0.005))

        # Add "tabletop" (ground) as a fixed Box welded to the world.
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

        # Figure out what YCB objects we have available to add.
        ycb_object_dir = os.path.join(
            getDrakePath(), "manipulation/models/ycb/sdf/")
        ycb_object_sdfs = os.listdir(ycb_object_dir)
        ycb_object_sdfs = [os.path.join(ycb_object_dir, path)
                           for path in ycb_object_sdfs]

        # Add random objects to the scene.
        parser = Parser(mbp, scene_graph)
        n_objects = np.random.randint(1, 12)
        obj_ids = []
        for k in range(n_objects):
            obj_i = np.random.randint(len(ycb_object_sdfs))
            parser.AddModelFromFile(
                file_name=ycb_object_sdfs[obj_i],
                model_name="obj_ycb_%03d" % k)
            obj_ids.append(k+2)

        mbp.Finalize()

        # Optional: set up a meshcat visualizer for the scene.
        #meshcat = builder.AddSystem(
        #    MeshcatVisualizer(scene_graph, draw_period=0.0333))
        #builder.Connect(scene_graph.get_pose_bundle_output_port(),
        #                meshcat.get_input_port(0))

        # Figure out where we're putting the camera for the scene by
        # pointing it inwards, and then applying a random yaw around
        # the origin.
        cam_quat_base = RollPitchYaw(
            68.*np.pi/180.,
            0.*np.pi/180,
            38.6*np.pi/180.).ToQuaternion()
        cam_trans_base = np.array([0.47, -0.54, 0.31])
        cam_tf_base = RigidTransform(quaternion=cam_quat_base,
                                p=cam_trans_base)
        # Rotate camera around origin
        cam_additional_rotation = RigidTransform(
            quaternion=RollPitchYaw(0., 0., np.random.uniform(0., np.pi*2.)).ToQuaternion(),
            p=[0, 0, 0]
        )
        cam_tf_base = cam_additional_rotation.multiply(cam_tf_base)
        cam_tfs = [cam_tf_base]

        # Set up the blender color camera using that camera, and specifying
        # the environment map and a material to apply to the "ground" object.
        # If you wanted to further rotate the scene (to e.g. rotate the background
        # of the scene relative to the table coordinate system), you could apply an
        # extra yaw here.
        offset_quat_base = RollPitchYaw(0., 0., 0.).ToQuaternion().wxyz()
        os.system("mkdir -p /tmp/ycb_scene_%03d" % scene_k)
        blender_color_cam = builder.AddSystem(BlenderColorCamera(
            scene_graph,
            draw_period=0.03333/2.,
            camera_tfs=cam_tfs,
            zmq_url="tcp://127.0.0.1:5556",
            env_map_path="data/env_maps/aerodynamics_workshop_4k.hdr",
            material_overrides=[
                (".*ground.*",
                    {"material_type": "CC0_texture",
                     "path": "data/test_pbr_mats/Wood15/Wood15"}),
            ],
            global_transform=RigidTransform(
                p=[0, 0, 0], quaternion=Quaternion(offset_quat_base)
            ),
            out_prefix="/tmp/ycb_scene_%03d/" % scene_k
        ))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        blender_color_cam.get_input_port(0))

        # Make a similar label camera to render label images for the scene.
        blender_label_cam = builder.AddSystem(BlenderLabelCamera(
            scene_graph,
            draw_period=0.03333/2.,
            camera_tfs=cam_tfs,
            zmq_url="tcp://127.0.0.1:5557",
            global_transform=RigidTransform(
                p=[0, 0, 0], quaternion=Quaternion(offset_quat_base)
            ),
            out_prefix="/tmp/ycb_scene_%03d/" % scene_k
        ))
        builder.Connect(scene_graph.get_pose_bundle_output_port(),
                        blender_label_cam.get_input_port(0))
        diagram = builder.Build()

        # Finish up creating the scene and set initial positions for the objects.
        diagram_context = diagram.CreateDefaultContext()
        mbp_context = diagram.GetMutableSubsystemContext(
            mbp, diagram_context)
        for obj_id in obj_ids:
            mbp.SetFreeBodyPose(
                mbp_context, body=mbp.get_body(BodyIndex(obj_id)),
                X_WB=RigidTransform(
                    p=[np.random.randn()*0.1,
                       np.random.randn()*0.1,
                       np.random.uniform(0.1, 0.3)],
                    quaternion=RollPitchYaw(
                        np.random.uniform(0, 2*np.pi, size=3)).ToQuaternion()
                )
            )

        # Use IK to find a nonpenetrating configuration of the scene from
        # which to start simulation.
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

        # Run a sim starting from whatever configuration IK figured out.
        mbp.SetPositions(mbp_context, q0_proj)

        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.set_target_realtime_rate(1.0)
        try:
            simulator.AdvanceTo(2.0)
        except Exception as e:
            print("Exception in sim: ", e)
            scene_k -= 1
