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
from pydrake.geometry import DispatchLoadMessage, Role, SceneGraph
from pydrake.lcm import DrakeMockLcm, Subscriber
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.common.value import AbstractValue
# from pydrake.systems.framework import (
#     AbstractValue, LeafSystem, PublishEvent, TriggerType
# )
from pydrake.geometry import (
    Box, Convex, Cylinder, Mesh, Sphere,
    FrameId, QueryObject, Role, SceneGraph
)
import meshcat.geometry as g  # noqa
import meshcat.transformations as tf  # noqa
from pydrake.systems.framework import (
    LeafSystem, PublishEvent, TriggerType
)
from pydrake.systems.rendering import PoseBundle
from pydrake.multibody.plant import ContactResults

from pydrake.common import FindResourceOrThrow
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import (
    FirstOrderLowPassFilter, TrajectorySource)
from pydrake.trajectories import PiecewisePolynomial


from blender_server.blender_server_interface.blender_server_interface import (
    BlenderServerInterface)

import warnings
from pydrake.common.deprecation import DrakeDeprecationWarning
# import sys, os

# # Disable
# def blockPrint():
#     sys.stdout = open(os.devnull, 'w')

# # Restore
# def enablePrint():
#     sys.stdout = sys.__stdout__


class BoundingBoxBundle(object):
    def __init__(self, num_bboxes=0):
        self.reset(num_bboxes)

    def MakeCopy(self):
        new = BoundingBoxBundle(self.num_bboxes)
        new.scales = deepcopy(self.scales)
        new.colors = deepcopy(self.colors)
        new.poses = [RigidTransform(tf) for tf in self.poses]
        return new

    def reset(self, num_bboxes):
        self.num_bboxes = num_bboxes
        self.scales = [[1., 1., 1] for n in range(num_bboxes)]
        self.poses = [RigidTransform() for n in range(num_bboxes)]
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
        self.DeclarePeriodicPublish(0.03333, 0.0)

        self.bbox_bundle_output_port = \
            self.DeclareAbstractOutputPort(
                self.DoAllocBboxBundle,
                self.DoCalcAbstractOutput)

    def DoAllocBboxBundle(self):
        return AbstractValue.Make(BoundingBoxBundle)

    def DoCalcAbstractOutput(self, context, y_data):
        bbox_bundle = BoundingBoxBundle(2)
        bbox_bundle.set_bbox_attributes(
            0, scale=[0.1, 0.15, 0.2],
            pose=RigidTransform(p=[0.5, 0., 0.1]),
            color=[1., 0., 0., 1.])
        bbox_bundle.set_bbox_attributes(
            1, scale=[0.05, 0.05, 0.05],
            pose=RigidTransform(p=[0.4, 0.1, 0.1]),
            color=[0., 1., 1., 1.])
        y_data.set_value(bbox_bundle)


class BlenderLabelCamera(LeafSystem):
    """
    BlenderLabelCamera is a System block that connects to the pose bundle
    output port of a SceneGraph and uses BlenderServer to render label images.
    """

    def __init__(self,
                 scene_graph,
                 zmq_url="default",
                 draw_period=0.033333,
                 camera_tfs=[RigidTransform()],
                 global_transform=RigidTransform(),
                 out_prefix=None,
                 show_figure=False,
                 save_scene=False):
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
            camera tfs: List of RigidTransform camera tfs.
            global transform: RigidTransform that gets premultiplied to every object.

        Note: This call will not return until it connects to the
              Blender server.

        TODO(gizatt) Consolidate functionality with BlenderColorCamera
        into a single BlenderCamera. Needs careful architecting to still support
        rendering color + label images with different Blender servers in parallel.
        """
        LeafSystem.__init__(self)

        if out_prefix is not None:
            self.out_prefix = out_prefix
        else:
            self.out_prefix = "/tmp/drake_blender_vis_labels_"
        self.current_publish_num = 0
        self.set_name('blender_label_camera')
        self.DeclarePeriodicPublish(draw_period, 0.)
        self.draw_period = draw_period
        self.save_scene = save_scene

        # Pose bundle (from SceneGraph) input port.
        self.DeclareAbstractInputPort(
            "lcm_visualization",
            AbstractValue.Make(PoseBundle(0)))

        if zmq_url == "default":
            zmq_url = "tcp://127.0.0.1:5556"
        elif zmq_url == "new":
            raise NotImplementedError("TODO")

        if zmq_url is not None:
            print("Connecting to blender server at zmq_url=" + zmq_url + "...")
        self.bsi = BlenderServerInterface(zmq_url=zmq_url)
        print("Connected to Blender server.")
        self._scene_graph = scene_graph

        self.global_transform = global_transform
        self.camera_tfs = camera_tfs

        def on_initialize(context, event):
            self.load()

        self.DeclareInitializationEvent(
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

        # Intercept load message via mock LCM.
        mock_lcm = DrakeMockLcm()
        mock_lcm_subscriber = Subscriber(
            lcm=mock_lcm,
            channel="DRAKE_VIEWER_LOAD_ROBOT",
            lcm_type=lcmt_viewer_load_robot)
        DispatchLoadMessage(self._scene_graph, mock_lcm)
        mock_lcm.HandleSubscriptions(0)
        assert mock_lcm_subscriber.count > 0
        load_robot_msg = mock_lcm_subscriber.message

        # Load all the elements over on the Blender side.
        self.num_link_geometries_by_link_name = {}
        self.link_subgeometry_local_tfs_by_link_name = {}
        self.geom_name_to_color_map = {}
        num_allocated_labels = 0
        max_num_objs = sum([load_robot_msg.link[i].num_geom
                            for i in range(load_robot_msg.num_links)])
        cmap = plt.cm.get_cmap("hsv", max_num_objs+1)
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
                material_key = "material_" + geom_name
                label_num = num_allocated_labels
                self.geom_name_to_color_map[geom_name] = cmap(label_num)
                num_allocated_labels += 1

                material_key_assigned = self.bsi.send_remote_call(
                    "register_material",
                    name=material_key,
                    material_type="emission",
                    color=cmap(label_num))

                if geom.type == geom.BOX:
                    assert geom.num_float_data == 3
                    # Blender cubes are 2x2x2 by default
                    self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="cube",
                        scale=[x*0.5 for x in geom.float_data[:3]],
                        location=geom.position,
                        quaternion=geom.quaternion,
                        material=material_key)

                elif geom.type == geom.SPHERE:
                    assert geom.num_float_data == 1
                    self.bsi.send_remote_call(
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
                    self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="cylinder",
                        scale=[geom.float_data[0], geom.float_data[0], 0.5*geom.float_data[1]],
                        location=geom.position,
                        quaternion=geom.quaternion,
                        material=material_key)

                elif geom.type == geom.MESH:
                    self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="obj",
                        path=geom.string_data[0:-3] + "obj",
                        scale=geom.float_data[:3],
                        location=geom.position,
                        quaternion=geom.quaternion,
                        material=material_key)

                else:
                    print("UNSUPPORTED GEOMETRY TYPE {} IGNORED".format(
                          geom.type))
                    continue

            self.link_subgeometry_local_tfs_by_link_name[link.name] = tfs

        for i, camera_tf in enumerate(self.camera_tfs):
            camera_tf_post = self.global_transform.multiply(camera_tf)
            self.bsi.send_remote_call(
                "register_camera",
                name="cam_%d" % i,
                location=camera_tf_post.translation().tolist(),
                quaternion=camera_tf_post.rotation().ToQuaternion().wxyz().tolist(),
                angle=np.pi/2.)

            self.bsi.send_remote_call(
                "configure_rendering",
                camera_name='cam_%d' % i,
                resolution=[640, 480],
                file_format="BMP",
                configure_for_masks=True,
                taa_render_samples=10,
                cycles=False)

        self.bsi.send_remote_call(
            "set_environment_map",
            path=None)

    def _DoPublish(self, context, event):
        # TODO(russt): Change this to declare a periodic event with a
        # callback instead of overriding _DoPublish, pending #9992.
        LeafSystem._DoPublish(self, context, event)

        if context.get_time() <= 1E-3:
            return

        pose_bundle = self.EvalAbstractInput(context, 0).get_value()

        for frame_i in range(pose_bundle.get_num_poses()):
            link_name = pose_bundle.get_name(frame_i)
            [source_name, frame_name] = self._parse_name(link_name)
            model_id = pose_bundle.get_model_instance_id(frame_i)
            # pose_matrix = pose_bundle.get_pose(frame_i)
            pose_matrix = pose_bundle.get_transorm(frame_i)
            for j in range(self.num_link_geometries_by_link_name[link_name]):
                offset = RigidTransform(
                    self.global_transform.GetAsMatrix4().dot(
                        # pose_matrix.matrix().dot(
                        pose_matrix.GetAsMatrix4().dot(
                            self.link_subgeometry_local_tfs_by_link_name[
                                link_name][j])))
                location = offset.translation()
                quaternion = offset.rotation().ToQuaternion()
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
            out_filepath = self.out_prefix + "%02d_%08d_label.bmp" % (
                i, self.current_publish_num)
            im = self.bsi.render_image(
                "cam_%d" % i, filepath=out_filepath)
            if self.show_figure:
                plt.subplot(1, n_cams, i+1)
                plt.imshow(np.transpose(im, [1, 0, 2]))

        if self.save_scene:
            scene_filepath = self.out_prefix + "_scene.blend"
            self.bsi.send_remote_call(
                "save_current_scene", path=scene_filepath)
        self.current_publish_num += 1

        if self.show_figure:
            plt.pause(0.01)
        print("Rendered a frame at %f seconds sim-time." % (
            context.get_time()))


class BlenderColorCamera(LeafSystem):
    """
    BlenderColorCamera is a System block that connects to the pose bundle
    output port of a SceneGraph and uses BlenderServer to render a color camera
    image.
    """

    def __init__(self,
                 scene_graph,
                 zmq_url="default",
                 draw_period=0.033333,
                 camera_tfs=[RigidTransform()],
                 material_overrides=[],
                 global_transform=RigidTransform(),
                 env_map_path=None,
                 out_prefix=None,
                 show_figure=False,
                 role=Role.kIllustration,
                 save_scene=False):
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
            camera tfs: List of RigidTransform camera tfs.
            material_overrides: A list of tuples of regex rules and
                material override arguments to be passed into a
                a register material call, not including names. (Those are
                assigned automatically by this class.)
            global transform: RigidTransform that gets premultiplied to every object.

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
        self.DeclarePeriodicPublish(draw_period, 0.)
        self.draw_period = draw_period
        self.save_scene = save_scene

        # Pose bundle (from SceneGraph) input port.
        self.DeclareAbstractInputPort(
            "lcm_visualization",
            AbstractValue.Make(PoseBundle(0)))

        # Optional pose bundle of bounding boxes.
        self.DeclareAbstractInputPort(
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
        
        self._role = role
        self.env_map_path = env_map_path

        # Compile regex for the material overrides
        self.material_overrides = [
            (re.compile(x[0]), x[1]) for x in material_overrides]
        self.global_transform = global_transform
        self.camera_tfs = camera_tfs

        def on_initialize(context, event):
            self.load()

        self.DeclareInitializationEvent(
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

    # def _format_geom_name(self, source_name, frame_name, i):
    #     return "{}::{}::{}::{}".format(source_name, model_id, frame_name, i)

    def _format_geom_name(self, source_name, frame_name, i):
        return "{}::{}::{}".format(source_name, frame_name, i)

    def load(self):
        """
        Loads all visualization elements in the Blender server.

        @pre The `scene_graph` used to construct this object must be part of a
        fully constructed diagram (e.g. via `DiagramBuilder.Build()`).
        """
        print('before initialize_scene')
        self.bsi.send_remote_call("initialize_scene")
        print('after intialize_scene')

        # Keeps track of registered bounding boxes, to keep us from
        # having to register + delete brand new objects every cycle.
        # If more than this number is needed in  a given cycle,
        # more are registered and this number is increased.
        # If less are needed, the unused ones are made invisible.
        # The attributes of all bounding boxes are updated every cycle.
        self.num_registered_bounding_boxes = 0
        self.num_visible_bounding_boxes = 0

        # Load all the elements over on the Blender side.
        self.num_link_geometries_by_link_name = {}
        self.link_subgeometry_local_tfs_by_link_name = {}
        
        print('get model_inspector')
        inspector = self._scene_graph.model_inspector()

        for frame_id in inspector.all_frame_ids():
            # check the link_name
            link_name = (inspector.GetOwningSourceName(frame_id) + '::' + inspector.GetName(frame_id))
            [source_name, frame_name] = self._parse_name(link_name)
            self.num_link_geometries_by_link_name[link_name] = inspector.NumGeometriesForFrameWithRole(frame_id, self._role)

            tfs = []

            if self.num_link_geometries_by_link_name[link_name] == 0:
                continue

            for j, g_id in enumerate(inspector.GetGeometries(frame_id, self._role)):
                color = (0.9, 0.9, 0.9, 1.0)
                alpha = 1.0

                props = inspector.GetIllustrationProperties(g_id)
                if props and props.HasProperty("phong", "diffuse"):
                    rgba = props.GetProperty("phong", "diffuse")
                    color = (rgba.r(), rgba.g(), rgba.b(), rgba.a())
                    alpha = rgba.a()

                # MultibodyPlant currently sets alpha=0 to make collision
                # geometry "invisible".  Ignore those geometries here.
                if alpha == 0:
                    continue

                geom_name = self._format_geom_name(source_name, frame_name, j)
                X_FG = inspector.GetPoseInFrame(g_id).GetAsMatrix4()

                geom_position = tuple(inspector.GetPoseInFrame(g_id).translation())
                quaternion = inspector.GetPoseInFrame(g_id).rotation().ToQuaternion()
                geom_quaternion = (quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z())

                tfs.append(X_FG)

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

                shape = inspector.GetShape(g_id)
                if isinstance(shape, Box):
                    # Blender cubes are 2x2x2 by default
                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="cube",
                        # scale=[x*0.5 for x in geom.float_data[:3]],
                        scale=[shape.width()*0.5, shape.depth()*0.5, shape.height()*0.5],
                        location=geom_position,
                        quaternion=geom_quaternion,
                        material=material_key)

                elif isinstance(shape, Sphere):
                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="sphere",
                        scale=shape.radius(),
                        location=geom_position,
                        quaternion=geom_quaternion,
                        material=material_key)

                elif isinstance(shape, Cylinder):
                    # Blender cylinders are r=1, h=2 by default
                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="cylinder",
                        scale=[shape.length(), shape.length(), shape.radius()],
                        location=geom_position,
                        quaternion=geom_quaternion,
                        material=material_key)

                elif isinstance(shape, (Mesh, Convex)):
                    geom = g.ObjMeshGeometry.from_file(shape.filename()[0:-3] + "obj")

                    do_load_geom = lambda: self.bsi.send_remote_call(
                        "register_object",
                        name="obj_" + geom_name,
                        type="obj",
                        path=shape.filename()[0:-3] + "obj",
                        scale=(shape.scale(), shape.scale(), shape.scale()),
                        location=geom_position,
                        quaternion=geom_quaternion,
                        material=material_key)

                    # Attempt to find a texture for the object by looking for an
                    # identically-named *.png next to the model.
                    # TODO(gizatt): In the long term, this kind of material information
                    # should be gleaned from the SceneGraph constituents themselves, so
                    # that we visualize what the simulation is *actually* reasoning about
                    # rather than what files happen to be present.
                    candidate_texture_path_png = shape.filename()[0:-3] + "png"
                    if not material_key_assigned and os.path.exists(candidate_texture_path_png):
                        material_key_assigned = self.bsi.send_remote_call(
                            "register_material",
                            name=material_key,
                            material_type="color_texture",
                            path=candidate_texture_path_png)

                else:
                    warnings.warn(f"Unsupported shape {shape} ignored")
                    continue

                print('after assigned geoms')

                if not material_key_assigned:
                    material_key_assigned = self.bsi.send_remote_call(
                        "register_material",
                        name=material_key,
                        material_type="color",
                        color=color)

                print('after material_key_assigned')

                # Finally actually load the geometry now that the material
                # is registered.
                do_load_geom()

                print('after do_load_geom()')

            self.link_subgeometry_local_tfs_by_link_name[link_name] = tfs

        for i, camera_tf in enumerate(self.camera_tfs):
            camera_tf_post = self.global_transform.multiply(camera_tf)
            self.bsi.send_remote_call(
                "register_camera",
                name="cam_%d" % i,
                location=camera_tf_post.translation().tolist(),
                quaternion=camera_tf_post.rotation().ToQuaternion().wxyz().tolist(),
                angle=np.pi/2.)

            self.bsi.send_remote_call(
                "configure_rendering",
                camera_name='cam_%d' % i,
                resolution=[640, 480],
                file_format="JPEG",
                taa_render_samples=20,
                cycles=True)

        print('after camera calls')

        if self.env_map_path:
            self.bsi.send_remote_call(
                "set_environment_map",
                path=self.env_map_path)

        print("done loading env map")

    def load1(self):
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
        mock_lcm_subscriber = Subscriber(
            lcm=mock_lcm,
            channel="DRAKE_VIEWER_LOAD_ROBOT",
            lcm_type=lcmt_viewer_load_robot)

        # warnings.filterwarnings("ignore", message="(Advanced) Explicitly dispatches an LCM load message")
        warnings.simplefilter("ignore", DrakeDeprecationWarning)
        DispatchLoadMessage(self._scene_graph, mock_lcm)
        # SendLoadMessage()  TODO
        mock_lcm.HandleSubscriptions(0)
        assert mock_lcm_subscriber.count > 0
        load_robot_msg = mock_lcm_subscriber.message

        # Load all the elements over on the Blender side.
        self.num_link_geometries_by_link_name = {}
        self.link_subgeometry_local_tfs_by_link_name = {}
        for i in range(load_robot_msg.num_links):
            link = load_robot_msg.link[i]
            [source_name, frame_name] = self._parse_name(link.name)
            print('frame_name', frame_name)
            print('link.name', link.name)
            self.num_link_geometries_by_link_name[link.name] = link.num_geom

            tfs = []
            for j in range(link.num_geom):
                geom = link.geom[j]

                # MultibodyPlant currently sets alpha=0 to make collision
                # geometry "invisible".  Ignore those geometries here.
                if geom.color[3] == 0:
                    continue

                geom_name = self._format_geom_name(source_name, frame_name, j)

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
                    print('geom_name', geom_name)
                    print('path', geom.string_data[0:-3])
                    print('scale', geom.float_data[:3])
                    print('loc', geom.position)
                    print('quat', geom.quaternion)
                    print('material_key', material_key)
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
                    print('color', geom.color[:4])

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
                angle=np.pi/2.)

            self.bsi.send_remote_call(
                "configure_rendering",
                camera_name='cam_%d' % i,
                resolution=[640, 480],
                file_format="JPEG",
                taa_render_samples=20,
                cycles=True)

        if self.env_map_path:
            self.bsi.send_remote_call(
                "set_environment_map",
                path=self.env_map_path)

    def DoPublish(self, context, event):
        # TODO(russt): Change this to declare a periodic event with a
        # callback instead of overriding _DoPublish, pending #9992.
        LeafSystem.DoPublish(self, context, event)

        print("In publish at time ", context.get_time())
        if context.get_time() <= 1E-3:
            return

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
                            rotation_quaternion=pose.rotation().ToQuaternion().wxyz().tolist(),
                            hide_render=False)
                    self.num_visible_bounding_boxes = k + 1
            for k in range(bbox_bundle.get_num_bboxes(),
                           self.num_visible_bounding_boxes):
                self.bsi.send_remote_call(
                        "update_object_parameters",
                        name="obj_bb_%d" % k,
                        hide_render=True)

        # print('pose_bundle type', type(pose_bundle))
        for frame_i in range(pose_bundle.get_num_poses()):
            link_name = pose_bundle.get_name(frame_i)
            # print('link_name', link_name)
            [source_name, frame_name] = self._parse_name(link_name)
            model_id = pose_bundle.get_model_instance_id(frame_i)
            # pose_matrix = pose_bundle.get_pose(frame_i)
            pose_matrix = pose_bundle.get_transform(frame_i)
            for j in range(self.num_link_geometries_by_link_name[link_name]):
                # I am sure this can be written shorter than this, this bloat is from
                # when this was isometry-based.
                offset = RigidTransform(
                    self.global_transform.GetAsMatrix4().dot(
                        # pose_matrix.matrix().dot(
                        pose_matrix.GetAsMatrix4().dot(
                            self.link_subgeometry_local_tfs_by_link_name[
                                link_name][j])))
                location = offset.translation()
                quaternion = offset.rotation().ToQuaternion().wxyz()
                geom_name = self._format_geom_name(
                    source_name, frame_name, j)
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
            self.last_out_filepath = out_filepath
            self.bsi.send_remote_call(
                "render", camera_name="cam_%d" % i, filepath=out_filepath)
            
        if self.save_scene:
            scene_filepath = self.out_prefix + "_scene.blend"
            self.bsi.send_remote_call(
                "save_current_scene", path=scene_filepath)
        self.current_publish_num += 1

        if self.show_figure:
            plt.pause(0.01)
        print("Rendered a frame at %f seconds sim-time." % (
            context.get_time()))
