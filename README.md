Remote-Operated Blender
-----------------------
<a href="https://www.youtube.com/watch?v=zUS33rvbRsc" target="_blank"><img src="drake_blender_visualizer/ycb_manipulation_renders.gif" 
alt="Preview of YCB manipulation example" border="10" /></a>

<a href="https://www.youtube.com/watch?v=oXHxblWw6YA" target="_blank"><img src="drake_blender_visualizer/ycb_subset_renders.jpg" 
alt="Preview of YCB subset example" border="10" /></a>

`blender-server` provides a client-server abstraction for getting scenes rendered in Blender without having to mess with Blender internals. The idea is that you run a server in the background that serves a [single!] client process at a time by receiving commands to set up scenes (by registering meshes and textures), and returns the rendered images.

## Details

This repo provides:
- Some convenient abstractions for setting up simple scenes
of textured meshes:
   - `blender_scripts/blender_scene_management.py` provides
   the highest-level abstractions, and builds on the other scripts
   in that folder. They can be invoked from Blender python scripts
   directly, presuming you have done the small amount of required
   Blender customization (see below). 
- A very simple ZMQ-based server for remotely operating that
highest-level abstraction. It provides a simple interface for
telling the Blender server to call any of those high-level
functions with arbitrary serializable arguments. It also has
a special command for getting an image back (currently a serialized
jpeg).
- A usage example for [Drake](github.com/RobotLocomotion/drake) (in the `drake_blender_visualizer` folder) that uses the Blender server to render images of a robot simulation scene.

## Why?

It's pretty tempting to want to [use Blender as a Python module](https://blender.stackexchange.com/questions/117200/how-to-build-blender-as-a-python-module), but as far as I can tell, it looks pretty tricky to get that actually working in a way that's easy and portable to install. However, installing Blender itself is very easy, and getting it to run a Python script is also easy. So this repo settles on running a Blender instance as a server process, and connecting to it over IPC to ask it to render images.

# Setup

If on Linux, run:

    ./setup_bionic.py

It should create:
- Create a `./build/` directory
- Download and extract Blender
- Install necessary extra Python libraries in Blender
- Download assets for running examples
- Optionally download Drake (using `--with_drake`)
- Create `./build/setup.bash` that you should source for using this.

If on Ubuntu but not `bionic`, this might still work. (Tested on Xenial, seems OK.) If on another system, you'll want to modify this, or make an alternative setup -- see the script itself for the setup steps required.

# Examples

## Basic examples

- Using the Blender server: launch the Blender server with blender, using e.g. `run_blender_server.sh`. Then run `test_blender_server.py` with any Python that has `zmq`, `numpy`, and `imageio`.
- Using the abstraction layer itself from Blender: launch `render_main_bsm.py` with Blender, using e.g. `run_example_bsh.sh`.

## Visualizing Drake robot simulations

You can launch the YCB object simulation by using `run_blender_server.sh` in one terminal, and then running
`python demo_blender_visualizer_on_ycb_tabletop.py`. This will dump images in `/tmp/ycb_scene_%02/` folders.
This assumes the appropriate assets have been downloaded by running `setup_bionic.py`.
 
A more advanced playback demonstrating visualizing a robot teleoperating some objects is also available. Start by downloading the required files with `drake_blender_visualizer/get_example_trajectory.sh`. Then launch a server with `run_blender_server.sh` in one terminal, and launch the ManipulationStation teleop playback example with `python demo_manipulation_station_with_bounding_boxes.py --log example_trajectory/teleop_log_manual_final.pickle --log_bbox example_trajectory/ycb_detections_full.yaml`. This will dump images in the `/tmp/manipulation_station_ycb` folder. Some convenience scripts for rendering videos are provided in this directory.


# Usage

## Schematic Flow for Rendering a Simple Scene

1) Client registers a environment map with a filename attribute to provide the background spheremap and lighting information.
2) Client registers each material with identifying name and texture path(s) or
colors.
3) Client registers each object under a unique identifying name, with optional
parameters for location, quaternion rotation, material (by identifying name).
4) Client registers extra lights under unique names.
5) Client registers camera under a unique name with its focal length, fov, etc.

While True:

6) Client requests a frame from that camera's unique name.
7) Client updates camera location, object location, etc...


# TODOs / Ideas for neat features

## Immediate
- Make Docker system for launching the Blender server.
- Add option to the Blender server interface to start the Blender
server in a subprocess.
- Light management isn't exposed.
- Camera FOV, focal length, other parameters probably not exposed.
- Textures are reasonable reusable between objects, I think, but objects
themselves are reloaded pretty liberally. Can I fix that?
- Images get saved out to disk + then reloaded for serialization back
to the client. This is really inefficient (hundreds of ms for big ones).
Getting images directly from Blender in background mode is tricky, though --
the standard trick of rigging up a Viewer node and grabbing its pixels only
works if Blender is *not* in background mode.

## Nice-to-have
- Manage different scenes simultaneously.
- Manage entities in frame trees rather than individual objects in maximal coords.
