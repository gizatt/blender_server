Remote-Operated Blender
-----------------------

Provides a Blender-Python ZMQ server that can be queried to render
new frames. Provides a very minimal and simplistic interface:

- *Single* client at a time.
- Client can request scene updates, and receives back a response code and
possible response text with fairly minimal explanation of if it worked.
  - Scene updates are formed as 'register', 'edit', and 'remove' requests:
    *everything* is treated as an object with a unique identifying name
    and set of properties, one of which is a string defining the object's
    class. The actual grounding of those objects into the scene is
    object-class dependent.
- Client can request a frame and receive it back.

## Schematic Flow for Rendering a Simple Scene

1) Client registers environment map with a filename attribute.
2) Client registers each material with identifying name and texture path(s) or
colors.
3) Client registers each object under a unique identifying name and material
types.
4) Client registers extra lights under unique names.
5) Client registers camera under a unique name with its focal length, fov, etc.

While True:

6) Client requests a frame from that camera's unique name.
7) Client updates camera location, object location, etc...


## TODOs / Ideas for neat features
- Manage different scenes simultaneously.
- Manage entities in frame trees rather than individual objects in maximal coords.]