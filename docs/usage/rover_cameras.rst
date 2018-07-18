============
rover_cameras
=============

The nodes in the ``rover_cameras`` package are designed to manipulate rover camera streams. The launch files are what launch them
to get all the feeds actually running.

-----
Nodes
-----

``image_rotater``
=================

The ``image_rotater`` node takes in one input stream and rotates it by a constant amount.

Topics
------

- ``~image_rotater/image_in`` - image stream to rotate
- ``~image_rotater/image_out`` - published image stream

Parameters
----------

- ``angle`` - angle in degrees to rotate by

``camera_diagnostics``
======================

The ``camera_diagnostics`` node monitors ``/dev/video`` paths and publishes diagnostic information for them.

Parameters
----------

- ``~expected_nums`` - video numbers to monitor

``swapper.py``
==============

Runs and manages ``usb_cam`` nodes to deal with cameras with a ros api. Typically you use the cli to interact with it.

------------
Launch files
------------

The only launch file in `rover_cameras` starts the camera swapper.

---
CLI
---

.. important::
        The cli for camera swapping requires you install the `prompt_toolkit` python package.

The cli (node name ``swapper_cli.py``) allows you to interact with the swapper.
Each camera is just represented as a name and a video device, the node doesn't actually store these things.

The CLI has built in help, (as well as autocomplete) but documentation is also provided here:

Commands
========

``quit``
--------

Quits the CLI.

``load``
--------

Two parameters, first is the camera name, next is the video device (``/dev/videoN``). Optionally takes a parameter, which is the
preffered device to unload if we cannot run this many cameras. Using this parameter allows you to quickly switch cameras.

```
> load belly_cam /dev/video0 head_cam
```
Swaps head_cam with belly_cam

``unload``
----------

Unloads the camera (by name) given to it as a parameter.

``list``
--------

Lists all running cameras by name.

