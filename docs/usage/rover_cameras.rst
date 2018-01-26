=============
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

------------
Launch files
------------

The only launch file in the package right now is the ``rover_cameras.launch`` file. This launches all the cameras. It currently publishes the following streams:

==================== =======
Stream               Content
==================== =======
/belly_cam/          Rear underside camera, not rotated
/belly_cam_rotated/  Rear underside camera, correct orientation
/head_cam/           Head camera (to change to left and right soon)
==================== =======

