===================
rover_teleoperation
===================

------------
Launch Files
------------

``simple_drive_teleop.launch``
==============================

This launch file starts a ``joy`` node and a drive teleoperation node.
It takes one parameter, ``dev`` which is the joystick to open.

.. important::

   The teleoperation node expects an XBox controller (or compatible), otherwise the mappings will not be correct

-----
Nodes
-----

``arm_ik_joy_teleop``
=================

This node (currently without a launch file) is for commanding the :doc:`ik_joint_controller <rover_ik.rst>` with a joystick.

It listens on the ``joy`` topic for joystick input and outputs to the ``arm_ik_controller/target`` topic.

``simple_drive_joy_node``
=========================

This node commands drive from a diff drive joystick.

It listens on the ``joy`` topic for joystick input and outputs to the ``/left_wheels_controller/cmd`` and ``/right_wheels_controller/cmd`` topics.
