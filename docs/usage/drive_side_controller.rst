=====================
drive_side_controller
=====================

``drive_side_controller`` is a controller in the ``rover_drive`` package that relays a single velocity command to multiple joints on
one side of the rover. Its pluginlib name is ``rover_drive/DriveSideController``. This controller uses the velocity interface.

----------
Parameters
----------

``joints``
==========

This is the only parameter in ``drive_side_controller``. It is a list and its contents are the joints to use for this side.
Example:

.. code:: yaml

    joints:
        - back_left_wheel
        - front_left_wheel

This would make the controller control the `back_left_wheel` and `front_left_wheel` with the same command.

------
Topics
------

``drive_side_controller/cmd``
=============================

This topic is subscribed to as a Float64 topic containing the current velocity command for all the joints.
