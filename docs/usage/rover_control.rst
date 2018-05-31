=============
rover_control
=============

------------
Launch Files
------------

``rover_control.launch``
========================


This launch file is for loading controller parameters and starting ``robot_state_publisher``. It takes two parameters:

- ``use_fake`` - default ``false`` - whether or not to load the fake_controllers file.
- ``controllers`` - default depends on use_fake - which controllers to load by default.

The launch file also starts the :doc:`rover_hw node <rover_hw.rst>` if the ``use_fake`` parameter is set to false.
