========
rover_hw
========

``rover_hw`` is the package which contains the HardwareInterface implementation for our rover. For more information on that, see either the
internal documentation and/or the documentation for ros_control.

This package contains only one file defining the interface, but its structure is important

-----------------------------
Hardware Extension Definition
-----------------------------

Everytime we add a new hardware component with ros_control to the rover, a new hardware_interface "extension" is created in its package.
For example, ``rover_drive`` has its implementation in ``src/drive_hw``. This folder is built into a library, and that library needs to provide
a class with the following methods:

Methods
=======

``init(hardware_interface::RobotHW *hw)``
-----------------------------------------

This method is called to initialize the hardware, as well as register any needed interfaces (e.g. ``JointStateInterface`` ) on the RobotHW instance.

``read()``
----------

This method should *only* update the interfaces with new values from the hardware, like encoders or limit switches.
It is called *before* updating the controllers

``write()``
-----------

This method should only *write* to the hardware with new values from the interfaces. It is called *after* updating the controllers.


-------------------------
``hw_node.cpp`` structure
-------------------------

This file contains the implementation of HardwareInterface. The convention for creating extensions to it was defined above.

For each extension you want to add, you must add the containing package as a dependency in the ``package.xml`` and ``CMakeLists.txt`` file.

In the denoted sections in the file, the declaration of your class and calling of its methods should be added, see how the rover_drive package does it
as an example.

