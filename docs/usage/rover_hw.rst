========
rover_hw
========

The rover_hw package contains the hardware interface and controller node.

-------------
``rover_hw_node``
-------------

``rover_hw_node`` is the node that runs all the controllers. It takes no parameters, as parametrization of hardware interfaces
should be done as compile time constants.

Running this node is enough to start a controller manager with proper hardware interfaces.