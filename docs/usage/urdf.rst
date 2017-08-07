Using the urdf
==============

Our urdf is contained in the `rover_description` package. Various different configurations for it are available (todo: actually add them), which can be
specified with launch file parameters to the `description.launch` file. Right now the current configuration parameters are:

==== ===========
Name Description
---- -----------
======== ================
has_arm  whether or not the arm is on
has_zed  whether or not the zed is on
======== ================

The launch file places the compiled urdf into `/robot_description` on the parameter server.