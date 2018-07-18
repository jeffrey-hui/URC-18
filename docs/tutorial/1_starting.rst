==================
Starting the rover
==================

Starting the rover is not a very complicated procedure.
First, you need to ssh into the TX2, then start rover.launch.
This process is detailed below.

----------------
SSH-ing to rover
----------------

To ssh into the rover, use ``$ ssh ubuntu@<ip>``

The IP depends on which TX2 is currently being used. If it is marked on top of the TX2, use it otherwise refer to this table.

======== ===============
TX2 Desc Ip Address
======== ===============
On Rover 192.168.137.213
======== ===============

Once on the rover, navigate to ``~/URC-18``

-------------------------
Starting rover launchfile
-------------------------

First, ensure ros is properly sourced with ``$ source /opt/ros/kinetic/setup.bash``
Next, source the rover's setup file with ``$ source rosws/devel/setup.bash``

Finally, start the rover software with ``roslaunch rover rover.launch``.
To start autonomous nodes, add ``start_auton:=true`` to the end of the previous command.

This procedure will start the rover's software, but to control it from a basestation you need to read on.
