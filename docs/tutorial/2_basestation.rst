=====================
Controlling the rover
=====================

Now that you've started the rover, you need to control it. The first thing you need to do is start the watchdog client.

.. note::
   The watchdog automatically stops the rover if the basestation is not connected for more than 1 second.

To start the watchdog client, use ``$ rosrun rover wclient.py``. 

--------
Drive
--------

Teleoperating the drivetrain is very easy. Start by running ``$ roslaunch rover_teleoperation simple_drive_teleop.launch``. If you get
an error about unable to open joysticks, you may need to find the right joystick device under ``/dev/input/js<N>`` and add ``dev:=/dev/input/js<N>`` to the
end of the command.

The controls are simple (on an xbox style controller):

- left & right stick - tank controls
- dpad up & down - gear up/down (speed levels)

------
Camera
------

Cameras can be viewed using the rqt utility. Adding the image view plugin under visualization allows you to view image topics. To control cameras, you need
to use the camera swapper cli. Information on how it works and the commands it supports can be found at :doc:`../usage/rover_cameras.rst`.

Using rqt, you can view the images by clicking the refresh icon in the image view plugin, then selecting the topic ending with "compressed" and with the name
of the camera you set in the cli.
   
