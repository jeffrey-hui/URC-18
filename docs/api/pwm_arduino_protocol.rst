====================
Arduino PWM Protocol
====================

The ArduinoPWM system that the current drive code uses has a very simple I2C protocol.

.. warning::

   This protocol will probably change as the hardware is finalized

The device is *not* register based, and instead works more like a UART protocol than anything else.

In order to send commands to the device, one byte indicating the command type is sent, followed by the command payload (arguments)

The commands are as follows:

Commands
========

``0x01`` - open pin
-----------------

This command will open a servo on the arduino. It will return one byte indicating the name of this servo. The format looks like this: ::

    0x01 0xPP
Where ``PP`` is this pin number on the arduino.

.. important::

    Servo names are numbers which refer to a servo. In other commands, use this instead of the pin


``0x02`` - set servo value
------------------------

This command sets the value of a servo in microseconds. The input is the servo name to change, and the microseconds to set it to.
Its format looks like this: ::

    0x02 0xPP 0xVV 0xVV
``PP`` is the name of the servo to set, and ``VVVV`` is an unsigned little-endian short denoting the value in microseconds to set it to.

``0x03`` - reset
--------------

Sending this command on its own will reset the state of the arduino. All servo names and values are lost.