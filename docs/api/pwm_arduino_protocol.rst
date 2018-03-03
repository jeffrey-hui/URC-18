====================
Arduino Protocol (eml_uberdriver)
====================

The eml_uberdriver protocol is designed for Encoders, Motors and Limitswitches.

The device is *not* register based, and instead works more like a UART protocol than anything else.

In order to send commands to the device, one byte indicating the command type is sent, followed by the command payload (arguments)

The commands are as follows:

Commands
========

``0x01`` - open pin as motor
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

.. note:

    This command is usually implemented as a hardware reset on the arduino, so if devices connected to it are sensitive to that, be aware of it.

``0x04`` - open pins as encoder
-------------------------------

Encoders take two pins, let's call them *A* and *B*. These pins are sent in this format: ::

    0x04 0xAA 0xBB
``AA`` is the pin for *A* as an unsigned byte, and ``BB`` is the same but for pin *B*. This command will (like the servo create command) return
one unsigned byte known as the encoder name.

.. important::

    Encoder names are what refer to the encoder.

.. note::

    Encoders start at zero when created with this command

``0x05`` - zero encoder
-----------------------

This command will zero an encoder. The format is this: ::

    0x05 0xEE
``EE`` is the name of the encoder to zero as an unsigned byte.

``0x06`` - read encoder
-----------------------

To read an encoder, use this command. The format is this: ::

    0x05 0xEE
``EE`` is the name of the encoder to read.

This command will return a *signed* 4-byte (32-bit) integer denoting the current relative position of the encoder since last zero.

.. note::

    Positive values mean clockwise, negative means anticlockwise.
