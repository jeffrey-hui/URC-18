========
rover_ik
========

-----------------------
``ik_joint_controller``
-----------------------

The ``rover_ik::IKJointController`` is a controller in ros_control that does inverse kinematics on a set of joints and feeds the result to a PID loop.
A full usage example can be seen in rover_control.

Parameters
==========

`joints`
--------

This parameter is a list of joints to control, each one should provide an ``EffortJointInterface``

`gains`
-------

Each subparameter in this parameter corresponds to PID gains for a joints, for example:

.. code:: yaml

   gains:
      arm_base_to_post: {p: 7, i: 1, d: 0, i_clamp: 1}

This sets the gains for joint ``arm_base_to_post`` to P = 7, I = 1, D = 0 and I_Clamp = 1.

`base_link` and `tip_link`
--------------------------

This correspond to the *links* at each end of the joint chain, base being the start and tip being the end.

Topics & Services
=================

`controller_name/target`
------------------------

Controller listens on this topic for the target pose of the `tip_link`, as defined in the parameters. The coordinates for this pose
are relative to the ``base_link``. (testing required)

`controller_name/request_position`
----------------------------------

This service can be called to return the current position of the arm as calculated via forward kinematics. Passing this back into
``target`` should cause no effect to the arm.
