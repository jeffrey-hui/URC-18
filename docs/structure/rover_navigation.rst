================
rover_navigation
================

``rover_navigation`` is our implementation / configuration of a move_base stack.

.. note:
   Although this file does document how to change the auton parameters, for general usage you should probably just start the launch file
   as described in the usage docs.

----------
``config``
----------

The config folder contains all of the yaml configuration for ``move_base``. There are three files in it.

``planner.yaml``
================

``planner.yaml`` contains all of the configuration for the planners (global and local). For documentation on what the parameters in it mean,
see the docs for ``base_local_planner`` and ``navfn``.

``local_costmap.yaml`` and ``global_costmap.yaml``
==================================================

These files contain the configuration for the local and global costmaps respectively. The documentation for them is in the `costmap_2d` package.

---------------------
Internal Launch Files
---------------------

Although the usage documentation only details one launch file, there are a lot of internal ones with more parametrization.

``move_base.launch``
====================

The ``move_base.launch`` file contains the launch spec for starting move_base itself. It contains two useful parameters, located on lines 6 and 7:

.. code-block:: xml
   :lineno-start: 6
        <param name="base_global_planner" value="global_planner/GlobalPlanner" />
        <param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS" />


This sets the ``base_(global/local)_planner``.

``odometry.launch``
===================

This is one of the most important files in the ``rover_navigation`` package. It contains the sensor fusion configuration. For documentation
on how to configure it, see the documentation for ``robot_localization``.

``visual_odom.launch``
======================

This file contains the parameters (but right now mainly just remappings) for the rtabmap visual odometry.

``rtabmap.launch``
==================

This file contains the parameters for rtabmap's main node. Add mapping parameters here.