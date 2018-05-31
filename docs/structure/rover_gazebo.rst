============
rover_gazebo
============

``rover_gazebo`` is the package that contains all of the gazebo stuff, like worlds and gazebo-specific textures/models.
It has a few folders, organizing the above gazebo-relating things.

There is no code in the ``rover_gazebo package`` at this time, although should we need gazebo plugins they will be placed in another
package, probably called ``rover_gazebo_plugins``

---------
``world``
---------

The ``world`` folder contains all of the gazebo worlds, in ``.world`` format.
The current worlds are:

==================== ==========================================================================================================
Filename             Content/function
==================== ==========================================================================================================
outdoor_world.gazebo An outdoor world with a heightmap. Good for large-scale testing where a natural-ish landscape is important
==================== ==========================================================================================================

Each world *should* have a launch file associated with it, which should spawn the rover with the same parameters as the (upcoming)
master launch file. An easy way to create this is to use the ``empty_world.launch`` file, which allows for customizing the world location.

---------
``media``
---------

The ``media`` folder is where all of the gazebo "media" is: textures, models, heightmaps, etc.
Any objects should be in their own folder, for example the "height" object (which is the heightmap used in the outdoor world) is in the
folder ``media/height``. Textures should be in a folder named ``media/<object name>/textures``. Models should be in a folder named ``media/<object name>/models``.
Any other files the object needs, including its ``.sdf`` file, should be located in the root folder for that object.

Here is a table summarizing this info:

============================= =================== 
Folder location               Files located there
============================= ===================
``media/<obj name>``          Files for the object named ``obj_name``.
``media/<obj name>/textures`` Textures for the object named ``obj_name``.
``media/<obj name>/models``   Models for the object named ``obj_name``.
============================= ===================