How to use the urdf parser
**************************
The URDF parser allows the usage of URDF files with TriP by converting the described robot into a list of TriP Transformations.

It can be used directly after importing the TriP library by calling the function trip_kinematics.from_urdf with the path to the URDF file as the argument. Note that the function returns a list of Transformations, which you probably want to create a Robot from in most cases:

.. code-block:: python

    transformations_list = trip_kinematics.urdf_parser.from_urdf(urdf_path)
    robot = Robot(transformations_list)


This means you can also add other Transformations manually on top of those specified in the URDF file, if required.

Also note that the parser includes defaults for certain values if the corresponding URDF tag is missing, specifically:

* <origin> defaults to <origin xyz="0 0 0" rpy="0 0 0" />.

  * (The same also applies if only one of xyz and rpy is specified, with the omitted value defaulting to "0 0 0")

* <axis> defaults to <axis  xyz="0 0 1" />
