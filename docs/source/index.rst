.. TriP documentation master file, created by
   sphinx-quickstart on Tue Mar 23 13:52:13 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to TriP's documentation!
================================

Have you ever worked with a robot with hydraulic actuators?
Or ever have to compensate bad motors by having them moving the joint via a complicated linkage?

Then you have worked with a hybrid kinematic chain.

TriP is a python library designed to calculate the forward- and inverse-kinematics of such chains.
Since hybrid chains are the most general type of rigid mechanisms this includes almost all robots.

Features
--------

- Model any robot (including closed and hybrid chains)
- Generate symbolic representations of forward kinematics
- Compute Jacobian matrices for differential kinematics
- Compute the inverse kinematics of arbitrary rigid mechanisms
- Compute the Inverse Kinematics in position and/or orientation 
- Support arbirtrary joint types and quaternions
- Includes a  number of ready to use examples (TriPed robot, Excavator Arm)
- TriPs validates the inverse kinematics algorithms with extensive testing using analytic solutions.


.. toctree::

    what_is_trip
    how_it_works
    usage
    tutorials
    code_docu

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
