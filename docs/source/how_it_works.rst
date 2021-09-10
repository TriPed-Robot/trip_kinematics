How TriP models Robots
**********************

This page describes how TriP models robots.
It is adviced to read this page before building your own robot.
Especially if the hybrid contains hybrid chains.

Transformations
===============

A Kinematic model is made up of Coordinate systems.
These coordinate systems are connected by transformations.

TriP implements its own :class:`.Transormation` class.

One can distinguish between static transformations and dynamic transformations.
Dynamic transformations change depending on an internal state thereby modeling the joints of a mechanism.



The :class:`.Transormation` class has an attribute that manages the internal state.

In general, states can influence the transformation in arbitrary ways.
Yet robotics uses several standard conventions.

The :class:`.Transormation` class currently supports:

.. TODO SUPPORTED CONVENTIONS

.. TODO describe how joints can be defined in these conventions
.. TODO add code examples for conventions

.. TODO describe children an parent as well as tree structure.

.. TODO exmaple of open chain transformations mit Robot außenrum und referenz zum Robot kapitel. 
.. Dann erwähnen das das nicht immer funktioniert und nächstes kapitel das dann erklärt.


Kinematic Groups
================

Most kinematic libraries rely only on such transformation objects because they only model open chains.
An example for this is `IKPY<https://github.com/Phylliade/ikpy> `_ . 

In an open-chain the position and orientation of a coordinate system depend only on one transformation (its parent).


But, consider the excavator arm below:

.. image:: images/excavator_arm.png
 :alt: excavator_arm

a coordinate system has more than one parent since the transformations form a loop.

Such a loop is called a closed kinematic chain.

.. TODO describe what the classical appoach its

In practice this is computationally expensive and unnecessairy.
Instead one would just calculate the orientation of the elbow joint as a function of  hydraulic cylinder position.
Then one could just treat the system like a normal open chain.

TriP embraces this mapping approach and implements it using the :class:`.KinematicGroup` class.
A :class:`.KinematicGroup` is made up of a `virtual_transformation`, a `actuated_state` and two mappings.
The mappings convert between the state of the `virtual_transformation`, called `virtual_state`, and the `actuated_state`.

.. image:: images/group_structure.png
 :alt: group_structure

As the grahic above implies the virtual_transformation is an open chain.
.. important::
    The virtual_transformation has to be a single open chain without branches.
    The reasons for this will be discussed in the next section

divide a robot into groups
--------------------------------
In the example above the excavator is modeled as a single group.
But is this the only way to divide the excavator into groups?
The excavator has two actuated states and two virtual states.
These are the lengths hydraulic cylinders :math:`a_1`,:math:`a_2` and the arm angles :math:`q_1`,:math:`q_2`.
This can be modeled using two groups one for each cylinder.
The second group only has to declare the first as its parent, like a :class:`.Transormation` would.
The advantage of making multiple smaller groups are twofold:

For one it improves modularity, making it easiert to reuse assembly parts.

But more importantly it reduced computational cost.
To keep virtual and actuated state consistent mapping has to be called every time part of one state changes.
A single group mechanism would mean updating every state.
This problem is especially bad for branching mechanisms.
Consider a for legged robot, setting the actuator of one leg would then mean updating all four legs.
To prevent this problem outright the virtual chain can not contain branches.

In summary groups should be defined as small as possible.
Small in this case refering to the number actuators that have to be grouped together.
The minimum size is defined by the closed chains.
Consider the following mechanism

.. image:: images/closed_chain_group_configs.png
    :alt: group_partitoning

Grouping a) and c) are valid groups, with a) being more performant. 
However the Grouping in b) is not valid. 
The reason is that the state of the top plattform depends on the state of all three green prismatic joints.

.. TODO noch auf offene ketten eingehen, warum sind die keine Gruppen?

These considerations lead to the following guidelines for building hybrid robots:

.. important::
    Every closed chain should be modeled by a Group. 
    Every open chain should be modeled by Transformations.  
    For example:

    .. image:: images/hybrid_chain_taxonomy_groups.png
        :alt: group_partitoning


This means that the excavator is made up of two groups.
Each group is has two mappings between the length hydraulic cylinders :math:`a_i` to the arm angles :math:`q_i`.
These can be calculated using trigonometry:

.. image:: images/geometric_mapping.png
    :alt: geometric_mapping

The corresponding code looks like this:

.. literalinclude:: ../../src/trip_robots/excavator_rr.py
   :language: python
   :linenos:
   :lines: 11-62


actuated state vs virtual state
-------------------------------
If one looks at the code above one can see that the dicionary values of the actuated state in lines 26 and 36 are float values, 
while the values of the virtual states in lines 32 and 39 are diciontaries.

This difference is due to the fact that virtual states always specify convention parameters of a :class:`Transformation`.
Actuated values on the other hand are not associated with a Transformation and thus dont adhere to transformation conventions.

This is an important difference to keep in mind when dealing with both states.
Below are a few examples of joints and how their actuated and virtual states would differ.


.. image:: images/state_differences_normal.png
    :alt: state_differences_normal

.. image:: images/state_differences.png
    :alt: state_differences



Using closure equations
-----------------------
While direct mappings are alway preferable it is not always possible to find a direct function.
In this case one can always resort to the closure equation.
Since TriP is based on mappings the closure equation is used to set up mapping functions who solve the closure equation.
For the mapping from actuated state to virtual state, the actuated states are fixed and the virtual states calculated.
Likewise for the reverse mapping, the virtual state is fixed while the actuated states are calculated.

The setup of the closure equation will require extra transformations.
This can be done by building a full open chain or for simple chains by directly setting up the transformation matrices using the Utility submodule.
In this case of the excavator, the following joints can be defined:

.. image:: images/closure_mapping.png
    :alt: closure_mapping

The solving of the closure equation can be performed by casadi, which TriP also uses for inverse kinematics calulations:

.. literalinclude:: ../../src/trip_robots/excavator_rr.py
   :language: python
   :linenos:
   :lines: 67-143



Defining virtual transformations
--------------------------------

.. TODO example von vorne zeigen, mitvirtueller kette als part, dann 6dof joint (constrained), dann constrained joint (nur orientierung)


