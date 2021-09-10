How TriP models Robots
**********************



Transformations
-------------------

A Kinematic model is made up of Coordinate systems.
These coordinate systems are connected by transformations.

TriP implements its own :class:`.Transormation` class.

One can distinguish between static transformations and dynamic transformations.
Dynamic transformations change depending on an internal state thereby modeling the joints of a mechanism.



The :class:`.Transormation` class has an attribute that manages the internal state.

In general, states can influence the transformation in arbitrary ways.
Yet robotics uses several standard conventions.

The :class:`.Transormation` class currently supports:

#TODO SUPPORTED CONVENTIONS

#TODO describe how joints can be defined in these conventions

#TODO describe children an parent as well as tree structure.



Kinematic Groups
---------------------

Most kinematic libraries rely only on such transformation objects because they only model open chains.
An example for this is `IKPY <https://github.com/Phylliade/ikpy> `_. 

In an open-chain the position and orientation of a coordinate system depend only on one transformation (its parent).


But, consider the excavator arm below:

.. image:: images/excavator_arm.png
 :alt: excavator_arm

a coordinate system has more than one parent since the transformations form a loop.

Such a loop is called a closed kinematic chain.





Dann geschlossene Ketten und wie das in der Regel geht.

Dann warum wir das nicht machen und Gruppenstruktur einführen

