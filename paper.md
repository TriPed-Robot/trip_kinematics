---
title: 'TriP: A Python package for the kinematic modelling of seriell-parallel hybrid robots'
tags:
  - Python
  - robotics
  - inverse kinematics
  - kinematics
  - parallel manipulator
  - serial manipulator
authors:
  - name: Jan Baumgärtner^[co-first author]
    orcid: 0000-0002-7825-3476
    affiliation: 1
  - name: Torben Miller^[co-first author]
    affiliation: 2
affiliations:
  - name: Heidelberg University
    index: 1
  - name: Independent Researcher
    index: 2
date: 28 October 2021
bibliography: paper.bib
---

# Summary

Robotics has long been dominated by serial or branching manipulators.
When strength and high stiffness are required and small workspaces are of no concern parallel robots are also sometimes used. 
However, the advent of 3D printing as a prototyping method for robotics has increased the need for mechanisms that combine both serial and parallel robots.
These hybrid chains have excellent stiffness, payload to weight ratio, and a decent workspace @survey.
The price to be paid for these improvements is modeling complexity.
Finding an explicit solution for the inverse or forward kinematics is often impossible.
Instead, numerical approaches have to be used which solve constrained optimization problems.
It has been shown by @kumar that most robotics frameworks are not equipped to solve these problems for parallel robots let alone hybrid robots.

TriP is a python package designed to close this gap using a modular modeling framework akin to the one described by @survey .
It allows the modeling of arbitrary kinematic topologies and is capable of calculating their forward and inverse kinematics.
It does so using a symbolic framework that makes it easy to implement custom case-dependent mathematical solvers.

# Statement of Need

While a huge number of researchers use hybrid serial parallel systems such as @PISLA, @verne or @berkley
most robotics frameworks such as openrave @openrave  or the matlab robotics toolbox @matlabrobot only support kinematic calculations for open chains.
Frameworks that do support inverse kinematics calculations for closed or hybrid chains, like copeliasim @coppeliaSim , are often commercial products.
This leaves developers to either shoehorn their hybrid robots into a framework not designed to handle them or be left to implement their kinematic calculations from scratch.

Especially during rapid prototyping, both can be tedious and time-consuming.  
TriP is a lightweight and easy-to-use package designed to simplify this process by directly modeling hybrid systems and calculating their kinematics.
It is thus primarily aimed at reasearchers and engineers who quickly want to build kinematic models in order to test their mechanical designs.

# Overview

TriP models robots using its Robot class.
A Robot object is made up of Transformation and KinematicGroup objects. The KinematicGroup objects are used to model parallel-kinematics while the Transformation objects model serial kinematic. See Figure \ref{hybrid_chain_taxonomy_groups} for reference where the links of each robot is colored according to its group or transformation.

![Different Hybrid Robot types and their object structure \label{hybrid_chain_taxonomy_groups}](hybrid_chain_taxonomy_groups.png)

Transformations can be specified using different conventions, currently, roll, pitch, and yaw angles, as well as quaternions, are supported.
Transformations can be either dynamic or static with dynamic transformations implementing joints.
A few examples can be seen in Figure \ref{sample_trafo}.

![Sample Joints using the Transformation class \label{sample_trafo}](sample_transformations.png)


Groups implement the abstraction approach described by  @survey .
Here the parallel kinematic is abstracted as a serial manipulator whose joint state can be mapped to the true joint state of the parallel manipulator.

For an excavator with two hydraulic cylinders, this results in two groups. Both can be seen in Figure \ref{group_structure}
where one is green and the other one is blue.

![Excavator Arm build from two Groups (green and blue) \label{group_structure}](group_structure.png)

The Figure also demonstrates the mappings, the virtual chain treats the excavator as if his hinges are directly actuated.
The groups also contain a mapping that maps the hinge state onto the state of the hydraulic cylinder.
In this case, such a mapping can be trigonometric, however, it is also possible to compute the mapping by solving the closure equation of the parallel manipulator.

Both groups and transformations can be connected to form transformation trees, this is done by specifying the parent-child relationship on initialization.

To solve the inverse kinematics for a given end-effector, TriP can generate a symbolic representation using casadi @casadi
This symbolic representation can be used to set up a solver object that then solves the inverse kinematics.
While the library already implements a simple inverse kinematics solver the symbolic representation makes it easy to implement custom solvers.

All features of TriP are thoroughly documented with tutorials and examples to help people get started.

# References