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

# Summary

The domain of robotics has long been dominated by either serial manipulator arms or parallel robots.
However, with the advent of 3D printing more and more combinations of both can be seen.
These hybrid chains have excellent stiffness, payload to weight ratio and a decent workspace. #TODO citation needed
The price to be payed for these improvements is modelling complexity.
Finding an explicit solution for the inverse or forward kinematics is often not possible and has to be solved as a constrained optimization problem.
Most of the established robotics modeling software’s are out of the box not possible to perform these calculations. #TODO citation needed

TriP a python package designed to close this gap using a modular modelling framework akin to the one proposed by #TODO cite appropriate paper.
It allows the modelling of arbitrary kinematic topologies and is capable of calculating their forward and inverse kinematics.
It does so using a symbolic framework that makes it easy to implement custom case dependent mathematical solvers.

# Statement of need

While a huge number of researchers use hybrid robots #TODO cite researchers (and robots?)
most robotics framework only support kinematic calculations for open chains. #TODO cite such frameworks and say "frameworks such as..."
This requires developers to either shoehorn their robots into such a framework # TODO cite example
or are left to implement their own kinematic calculations from scratch.

Especially during rapid prototyping both can be tedious and time consuming.  #TODO example of hybrid legs for cheap quadrupeds?
TriP is designed to simplify this process by directly modelling hybrid systems and calculating their kinematics.

# Overview

TriP models robots using the Robot class.
A Robot object is made up of Transformation and KinematicGroup objects. The KinematicGroup objects are used to model parallel kinematics while the Transformation objects model serial kinematic.

Groups use the decompositioning approach described in #TODO cite appropriate paper
Here the parallel kinematic is treated as a serial manipulator whose joint state can be mapped to the true joint state of the parallel manipulator.

A excavator actuated using two hydraulic cylinders can thus be described using the following example code:

#TODO Here Excavator example






