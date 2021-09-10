![trip_logo](docs/source/trip_logo_dark.png)
# TriP
TriP is a python library designed to calculate the forward- and inverse-kinematics of rigid mechanisms.

This includes open chains, closed chains, and even combinations of both.

## Features

- Model any robot (including closed and hybrid chains)
- Generate symbolic representations of forward kinematics
- Compute Jacobian matrices for differential kinematics
- Compute the inverse kinematics of arbitrary rigid mechanisms
- Compute the Inverse Kinematics in position and/or orientation 
- Support arbitrary joint types and quaternions
- Includes several ready to use examples (TriPed robot, Excavator Arm)
- TriPs validates the inverse kinematics algorithms with extensive testing using analytic solutions.

## Installation

Using PyPI (recommended):
`
pip install trip_kinematics
`

Installing from Source:
`
git clone https://github.com/TriPed-Robot/TriP
cd TriP
pip install src/
`



 