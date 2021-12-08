![trip_logo](docs/source/trip_logo.svg)

[![Code Grade](https://api.codiga.io/project/29339/score/svg](https://frontend.code-inspector.com/project/29339/dashboard) 
[![codecov](https://codecov.io/gh/TriPed-Robot/TriP/branch/main/graph/badge.svg?token=T6TMY8CD8M)](https://codecov.io/gh/TriPed-Robot/TriP)
[![Documentation Status](https://readthedocs.org/projects/trip-kinematics/badge/?version=main)](https://trip-kinematics.readthedocs.io/en/main/?badge=main)
![example workflow](https://github.com/TriPed-Robot/TriP/actions/workflows/python-package.yml//badge.svg)
[![DOI](https://zenodo.org/badge/350709377.svg)](https://zenodo.org/badge/latestdoi/350709377)
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

Installing the latest release (recommended):
```
pip install trip-kinematics
```

Installing the lnewest version from source:
```
git clone https://github.com/TriPed-Robot/TriP
cd TriP
pip install src/
```

## Documentation
The documentation can be found [here](https://trip-kinematics.readthedocs.io/en/main/)


 
