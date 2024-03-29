![trip_logo](docs/source/trip_logo.svg)

[![Code Grade](https://api.codiga.io/project/29339/score/svg)](https://frontend.code-inspector.com/project/29339/dashboard) 
[![codecov](https://codecov.io/gh/TriPed-Robot/TriP/branch/main/graph/badge.svg?token=T6TMY8CD8M)](https://codecov.io/gh/TriPed-Robot/TriP)
[![Documentation Status](https://readthedocs.org/projects/trip-kinematics/badge/?version=main)](https://trip-kinematics.readthedocs.io/en/main/?badge=main)
![example workflow](https://github.com/TriPed-Robot/TriP/actions/workflows/python-package.yml//badge.svg)
[![DOI](https://joss.theoj.org/papers/10.21105/joss.03967/status.svg)](https://doi.org/10.21105/joss.03967)
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

Installing the newest version from source:
```
git clone https://github.com/TriPed-Robot/trip_kinematics
cd trip_kinematics
pip install src/
```

## Documentation
The documentation can be found [here](https://trip-kinematics.readthedocs.io/en/main/)

## How to cite this library
If you find this work useful please give credits to the authors by citing:

* Baumgärtner et al., (2022). TriP: A Python package for the kinematic modeling of serial-parallel hybrid robots. Journal of Open Source Software, 7(71), 3967, https://doi.org/10.21105/joss.03967

```
@article{Baumgärtner2022,
  doi = {10.21105/joss.03967},
  url = {https://doi.org/10.21105/joss.03967},
  year = {2022},
  publisher = {The Open Journal},
  volume = {7},
  number = {71},
  pages = {3967},
  author = {Jan Baumgärtner and Torben Miller},
  title = {TriP: A Python package for the kinematic modeling of serial-parallel hybrid robots},
  journal = {Journal of Open Source Software}
}
```


 
