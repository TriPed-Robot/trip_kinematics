# setup.py
import setuptools

setuptools.setup(
    name='trip_kinematics',
    version='1.0.2',
    author='Torben Miller, Jan Baumgärtner',
    license='MIT',
    description='...',
    install_requires=['casadi>=3.5.5', 'numpy>=1.17.4, < 1.20.0'],
    packages=['trip_kinematics', 'trip_robots']
)
