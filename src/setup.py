# setup.py
import setuptools

with open("requirements.txt", "r") as fh:
    requirements = fh.readlines()

setuptools.setup(
    name='Trip',
    version='1.0',
    author='Torben Miller, Jan Baumgärtner',
    license='MIT',
    description='...',
    install_requires=[req for req in requirements if req[:2] != "# "],
    packages=['trip_kinematics','examples']
)
