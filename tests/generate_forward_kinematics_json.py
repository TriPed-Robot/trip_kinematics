import os
import defusedxml.ElementTree as ET
import random
import json

import kinpy as kp
import numpy as np


def state_to_kp(state):
    """The states in TriP are denoted with the '_mov' tag to indicate that it moves and for example
    a '_tz' tag to indicate a translation along the z axis. As these are not used in kinpy the last
    two tags are removed from the string to yield the proper state name

    Args:
        state (dict): dictionary with the state name of each joint as it occurs in TriP as the key
                      and a float as the value

    Returns:
        (dict): the input dictionary where the keys are adjusted to the form that kinpy states use

    """
    return {
        "_".join(joint_name.split("_")[:-2]): value
        for joint_name, value in state.items()
    }


def initialize_state(robot):
    """Creates a dictionary whose entries each correspond to a movable joint of the input
    :py:class:`Robot`, with all values (joint positions) set to 0.

    Args:
        robot (Robot): A TriP Robot.

    Returns:
        (dict): Dictionary representing the robot's state, with all values initialized to zeros.

    """
    return {
        joint_name: 0
        for joint_name in robot.get_actuated_state()
    }


def create_kinpy_chain(path):
    """Takes a path to a URDF file and converts it into a kinpy kinematic chain.

    Args:
        path (str): Path to a URDF file.

    Returns:
        (Chain): kinpy kinematic chain.

    """
    with open(path, encoding='utf8') as file:
        urdf_data_str = file.read()
        return kp.build_chain_from_urdf(urdf_data_str)


def compare_urdf_trip_vs_kinpy(path, rng_states_count=10, atol=1e-08):
    """Reads a URDF file, converts it into a TriP robot and a kinpy kinematic chain, calculates
    forward kinematics for both, and checks whether the results are within a tolerance of each
    other. This is done first using the state where all joint positions are set to zero, then for
    a number of states where all joint positions are generated randomly.

    Args:
        path (str): Path to the URDF file to test.
        rng_states_count (int, optional): The number of randomized states. Defaults to 10.
        atol (float, optional): The absolute tolerance parameter for numpy :py:func:`allclose`.
                                Defaults to 1e-08.

    Raises:
        AssertionError: Results were not within tolerance of each other.

    """
    # Setup kinpy chain
    try:
        chain_kinpy = create_kinpy_chain(path)
    except KeyError as err:
        raise ValueError(
            f'File {path} is not valid. Unsupported joint type? Missing tag? (error was {err})'
        ) from err

    # First state: initialize all joint values to zero
    state_init = {
        joint_name: 0
        for joint_name in chain_kinpy.get_joint_parameter_names()
    }
    test_states = [state_init]

    # RNG states: initialize a number of states with random values
    for _ in range(rng_states_count):
        new_state = {
            joint: random.uniform(-np.pi, np.pi)
            for joint in state_init.keys()
        }
        test_states.append(new_state)

    # Save forward kinematics for all states
    forward_kinematics = [
        {
            'state': state,
            'transformations': {
                link: {'rot': list(transform.rot), 'pos': list(transform.pos)}
                for link, transform in chain_kinpy.forward_kinematics(state_to_kp(state)).items()
            }
        }
        for state in test_states
    ]

    # Replace .urdf extension with .json
    path_json = path[:-4] + 'json'

    with open(path_json, 'w', encoding='utf8') as file:
        json.dump(forward_kinematics, file, separators=(',', ':'))


if __name__ == '__main__':
    compare_urdf_trip_vs_kinpy('tests/urdf_examples/large_tree_test.urdf')
