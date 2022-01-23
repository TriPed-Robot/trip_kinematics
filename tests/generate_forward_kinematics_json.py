import os
import pathlib
import random
import json

import kinpy as kp
import numpy as np
from test_URDFParser import urdf_path_to_json_path, precomputed_kinematics_dir_name


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


def generate_forward_kinematics_json(urdf_path, rng_states_count=10):
    """Calculates forward kinematics for the input URDF file using kinpy and saves these to a
    JSON file.

    Args:
        path (str): Path to the URDF file.
        rng_states_count (int, optional): The number of randomized states. Defaults to 10.

    """
    # Setup kinpy chain
    try:
        chain_kinpy = create_kinpy_chain(urdf_path)
    except KeyError as err:
        raise ValueError(
            f'File {urdf_path} is not valid. Unsupported joint type? Missing tag? (error was {err})'
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

    # Save forward kinematics results and joint positions for all states
    forward_kinematics = [
        {
            'state': state,
            'transformations': {
                link: {'rot': list(transform.rot),
                       'pos': list(transform.pos)}
                for link, transform in chain_kinpy.forward_kinematics(state).items()
            }
        }
        for state in test_states
    ]

    return json.dumps(forward_kinematics, separators=(',', ':'))


if __name__ == '__main__':
    urdf_examples_dir = os.path.join('tests', 'urdf_examples')
    precomputed_kinematics_dir = pathlib.Path(urdf_examples_dir) / precomputed_kinematics_dir_name
    precomputed_kinematics_dir.mkdir(exist_ok=True)
    # Iterate through files for which we compute forward kinematics. Skip subdirectories of
    # urdf_examples_dir, because as of now, the only subdirectory contains (intentionally) broken
    # URDFs. If that changes, change this too.
    for entry in os.scandir(urdf_examples_dir):
        if entry.is_file() and pathlib.Path(entry).suffix == '.urdf':
            with open(urdf_path_to_json_path(entry.path), 'w', encoding='utf8') as file:
                forward_kinematics = generate_forward_kinematics_json(entry.path)
                file.write(forward_kinematics)
