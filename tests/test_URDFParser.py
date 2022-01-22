import unittest
import os
import pathlib
import json

import defusedxml.ElementTree as ET
import kinpy as kp
import numpy as np
from trip_kinematics import Utility, URDFParser, Robot, forward_kinematics


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
    return state


def state_to_trip(trip_state, kp_state):
    """The function state_to_trip was added to have a uniform way of using the state between kinpy
    and TriP in the tests

    Args:
        state (dict): dictionary with the state name of each joint as it occurs in TriP as the key
                      and a float as the value

    Returns:
        (dict): the input dictionary where the keys are adjusted to the form that kinpy states use

    """

    # this now sets the trip states to be equal to the previously generated kinpy states
    state = {
        joint_name : kp_state["_".join(joint_name.split("_")[:-2])]
        for joint_name in trip_state.keys()
    }

    return state


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


def get_joint_tree_dict(path):
    """Calculates a dictionary representing parent-child relationships between joints from a URDF
    file. Used to build a tree of joints.

    Args:
        path (str): Path to a URDF file.

    Returns:
        Dict[str, Dict]: Dictionary representing parent-child relationships between joints.

    """
    tree = ET.parse(path)
    root = tree.getroot()
    joints = root.findall('joint')

    # pylint: disable=protected-access
    return URDFParser._build_joint_tree_dict(joints)


def compare_urdf_trip_vs_json(path, atol=1e-08):
    """Reads a URDF file, converts it into a TriP robot, calculates forward kinematics, and checks
    whether the results are close to those saved in a JSON file. The JSON file must have the
    same name as the URDF file, except the extension should be changed from .urdf to .json.

    Args:
        path (str): Path to the URDF file to test.
        atol (float, optional): Absolute tolerance of the comparison; see numpy documentation for
                                :py:func:`allclose`. Defaults to 1e-08.

    Raises:
        AssertionError: Results were not within tolerance of each other.

    """
    # Setup TriP robot using the URDF parser
    try:
        robot = Robot(URDFParser.from_urdf(path))
        state_init = initialize_state(robot)
    except ValueError as err:
        raise ValueError(
            f'File {path} is not valid. Unsupported joint type? (error was {err})'
        ) from err

    path_json = pathlib.Path(path).with_suffix('.json')
    with open(path_json, encoding='utf8') as json_file:
        for robot_position in json.load(json_file):
            # First set the state from the file to TriP
            state = robot_position['state']
            transformations = robot_position['transformations']
            robot.set_actuated_state(state_to_trip(state_init, state))

            for joint, joint_dict in get_joint_tree_dict(path).items():
                # Convert position and quaternion rotation saved in JSON to homogenous matrix
                pos_and_rot = transformations[joint_dict['child_link']]
                pos = pos_and_rot["pos"]
                rot = pos_and_rot["rot"]
                pos_homogenous = Utility.hom_translation_matrix(*pos).astype('float')
                rot_matrix = Utility.quat_rotation_matrix(*rot).astype('float')
                rot_homogenous = Utility.hom_rotation(rot_matrix).astype('float')
                kp_transformation = pos_homogenous @ rot_homogenous

                # TriP forward kinematics
                trip_transformation = forward_kinematics(robot, joint).astype('float')

                assert np.allclose(kp_transformation, trip_transformation, atol=atol)


class TestStates(unittest.TestCase):
    def test_all_urdf_files(self):
        names = [
            "one_fixed_joint",
            "one_continuous_joint",
            "one_revolute_joint",
            "one_prismatic_joint",
            "fixed_to_revolute_joint",
            "prismatic_to_continuous_joint",
            "large_tree_test",
            "ground_to_many",
        ]

        names_exceptions = [
            # Valid XML, but not valid URDF robots
            ("urdf_invalid_missing_joint_name", ValueError),
            ("urdf_invalid_missing_joint_type", ValueError),
            ("urdf_invalid_unsupported_joint_type", ValueError),

            # Invalid XML
            ("xml_invalid_mismatched_tag", ET.ParseError),
            ("xml_invalid_invalid_token", ET.ParseError),
        ]

        urdf_examples_dir = os.path.join('tests', 'urdf_examples')

        for name in names:
            full_path = os.path.join(urdf_examples_dir, name + '.urdf')
            compare_urdf_trip_vs_json(full_path)

        for name, exception in names_exceptions:
            full_path = os.path.join(urdf_examples_dir, 'invalid_urdf', name + '.urdf')
            with self.assertRaises(exception):
                compare_urdf_trip_vs_json(full_path)

    def test_align_vectors(self):
        test_cases = [
            ([1, 2, 3], [4, 5, 6]),     # Random angle
            ([1, 0, 0], [0, 1, 0]),     # 90 degrees
            ([1, 0, 0], [-1, 0, 0]),    # -180 degrees
            ([0, 0, 1], [0, 0, -1]),    # -180 degrees
            ([0, 1, 4], [0, -1, -4]),   # -180 degrees
            ([-3, 1, 4], [3, -1, -4]),  # -180 degrees
            ([8, 9, 3], [8, 9, 3]),     # zero angle
        ]

        for target, to_align in test_cases:
            target /= np.linalg.norm(target)
            to_align /= np.linalg.norm(to_align)

            rotation_matrix = URDFParser.align_vectors(target, to_align)
            aligned = to_align @ rotation_matrix

            assert np.all(np.isclose(aligned, target))
