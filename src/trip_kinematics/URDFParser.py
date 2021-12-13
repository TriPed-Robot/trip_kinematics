import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Dict, List

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
from trip_kinematics.Transformation import Transformation


def from_urdf(filename: str) -> List[Transformation]:
    """Generates a list of :py:class`Transformations` describing a robot specified in a URDF file.

    Args:
        filename (str): Path to URDF file.

    Raises:
        ValueError: Could not parse URDF file.

    Returns:
        List[Transformation]: List of transformations that describe the robot.
    """
    tree = ET.parse(filename)
    root = tree.getroot()
    joints = root.findall('joint')
    joint_name_to_transformations = {}

    joint_tree_dict = build_joint_tree_dict(joints)

    root_joints = [name for name, value in joint_tree_dict.items() if not value['parent']]

    for i, joint in enumerate(joints):
        name = joint.get('name')
        if not name:
            raise ValueError('Missing name field in URDF file')
        joint_tree_dict[name]['joint_index'] = i

    for joint in joints:
        joint_name_to_transformations[joint.get('name')] = get_transformations_for_joint(joint)

    transformations = []
    for joint in root_joints:
        transformations.extend(create_transformations_from_tree(joint,
                                                                joint_tree_dict,
                                                                joint_name_to_transformations))

    return transformations


def build_joint_tree_dict(joints: List[ET.Element]) -> Dict[str, Dict]:
    """Creates a dictionary representing parent-child relationships between joints (can be used to
    build a tree of joints).

    Args:
        joints (List[ET.Element]): List of <joint> tags from the URDF file.

    Returns:
        dict[str, Dict]: A dictionary representing parent-child relationships between joints.
    """
    # Keep track of parent and children links of joints
    # and use that to search for the parent joint of each joint
    joint_tree_dict = {
        joint.get('name'): {
            'child_link': joint.find('child').get('link'),
            'parent_link': joint.find('parent').get('link')
        } for joint in joints}

    parent_link_to_joint = defaultdict(list)
    child_link_to_joint = defaultdict(list)

    for joint in joints:
        parent_link_to_joint[joint.find('parent').get('link')].append(joint.get('name'))
        child_link_to_joint[joint.find('child').get('link')].append(joint.get('name'))

    for val in joint_tree_dict.values():
        val['parent'] = child_link_to_joint[val['parent_link']]
        val['child'] = parent_link_to_joint[val['child_link']]

    return joint_tree_dict


def align_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Generates a rotation matrix that makes b parallel to a.

    Args:
        a (np.ndarray): 3D Vector.
        b (np.ndarray): 3D Vector.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)

    # if the vectors are parallel but in opposite directions, return a 180 degree rotation
    if np.array_equal(a, -b):
        return -np.identity(3)

    align_axis = np.cross(a, b)
    cos_angle = np.dot(a, b)
    k = 1 / (1 + cos_angle)

    result = np.array([[(align_axis[0] * align_axis[0] * k) + cos_angle,
                        (align_axis[1] * align_axis[0] * k) - align_axis[2],
                        (align_axis[2] * align_axis[0] * k) + align_axis[1]],
                       [(align_axis[0] * align_axis[1] * k) + align_axis[2],
                        (align_axis[1] * align_axis[1] * k) + cos_angle,
                        (align_axis[2] * align_axis[1] * k) - align_axis[0]],
                       [(align_axis[0] * align_axis[2] * k) - align_axis[1],
                        (align_axis[1] * align_axis[2] * k) + align_axis[0],
                        (align_axis[2] * align_axis[2] * k) + cos_angle]])
    return result


def get_transformations_for_joint(joint: ET.Element) -> List[List]:
    """Generates the parameters for up to 4 transformations for the input joint. These correspond
    to: 1. translation and 2. rotation (both taken from the origin tag of the URDF file), then
    3. a rotation which ensures joint movement is aligned with the axes of the local coordinate
    system, then 4. a transformation which represents joint movement (the only one with state
    variables).

    Args:
        joint (ET.Element): A <joint> tag in the URDF file.

    Raises:
        ValueError: Could not parse URDF file.

    Returns:
        List[List]: A list of parameters for up to four py:class`Transformation` objects that
        describe the input joint.
    """
    # read properties from urdf
    name = joint.get('name')
    type_ = joint.get('type')
    origin = joint.find('origin')
    joint_transformations = []

    try:
        assert name
        assert type_
        assert origin
        xyz_vals = origin.get('xyz')
        rpy_vals = origin.get('rpy')
        assert xyz_vals
        assert rpy_vals
        axis = joint.find('axis')
        assert axis is not None
    except AssertionError as e:
        raise ValueError('Error: Invalid URDF file ({})'.format(e))

    # Translation and rotation
    xyz = np.array(list(map(float, xyz_vals.split(' '))))
    rpy = np.array(list(map(float, rpy_vals.split(' '))))

    # Fixed and floating joints have no axis
    if type_ in ['fixed', 'floating']:
        axis = None
    else:
        axis = np.array(list(map(float, axis.split(' '))))
        axis = axis / np.linalg.norm(axis)

    # For each joint, define four transformations
    # ? (maybe the order should be swapped, depends on the urdf specification i think)
    tra = [name + '_tra', {'tx': xyz[0], 'ty': xyz[1], 'tz': xyz[2]}, []]
    rot = [name + '_rot', {'rx': rpy[0], 'ry': rpy[1], 'rz': rpy[2]}, []]

    joint_transformations.extend([rot, tra])

    if axis is not None:
        # align axis to x axis
        align_transformation = align_vectors(np.array([0, 0, 1]), axis)
        align_eulers = \
            ScipyRotation.from_matrix(align_transformation).as_euler('xyz', degrees=False)
        sta = [name + '_sta',
               {'rx': align_eulers[0], 'ry': align_eulers[1], 'rz': align_eulers[2]},
               []]

        joint_transformations.append(sta)

    if type_ in ['continuous', 'revolute']:  # TODO make sure that it always uses rz
        mov = [name + '_mov', {'rz': 0}, ['rz']]

    elif type_ == 'prismatic':  # TODO is this also along the z axis
        mov = [name + '_mov', {'tz': 0}, ['tz']]

    elif type_ == 'floating':  # TODO what do we really need to do here? do we need the sta
        mov = [name + '_mov',
               {'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
               ['tx', 'ty', 'tz', 'rx', 'ry', 'rz']]

    elif type_ == 'planar':  # TODO are these the right axis?
        mov = [name + '_mov', {'tx': 0, 'ty': 0}, ['tx', 'ty']]

    elif type_ == 'fixed':
        mov = [name + '_mov', {}, []]

    else:
        raise ValueError("unknown joint type \"" + type_ + "\"")

    joint_transformations.append(mov)

    return joint_transformations


def create_transformations_from_tree(joint: str,
                                     joint_tree_dict: Dict[str, Dict],
                                     joint_name_to_transformations: Dict[str, List],
                                     parent: Transformation = None) -> List[Transformation]:
    """Recursively builds a tree of py:class`Transformation` objects, starting from the root and
    traversing the tree towards the children.

    Args:
        joint (str): Name of the joint.
        joint_tree_dict (Dict[str, Dict]): Represents the relationships between all joints.
        joint_name_to_transformations (Dict[str, List]): Contains the parameters for the
                                                         py:class`Transformation` objects.
        parent (Transformation, optional): The parent of current joint. Defaults to None for the
                                           root, is set recursively for its children.

    Returns:
        List[Transformation]: List of py:class`Transformation` objects for input node and all its
                              descendants.
    """
    transformations_list = []

    for trans in joint_name_to_transformations[joint]:
        # trans[1] is the parametric description of the transformations
        # If len(trans) > 2, then the transformation has state variables
        is_nonzero = any(i != 0 for i in trans[1].values()) or len(trans[2]) > 0
        if is_nonzero:
            tmp = Transformation(name=trans[0],
                                 values=trans[1],
                                 state_variables=trans[2],
                                 parent=parent)
            transformations_list.append(tmp)
            parent = tmp

    if joint_tree_dict[joint]['child']:
        for child in joint_tree_dict[joint]['child']:
            transformations_list.extend(
                create_transformations_from_tree(child,
                                                 joint_tree_dict,
                                                 joint_name_to_transformations,
                                                 parent)
            )

    return transformations_list
