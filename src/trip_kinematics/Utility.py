from numpy import array
from casadi import cos, sin


def identity_transformation():
    return array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=object)


def hom_translation_matrix(tx=0, ty=0, tz=0):
    return array(
        [[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0., 0., 0., 1.]], dtype=object)


def hom_rotation(rotation_matrix):
    matrix = identity_transformation()
    matrix[:3, :3] = rotation_matrix
    return matrix


def quat_rotation_matrix(qw, qx, qy, qz) -> array:
    """Generates a 3x3 rotation matrix from q quaternion

    Args:
        qw (float): part of a quaternion [qw,qx,qy,qz]
        qx (float): part of a quaternion [qw,qx,qy,qz]
        qy (float): part of a quaternion [qw,qx,qy,qz]
        qz (float): part of a quaternion [qw,qx,qy,qz]

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz + qy*qw)], [2*(qx*qy + qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz - qx*qw)], [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)]], dtype=object)


def x_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the x axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]], dtype=object)


def y_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the y axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]], dtype=object)


def z_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the z axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]], dtype=object)


def get_rotation(matrix):
    """Returns the 3x3 rotation matrix of the :py:class`TransformationMatrix`

    Returns:
        numpy.array: The 3x3 rotation matrix
    """
    return matrix[: 3, : 3]


def get_translation(matrix):
    """Returns the translation of the :py:class`TransformationMatrix`

    Returns:
        numpy array: The 3 dimensional translation
    """
    return matrix[: 3, 3]
