import numpy as np
from casadi import MX, cos, sin


def quat_rotation_matrix(q0, q1, q2, q3) -> np.matrix:
    """[summary]

    Args:
        a ([type]): [description]
        b ([type]): [description]
        c ([type]): [description]
        d ([type]): [description]

    Returns:
        np.matrix: [description]
    """
    return np.array([[1-2*(q2**2+q3**2), 2*(q1*q2-q3*q0), 2*(q1*q3 + q2*q0)], [2*(q1*q2 + q3*q0), 1-2*(q1**2+q3**2), 2*(q2*q3 - q1*q0)], [2*(q1*q3-q2*q0), 2*(q2*q3+q1*q0), 1-2*(q1**2+q2**2)]], dtype=object)


def x_axis_rotation_matrix(theta):
    """[summary]

    Args:
        theta ([type]): [description]

    Returns:
        [type]: [description]
    """
    return np.array([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]], dtype=object)


def y_axis_rotation_matrix(theta):
    """[summary]

    Args:
        theta ([type]): [description]

    Returns:
        [type]: [description]
    """
    return np.array([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]], dtype=object)


def z_axis_rotation_matrix(theta):
    """[summary]

    Args:
        theta ([type]): [description]

    Returns:
        [type]: [description]
    """
    return np.array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]], dtype=object)


class Homogenous_transformation_matrix:
    """[summary]
    """

    def __init__(self, q0=0, q1=0, q2=0, q3=0, tx=0, ty=0, tz=0, conv='quat', alpha=0, beta=0, gamma=0):
        """[summary]

        Args:
            a (int, optional): [description]. Defaults to 0.
            b (int, optional): [description]. Defaults to 0.
            c (int, optional): [description]. Defaults to [summary] 0.
        """
        self.matrix: np.array = np.array(
            [[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0., 0., 0., 1.]], dtype=object)
        if conv == 'quat':
            self.matrix[:3, :3] = quat_rotation_matrix(q0, q1, q2, q3)
        if conv == 'xyz':
            self.matrix[:3, :3] = x_axis_rotation_matrix(
                alpha) @ y_axis_rotation_matrix(beta) @ z_axis_rotation_matrix(gamma)

    def get_translation(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return self.matrix[: 3, 3]

    def get_rotation(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return self.matrix[: 3, : 3]

    def __mul__(self, other):
        """[summary]

        Args:
            other ([type]): [description]

        Returns:
            [type]: [description]
        """
        new = Homogenous_transformation_matrix()
        new.matrix = self.matrix @ other.matrix
        #new.matrix = np.matmul(self.matrix, other.matrix)
        return new

    def __str__(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return str(self.matrix)
