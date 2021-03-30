import numpy as np
from casadi import MX, cos, sin


def quad_rotation_matrix(a, b, c, d) -> np.matrix:
    """[summary]

    Args:
        a ([type]): [description]
        b ([type]): [description]
        c ([type]): [description]
        d ([type]): [description]

    Returns:
        np.matrix: [description]
    """
    return np.matrix([[1-2*(c**2+d**2), 2*(b*c-d*a), 2*(b*d + c*a)], [2*(b*c + d*a), 1-2*(b**2+d**2), 2*(c*d - b*a)], [2*(b*d-c*a), 2*(c*d+b*a), 1-2*(b**2+c**2)]], dtype=object)


def x_axis_rotation_matrix(theta):
    """[summary]

    Args:
        theta ([type]): [description]

    Returns:
        [type]: [description]
    """
    return np.matrix([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]], dtype=object)


def y_axis_rotation_matrix(theta):
    """[summary]

    Args:
        theta ([type]): [description]

    Returns:
        [type]: [description]
    """
    return np.matrix([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]], dtype=object)


def z_axis_rotation_matrix(theta):
    """[summary]

    Args:
        theta ([type]): [description]

    Returns:
        [type]: [description]
    """
    return np.matrix([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]], dtype=object)


class Homogenous_transformation_matrix:
    """[summary]
    """

    def __init__(self, a=0, b=0, c=0, d=0, tx=0, ty=0, tz=0, conv='quad', rx=0, ry=0, rz=0):
        """[summary]

        Args:
            a (int, optional): [description]. Defaults to 0.
            b (int, optional): [description]. Defaults to 0.
            c (int, optional): [description]. Defaults to [summary] 0.
        """
        self.matrix: np.matrix = np.matrix(
            [[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0., 0., 0., 1.]], dtype=object)
        if conv == 'quad':
            self.matrix[:3, :3] = quad_rotation_matrix(a, b, c, d)
        if conv == 'xyz':
            self.matrix[:3, :3] = x_axis_rotation_matrix(
                rx) @ y_axis_rotation_matrix(ry) @ z_axis_rotation_matrix(rz)
        if conv == 'zyx':
            self.matrix[:3, :3] = z_axis_rotation_matrix(
                rz) @ y_axis_rotation_matrix(ry) @ x_axis_rotation_matrix(rx)

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

    def get_MX(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return MX(self.matrix)

    def times(self, second):
        """[summary]

        Args:
            second ([type]): [description]

        Returns:
            [type]: [description]
        """
        new = Homogenous_transformation_matrix()
        new.matrix = self.matrix @ second.matrix
        return new

    def __str__(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return str(self.matrix)
