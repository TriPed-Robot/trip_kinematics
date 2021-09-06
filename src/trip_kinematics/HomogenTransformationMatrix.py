from numpy import array
from casadi import cos, sin


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


class TransformationMatrix:
    """Class Implementing and Building a simple homogenous transformation matrix.

    Args:
        qw (float, optional): part of a quaternion [qw,qx,qy,qz]. Defaults to 1.
        qx (float, optional): part of a quaternion [qw,qx,qy,qz]. Defaults to 0.
        qy (float, optional): part of a quaternion [qw,qx,qy,qz]. Defaults to 0.
        qz (float, optional): part of a quaternion [qw,qx,qy,qz]. Defaults to 0.
        tx (float, optional): translation along x-axis. Defaults to 0.
        ty (float, optional): translation along y-axis. Defaults to 0.
        tz (float, optional): translation along z-axis. Defaults to 0.
        conv (str, optional): convention used to build the transformation matrix. Defaults to 'quat'.
        rx (float, optional): rotation around the x-axis. Defaults to 0.
        ry (float, optional): rotation around the y-axis. Defaults to 0.
        rz (float, optional): rotation around the z-axis. Defaults to 0.
        """

    def __init__(self, qw=1, qx=0, qy=0, qz=0, tx=0, ty=0, tz=0, conv='quat', rx=0, ry=0, rz=0):


        self.matrix: array = array(
            [[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0., 0., 0., 1.]], dtype=object)
        if conv == 'quat':
            self.matrix[:3, :3] = quat_rotation_matrix(qw, qx, qy, qz)
        if conv == 'xyz':
            self.matrix[:3, :3] = x_axis_rotation_matrix(
                rx) @ y_axis_rotation_matrix(ry) @ z_axis_rotation_matrix(rz)

    def get_translation(self):
        """Returns the translation of the :py:class`TransformationMatrix`

        Returns:
            numpy array: The 3 dimensional translation
        """
        return self.matrix[: 3, 3]

    def get_rotation(self):
        """Returns the 3x3 rotation matrix of the :py:class`TransformationMatrix`

        Returns:
            numpy.array: The 3x3 rotation matrix
        """
        return self.matrix[: 3, : 3]

    def __mul__(self, other):
        """[summary]

        Args:
            other ([type]): [description]

        Returns:
            [type]: [description]
        """
        new = TransformationMatrix()
        new.matrix = self.matrix @ other.matrix
        return new

    def __str__(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return str(self.matrix)
