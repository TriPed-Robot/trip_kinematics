import numpy as np
from casadi import cos, sin


class Rotation:
    """Represents a 3D rotation.

    Can be initialized from quaternions, rotation matrices, or Euler angles, and can be represented
    as quaternions. Reimplements a (small) part of the scipy.spatial.transform.Rotation API and is
    meant to be used for converting between rotation representations to avoid depending on SciPy.
    This class is not meant to be instantiated directly using __init__; use the methods
    from_[representation] instead.

    """
    def __init__(self, quat):
        quat /= np.linalg.norm(quat)

        self._w = quat[0]
        self._x = quat[1]
        self._y = quat[2]
        self._z = quat[3]

    @classmethod
    def from_quat(cls, wxyz):
        """Creates a :py:class`Rotation` object from quaternions.

        Args:
            wxyz (np.array): Quaternion in (w, x, y, z) format.

        Returns:
            [type]: [description]
        """
        return cls(wxyz)

    @classmethod
    def from_euler(cls, rpy):
        sin_r = np.sin(0.5 * rpy[0])
        cos_r = np.cos(0.5 * rpy[0])
        sin_p = np.sin(0.5 * rpy[1])
        cos_p = np.cos(0.5 * rpy[1])
        sin_y = np.sin(0.5 * rpy[2])
        cos_y = np.cos(0.5 * rpy[2])

        w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
        x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
        y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
        z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y

        return cls(np.array([x, y, z, w]))

    @classmethod
    def from_matrix(cls, matrix):
        diag1 = matrix[0, 0]
        diag2 = matrix[1, 1]
        diag3 = matrix[2, 2]
        thing = diag1 + diag2 + diag3
        diag = np.array([diag1, diag2, diag3, thing])
        argmax = np.argmax(diag)
        quat = np.zeros(4)

        if argmax != 3:
            i = argmax
            j = (i + 1) % 3
            k = (j + 1) % 3

            quat[i] = 1 - diag[3] + 2 * matrix[i, i]
            quat[j] = matrix[j, i] + matrix[i, j]
            quat[k] = matrix[k, i] + matrix[i, k]
            quat[3] = matrix[k, j] - matrix[j, k]
        else:
            quat[0] = matrix[2, 1] - matrix[1, 2]
            quat[1] = matrix[0, 2] - matrix[2, 0]
            quat[2] = matrix[1, 0] - matrix[0, 1]
            quat[3] = 1 + diag[3]

        return cls(quat)

    def as_quat(self):
        return np.array([self._w, self._x, self._y, self._z])

    def __str__(self):
        return f'''Rotation wxyz=({self._w:.3f}, {self._x:.3f}, {self._y:.3f}, {self._z:.3f})'''

    def __repr__(self):
        return f'''Rotation wxyz=({self._w:.3f}, {self._x:.3f}, {self._y:.3f}, {self._z:.3f})'''


def identity_transformation():
    """Returns a 4x4 identity matix

    Returns:
        numpy.array: a 4x4 identity matrix
    """
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=object)


def hom_translation_matrix(t_x=0, t_y=0, t_z=0):
    """Returns a homogenous translation matrix

    Args:
        t_x (int, optional): Translation along the x axis. Defaults to 0.
        t_y (int, optional): Translation along the y axis. Defaults to 0.
        t_z (int, optional): Translation along the z axis. Defaults to 0.

    Returns:
        numpy.array: A 4x4 homogenous translation matrix
    """
    return np.array(
        [[1, 0, 0, t_x], [0, 1, 0, t_y], [0, 0, 1, t_z], [0., 0., 0., 1.]], dtype=object)


def hom_rotation(rotation_matrix):
    """Converts a 3x3 rotation matrix into a 4x4 homogenous rotation mtrix

    Args:
        rotation_matrix (numpy.array): A 3x3 rotation matrix

    Returns:
        numpy.array:  A 4x4 homogenous rotation matrix
    """
    matrix = identity_transformation()
    matrix[:3, :3] = rotation_matrix
    return matrix


def quat_rotation_matrix(q_w, q_x, q_y, q_z) -> np.array:
    """Generates a 3x3 rotation matrix from q quaternion

    Args:
        q_w (float): part of a quaternion [q_w,q_x,q_y,q_z]
        q_x (float): part of a quaternion [q_w,q_x,q_y,q_z]
        q_y (float): part of a quaternion [q_w,q_x,q_y,q_z]
        q_z (float): part of a quaternion [q_w,q_x,q_y,q_z]

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return np.array([[1-2*(q_y**2+q_z**2), 2*(q_x*q_y-q_z*q_w), 2*(q_x*q_z + q_y*q_w)],
                 [2*(q_x*q_y + q_z*q_w), 1-2 *
                     (q_x**2+q_z**2), 2*(q_y*q_z - q_x*q_w)],
                 [2*(q_x*q_z-q_y*q_w), 2*(q_y*q_z+q_x*q_w), 1-2*(q_x**2+q_y**2)]], dtype=object)


def x_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the x axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return np.array([[1, 0, 0],
                  [0, cos(theta), -sin(theta)],
                  [0, sin(theta), cos(theta)]], dtype=object)


def y_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the y axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return np.array([[cos(theta), 0, sin(theta)],
                  [0, 1, 0],
                  [-sin(theta), 0, cos(theta)]], dtype=object)


def z_axis_rotation_matrix(theta):
    """Generates a matrix rotating around the z axis

    Args:
        theta (float): The angle of rotation in rad

    Returns:
        numpy.array: A 3x3 rotation matrix
    """
    return np.array([[cos(theta), -sin(theta), 0],
                  [sin(theta), cos(theta), 0],
                  [0, 0, 1]], dtype=object)


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
