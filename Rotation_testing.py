from scipy.spatial.transform import Rotation as R
import numpy as np


class Rotation:
    def __init__(self, quat):
        quat /= np.linalg.norm(quat)

        self._w = quat[0]
        self._x = quat[1]
        self._y = quat[2]
        self._z = quat[3]

    @classmethod
    def from_quat(cls, wxyz):
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


def main():
    r1 = Rotation.from_euler([1, 2, 3])
    sr1 = R.from_euler('xyz', [1, 2, 3])

    matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    r2 = Rotation.from_matrix(matrix)
    sr2 = R.from_matrix(matrix)

    matrix = np.array([[3, 5, 6], [4, 1, 2], [-5, 9, 16]])
    r3 = Rotation.from_matrix(matrix)
    sr3 = R.from_matrix(matrix)

    assert np.allclose(r1.as_quat(), sr1.as_quat())
    assert np.allclose(r2.as_quat(), sr2.as_quat())
    assert np.allclose(r3.as_quat(), sr3.as_quat())

    print('hi')


if __name__ == '__main__':
    main()
