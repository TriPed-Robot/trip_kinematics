from typing import Dict, List
from copy import deepcopy

from trip_kinematics.Utility import hom_translation_matrix, x_axis_rotation_matrix
from trip_kinematics.Utility import y_axis_rotation_matrix, z_axis_rotation_matrix
from trip_kinematics.Utility import quat_rotation_matrix
import trip_kinematics.KinematicGroup


def array_find(arr, obj) -> int:
    """A helpher function which finds the index of an object in an array.
       Instead of throwing an error when no index can be found it returns -1.

    Args:
        arr : the array to be searched
        obj : the object whose index is to be found.

    Returns:
        int: The index of obj in the array. -1 if the object is not in the array
    """
    index = -1
    try:
        index = arr.index(obj)
        return index
    except:
        return -1


class Transformation():
    """Initializes the :py:class:`Transformation` class.

    Args:
        name (str): The unique name identifying the Transformation.
                    No two :py:class:`Transformation` objects of a :py:class`Robot`
                    should have the same name
        values (Dict[str, float]): A parametric description of the transformation.
        state_variables (List[str], optional): This list describes which state variables are
                                               dynamically changable.
                                               This is the case if the :py:class:`Transformation`
                                               represents a joint. Defaults to [].

    Raises:
        ValueError: A dynamic state was declared that does not
                    correspond to a parameter declared in ``values``.
    """

    @staticmethod
    def get_convention(state: Dict[str, float]):
        """Returns the connvention which describes how the matrix
           of a :py:class:`Transformation` is build from its state.

        Args:
            state (Dict[str, float]): :py:attr:'state'

        Raises:
            ValueError: "Invalid key." If the dictionary kontains keys that dont
                        correspond to a parameter of the transformation.
            ValueError: "State can't have euler angles and quaternions!"
                        If the dictionary contains keys correspondig
                        to multiple mutually exclusive conventions.

        Returns:
            [type]: A string describing the convention
        """

        valid_keys = ['tx', 'ty', 'tz', 'qw', 'qx',
                      'qy', 'qz', 'rx', 'rz', 'ry']

        quaternion_keys = valid_keys[3:7]
        euler_keys = valid_keys[7:]
        got_quaternion = False
        got_euler = False

        for key in state.keys():
            if array_find(valid_keys, key) == -1:
                raise ValueError(
                    "Invalid key, acceptable keys are: "+str(valid_keys))
            if array_find(quaternion_keys, key) >= 0:
                got_quaternion = True
            if array_find(euler_keys, key) >= 0:
                got_euler = True

        if got_euler and got_quaternion:
            raise ValueError("State can't have euler angles and quaternions!")

        if got_euler:
            return "euler"
        return "quaternion"

    def __init__(self, name: str, values: Dict[str, float],
                 state_variables: List[str] = None, parent=None):

        self._name = name
        self.children = []

        if parent is None:
            self.parent = name
        elif isinstance(parent, (trip_kinematics.KinematicGroup, Transformation)):
            self.parent = str(parent)
            parent.add_children(name)
        else:
            raise TypeError(
                "The parent of a Transformation must be either " +
                "a KinematicGroup or a Transformation")

        if state_variables is None:
            state_variables = []
        if not set(state_variables) <= set(values.keys()):
            raise ValueError(
                "Element of state_variables do not correspond to key(s) of values dictionary." +
                " This can happen if both follow different conventions")

        constants = {}
        state = {}

        self.convention = Transformation.get_convention(values)

        for key in values.keys():
            if array_find(state_variables, key) != -1:
                state.setdefault(key, values.get(key))
            else:
                constants.setdefault(key, values.get(key))

        self._state = state
        self._constants = constants

    def __str__(self):
        return self._name

    def get_state(self):
        """Returns a copy of the :py:attr:`_state`
           attribute of the :py:class:`Transformation` object.

        Returns:
            Dict[str,float]: a copy of the :py:attr:`_state`
        """
        return deepcopy(self._state)

    def set_state(self, state: Dict[str, float]):
        """Sets the state of the :py:class:`Transformation` object.


        Args:
            state (Dict[str, float]): Dictionary with states that should be set.
                                      Does not have to be the full state.

        Raises:
            KeyError: If a key in the argument is not valid state parameter name.
        """
        for key in state.keys():
            if not key in self._state.keys():
                raise KeyError(
                    "The specified keys is not part of the Transformations state." +
                    "Maybe the Transofmration uses a different convention?")
            self._state[key] = state[key]

    def get_name(self):
        """Returns the :py:attr:`_name` of the :py:class:`Transformation`

        Returns:
            str: a copy the :py:attr:`_name` attribute
        """
        return deepcopy(self._name)

    def get_transformation_matrix(self):
        """Returns a homogeneous transformation matrix build
           from the :py:attr:`state` and :py:attr:`constants`

        Raises:
            RuntimeError: If the convention used in :py:attr:`state` is not supported.
                          Should normally be catched during initialization.

        Returns:
            [type]: A transformation matrix build using the parameters
                    of the :py:class:`Transformation` :py:attr:`state`
        """

        # collect transformation parameters from state and constants respectively
        t_x = self._constants.get(
            'tx') if 'tx' in self._constants.keys() else self._state.get('tx', 0)
        t_y = self._constants.get(
            'ty') if 'ty' in self._constants.keys() else self._state.get('ty', 0)
        t_z = self._constants.get(
            'tz') if 'tz' in self._constants.keys() else self._state.get('tz', 0)

        matrix = hom_translation_matrix(t_x, t_y, t_z)

        if self.convention == 'euler':
            r_x = self._constants.get(
                'rx') if 'rx' in self._constants.keys() else self._state.get('rx', 0)
            r_y = self._constants.get(
                'ry') if 'ry' in self._constants.keys() else self._state.get('ry', 0)
            r_z = self._constants.get(
                'rz') if 'rz' in self._constants.keys() else self._state.get('rz', 0)

            matrix[:3, :3] = x_axis_rotation_matrix(
                r_x) @ y_axis_rotation_matrix(r_y) @ z_axis_rotation_matrix(r_z)

        elif self.convention == 'quaternion':
            q_w = self._constants.get(
                'qw') if 'qw' in self._constants.keys() else self._state.get('qw', 0)
            q_x = self._constants.get(
                'qx') if 'qx' in self._constants.keys() else self._state.get('qx', 0)
            q_y = self._constants.get(
                'qy') if 'qy' in self._constants.keys() else self._state.get('qy', 0)
            q_z = self._constants.get(
                'qz') if 'qz' in self._constants.keys() else self._state.get('qz', 0)

            matrix[:3, :3] = quat_rotation_matrix(q_w, q_x, q_y, q_z)
        else:
            raise RuntimeError("No Convention. " +
                               "This should normally be catched during initialization. " +
                               "Did you retroactively change the keys of the Transformation state?")

        return matrix

    def add_children(self, child: str):
        """Adds the name of a :py:class:`KinematicGroup` or :py:class:`Transformation`
           as a child.

        Args:
            child (str): the name of a :py:class:`KinematicGroup` or :py:class:`Transformation`
        """
        self.children.append(child)
