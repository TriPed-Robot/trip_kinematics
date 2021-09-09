from typing import Dict, List
from copy import deepcopy

from trip_kinematics.Utility import hom_translation_matrix, x_axis_rotation_matrix, y_axis_rotation_matrix, z_axis_rotation_matrix, quat_rotation_matrix

def array_find(arr, obj) -> int:
    index = -1
    try:
        index = arr.index(obj)
        return index
    except:
        return -1

class Transformation():
    """Initializes the :py:class:`Transformation` class. 

    Args:
        name (str): The unique name identifying the . No two :py:class:`Transformation` objects of a :py:class`Robot` should have the same name
        values (Dict[str, float]): A parametric description of the transformation. 
        state_variables (List[str], optional): This list describes which state variables are dynamically changable. 
                                               This is the case if the :py:class:`Transformation` represents a joint. Defaults to [].

    Raises:
        ValueError: A dynamic state was declared that does not correspond to a parameter declared in ``values``.
    """

    @staticmethod
    def get_convention(state: Dict[str, float]):
        """Returns the connvention which describes how the matrix  of a :py:class:`Transformation` is build from its state. 

        Args:
            state (Dict[str, float]): :py:attr:'state'

        Raises:
            ValueError: "Invalid key." If the dictionary kontains keys that dont correspond to a parameter of the transformation.
            ValueError: "State can't have euler angles and quaternions!" If the dictionary contains keys correspondig to multiple mutually exclusive conventions.

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
                raise ValueError("Invalid key, acceptable keys are: "+str(valid_keys))
            if array_find(quaternion_keys, key) >= 0:
                got_quaternion = True
            if array_find(euler_keys, key) >= 0:
                got_euler = True

        if got_euler and got_quaternion:
            raise ValueError("State can't have euler angles and quaternions!")

        if got_euler:
            return "euler"
        else:
            return "quaternion"

    def __init__(self, name: str, values: Dict[str, float], state_variables: List[str] = []):

        self._name = name

        if not set(state_variables) <= set(values.keys()):
            raise ValueError(
                "Element of state_variables do not correspond to key(s) of values dictionary. This can happen if both follow different conventions")

        constants = {}
        state = {}

        self.convention = Transformation.get_convention(values)

        for key in values.keys():
            if array_find(state_variables, key) != -1:
                state.setdefault(key, values.get(key))
            else:
                constants.setdefault(key, values.get(key))

        self._state     = state
        self._constants = constants

    def __str__(self):
        return self._name

    def get_state(self):
        return deepcopy(self._state)
    def set_state(self, state: Dict[str, float]):
        for key in state.keys():
            if not key in self._state.keys():
                raise KeyError("The specified keys is not part of the Transformations state. Maybe the Transofmration uses a different convention?")
            self._state[key] = state[key]

            
    def get_name(self):
        return deepcopy(self._name)

    def get_transformation_matrix(self):
        """Returns a homogeneous transformation matrix build from the :py:attr:`state` and :py:attr:`constants`

        Raises:
            RuntimeError: If the convention used in :py:attr:`state` is not supported. Should normally be catched during initialization.

        Returns:
            [type]: A transformation matrix build using the parameters of the  :py:class:`Transformation` :py:attr:`state`
        """

        # collect transformation parameters from state and constants respectively
        tx = self._constants.get('tx') if 'tx' in self._constants.keys() else self._state.get('tx',0)
        ty = self._constants.get('ty') if 'ty' in self._constants.keys() else self._state.get('ty',0)
        tz = self._constants.get('tz') if 'tz' in self._constants.keys() else self._state.get('tz',0)

        matrix = hom_translation_matrix(tx,ty,tz)

        if self.convention == 'euler':
            rx = self._constants.get('rx') if 'rx' in self._constants.keys() else self._state.get('rx',0)
            ry = self._constants.get('ry') if 'ry' in self._constants.keys() else self._state.get('ry',0)
            rz = self._constants.get('rz') if 'rz' in self._constants.keys() else self._state.get('rz',0)

            matrix[:3, :3] = x_axis_rotation_matrix(
                rx) @ y_axis_rotation_matrix(ry) @ z_axis_rotation_matrix(rz)

        elif self.convention == 'quaternion':
            qw = self._constants.get('qw') if 'qw' in self._constants.keys() else self._state.get('qw',0)
            qx = self._constants.get('qx') if 'qx' in self._constants.keys() else self._state.get('qx',0)
            qy = self._constants.get('qy') if 'qy' in self._constants.keys() else self._state.get('qy',0)
            qz = self._constants.get('qz') if 'qz' in self._constants.keys() else self._state.get('qz',0)

            matrix[:3, :3] = quat_rotation_matrix(qw, qx, qy, qz)
        else:
            raise RuntimeError("No Convention. This should normally be catched during initialization. " + 
                               "Did you retroactively change the keys of the Transformation state?")
        
        return matrix


