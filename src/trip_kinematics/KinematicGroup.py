from typing import Dict, List, Callable, Union
from trip_kinematics.HomogenTransformationMartix import HomogenousTransformationMatrix
from copy import deepcopy


def array_find(arr, obj) -> int:
    index = -1
    try:
        index = arr.index(obj)
        return index
    except:
        return -1


def virtual_state_to_keys(virtual_state):
    return list(map(lambda obj: obj.keys(), virtual_state))


def validate_keys_and_get_convention(state: Dict[str, float]):

    valid_keys = ['tx', 'ty', 'tz', 'qw', 'qx',
                  'qy', 'qz', 'rx', 'rz', 'ry']

    quaternion_keys = valid_keys[3:7]
    euler_keys = valid_keys[7:]
    got_quaternion = False
    got_euler = False

    for key in state.keys():
        if array_find(valid_keys, key) == -1:
            raise ValueError("Invalid key.")
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


class TransformationParameters():

    def __init__(self, values: Dict[str, float], state_variables: List[str] = []):
        if not set(state_variables) <= set(values.keys()):
            raise ValueError(
                "Key(s) from stateVariables not present inside values")

        constants = {}
        state = {}

        self.convention = validate_keys_and_get_convention(values)

        for key in values.keys():
            if array_find(state_variables, key) != -1:
                state.setdefault(key, values.get(key))
            else:
                constants.setdefault(key, values.get(key))

        self.state = state
        self.constants = constants


def make_homogenious_transformation_matrix(para: TransformationParameters):
    if para.convention == 'euler':
        rx = 0
        ry = 0
        rz = 0
    if para.convention == 'quaternion':
        qw = 0
        qx = 0
        qy = 0
        qz = 0

    tx = 0
    ty = 0
    tz = 0

    for key in para.constants.keys():
        if key == 'rx':
            rx = para.constants.get(key)
        if key == 'ry':
            ry = para.constants.get(key)
        if key == 'rz':
            rz = para.constants.get(key)
        if key == 'qw':
            qw = para.constants.get(key)
        if key == 'qx':
            qx = para.constants.get(key)
        if key == 'qy':
            qy = para.constants.get(key)
        if key == 'qz':
            qz = para.constants.get(key)
        if key == 'tx':
            tx = para.constants.get(key)
        if key == 'ty':
            ty = para.constants.get(key)
        if key == 'tz':
            tz = para.constants.get(key)

    for key in para.state.keys():
        if key == 'rx':
            rx = para.state.get(key)
        if key == 'ry':
            ry = para.state.get(key)
        if key == 'rz':
            rz = para.state.get(key)
        if key == 'qw':
            qw = para.state.get(key)
        if key == 'qx':
            qx = para.state.get(key)
        if key == 'qy':
            qy = para.state.get(key)
        if key == 'qz':
            qz = para.state.get(key)
        if key == 'tx':
            tx = para.state.get(key)
        if key == 'ty':
            ty = para.state.get(key)
        if key == 'tz':
            tz = para.state.get(key)
    if para.convention == 'euler':
        return HomogenousTransformationMatrix(rx=rx, ry=ry, rz=rz, conv='xyz', tx=tx, ty=ty, tz=tz)
    if para.convention == 'quaternion':
        return HomogenousTransformationMatrix(qw=qw, qx=qx, qy=qy, qz=qz, conv='quat', tx=tx, ty=ty, tz=tz)
    raise RuntimeError("No Convention.")


class KinematicGroup():

    def __init__(self, virtual_transformations: List[TransformationParameters], actuated_state: Dict[str, float] = None, f_mapping: Callable = None, g_mapping: Callable = None, parent=None):

        # Adds itself as child to parent
        if parent != None:
            self.__parent = parent
            parent.__add_child(self)

        self.__virtual_transformations = deepcopy(virtual_transformations)

        # if there are actuated states a f_mapping and a g_mapping are nessesary

        if not ((actuated_state and f_mapping and g_mapping) or not (actuated_state and f_mapping and g_mapping)):
            raise ValueError(
                "For actuated states a forward and an inverse mapping are required.")

        if actuated_state:
            self.__actuated_state = deepcopy(actuated_state)
        else:
            self.__actuated_state = None
        # check if states are valid

        virtual_state = []
        for spec in virtual_transformations:
            virtual_state.append(spec.state)

        self.__virtual_state = virtual_state

        # check mappings
        if f_mapping:
            f_mapping_to_check = f_mapping(actuated_state)

            if virtual_state_to_keys(f_mapping_to_check) != virtual_state_to_keys(virtual_state):
                raise RuntimeError("f_mapping does not fit virtual state")

            self.__f_mapping = f_mapping

            g_mapping_to_check = g_mapping(virtual_state)

            if g_mapping_to_check.keys() != actuated_state.keys():
                raise RuntimeError("g_mapping does not fit actuated state")

            self.__g_mapping = g_mapping

            # check if initalvalues fit f_mapping and g_mapping

            for i in range(len(f_mapping_to_check)):
                state = f_mapping_to_check[i]
                init_values_do_not_match = False
                for key in state.keys():
                    if state[key] != virtual_state[i][key]:
                        init_values_do_not_match = True
            if init_values_do_not_match:
                print("Calculated state values do not match given values! Using set_state() before forward_kinematics() or inverse_kinematics() is recommended")

    def set_state(self, state: Union[Dict[str, float], List[Dict[str, float]]]) -> None:

        # forward
        if isinstance(state, dict):
            if self.__actuated_state.keys() == state.keys():
                self.__actuated_state = deepcopy(state)
                self.__virtual_state = self.__f_mapping(self.__actuated_state)
                self.__update_chain()

        # inverse
        elif isinstance(state, list):
            if virtual_state_to_keys(self.__virtual_state) == virtual_state_to_keys(state):
                self.__virtual_state = deepcopy(state)
                self.__update_chain()
                if self.__actuated_state != None:   # for the trivialcase there is no actuated state
                    self.__actuated_state = self.__g_mapping(
                        self.__virtual_state)

        else:
            raise ValueError("State does not match!")

    def get_virtual_state(self) -> List[Dict[str, float]]:
        out = []
        for trans_para in self.__virtual_transformations:
            out.append(deepcopy(trans_para.state))
        return out

    def get_actuated_state(self):
        if self.__actuated_state:
            return deepcopy(self.__actuated_state)
        else:
            return None

    def get_transformation(self) -> HomogenousTransformationMatrix:

        # Identity matrix
        transformation = HomogenousTransformationMatrix()
        for part in self.__virtual_transformations:
            hmt = make_homogenious_transformation_matrix(part)
            transformation = transformation * hmt

        return transformation

    def get_virtual_transformations(self):
        return deepcopy(self.__virtual_transformations)

    def __update_chain(self):
        for i in range(len(self.__virtual_state)):
            self.__virtual_transformations[i].state = self.__virtual_state[i]

    def __add_child(self, child):
        self.__child = child
