from typing import Dict, List, Callable, Union
from trip_kinematics.HomogenTransformationMatrix import HomogenousTransformationMatrix
from copy import deepcopy


def array_find(arr, obj) -> int:
    index = -1
    try:
        index = arr.index(obj)
        return index
    except:
        return -1


class Transformation():

    @staticmethod
    def get_convention(state: Dict[str, float]):

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

    def __init__(self, name: str, values: Dict[str, float], state_variables: List[str] = []):

        self.__name = name

        if not set(state_variables) <= set(values.keys()):
            raise ValueError(
                "Key(s) from stateVariables not present inside values")

        constants = {}
        state = {}

        self.convention = Transformation.get_convention(values)

        for key in values.keys():
            if array_find(state_variables, key) != -1:
                state.setdefault(key, values.get(key))
            else:
                constants.setdefault(key, values.get(key))

        self.state = state
        self.constants = constants

    def __str__(self):
        return self.__name


def make_homogenous_transformation_matrix(para: Transformation):
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

    @staticmethod
    def object_list_to_key_lists(object_lst):
        return list(map(lambda obj: list(obj.keys()), object_lst))

    def __init__(self, name: str, virtual_transformations: List[Transformation], actuated_state: List[Dict[str, float]] = None, f_mapping: Callable = None, g_mapping: Callable = None, f_args=None, g_args=None, parent=None):

        self.__name = name

        # Adds itself as child to parent
        if parent != None:
            self.__parent = parent
            parent.__add_child(self)

        self.__virtual_transformations = deepcopy(virtual_transformations)

        # if there are actuated states a f_mapping and a g_mapping are nessesary
        if (f_mapping or g_mapping) and not actuated_state:
            raise ValueError(
                'Error: Actuated state is missing. You provided a mapping to actuate the group but no state to be actuated.')
        if (f_mapping and not g_mapping) or (not f_mapping and g_mapping):
            raise ValueError(
                'Error: Only one mapping provided. You need mappings for both ways. Consider to pass a trival mapping.')

        if actuated_state:
            self.__actuated_state = deepcopy(actuated_state)
        else:
            self.__actuated_state = None
        # check if states are valid

        if not f_mapping:
            # trivial mapping
            pass
        if not g_mapping:
            # trivial mapping
            pass

        virtual_state = []
        for transformation in virtual_transformations:
            virtual_state.append(transformation.state)

        self.__virtual_state = virtual_state

        # check mappings
        if f_mapping:
            self.___original_f_mapping = f_mapping
            if f_args:
                self.__f_mapping = lambda state: f_mapping(state, *f_args)
            self.__f_mapping = f_mapping

            f_mapping_to_check = self.__f_mapping(actuated_state)

            if KinematicGroup.object_list_to_key_lists(f_mapping_to_check) != KinematicGroup.object_list_to_key_lists(virtual_state):
                raise RuntimeError("f_mapping does not fit virtual state")

            self.___original_g_mapping = g_mapping
            if g_args:
                self.__g_mapping = lambda state: g_mapping(state, *g_args)
            self.__g_mapping = g_mapping

            g_mapping_to_check = g_mapping(virtual_state)

            if KinematicGroup.object_list_to_key_lists(g_mapping_to_check) != KinematicGroup.object_list_to_key_lists(actuated_state):
                raise RuntimeError("g_mapping does not fit actuated state")

            # check if initalvalues fit f_mapping and g_mapping

            for i in range(len(f_mapping_to_check)):
                state = f_mapping_to_check[i]
                init_values_do_not_match = False
                for key in state.keys():
                    if state[key] != virtual_state[i][key]:
                        init_values_do_not_match = True
            if init_values_do_not_match:
                print("Calculated state values do not match given values! Using set_state() before forward_kinematics() or inverse_kinematics() is recommended.")

    def set_virtual_state(self, state: List[Dict[str, float]]):

        if KinematicGroup.object_list_to_key_lists(self.__virtual_state) == KinematicGroup.object_list_to_key_lists(state):
            self.__virtual_state = deepcopy(state)
            self.__update_chain()
            self.__actuated_state = self.__g_mapping(self.__virtual_state)
        else:
            print("Error: State not set! Keys do not match! Make sure that your state includes the same keys as your intial virtual transformations.")

    def set_actuated_state(self, state: List[Dict[str, float]]):

        if KinematicGroup.object_list_to_key_lists(self.__actuated_state) == KinematicGroup.object_list_to_key_lists(state):
            self.__actuated_state = deepcopy(state)
            self.__virtual_state = self.__f_mapping(self.__actuated_state)
            self.__update_chain()
        else:
            print("Error: State not set! Keys do not match! Make sure that your state includes the same keys as the intial actuated state.")

    def __str__(self):
        return self.__name

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
            hmt = make_homogenous_transformation_matrix(
                part)
            transformation = transformation * hmt

        return transformation

    def get_virtual_transformations(self):
        return deepcopy(self.__virtual_transformations)

    def __update_chain(self):
        for i in range(len(self.__virtual_state)):
            self.__virtual_transformations[i].state = self.__virtual_state[i]

    def __add_child(self, child):
        self.__child = child

    def pass_arguments_g(self, argv):
        g_map = self.___original_g_mapping
        self.__g_mapping = lambda state: g_map(state, *argv)

    def pass_arguments_f(self, argv):
        f_map = self.___original_f_mapping
        self.__f_mapping = lambda state: f_map(state, *argv)
