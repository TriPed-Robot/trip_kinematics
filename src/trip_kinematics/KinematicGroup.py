from typing import Dict, List, Callable, Union
from trip_kinematics.HomogenTransformationMatrix import TransformationMatrix
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

    def get_name(self):
        return deepcopy(self.__name)

    def get_transformation_matrix(self):
        if self.convention == 'euler':
            rx = 0
            ry = 0
            rz = 0
        if self.convention == 'quaternion':
            qw = 0
            qx = 0
            qy = 0
            qz = 0

        tx = 0
        ty = 0
        tz = 0

        for key in self.constants.keys():
            if key == 'rx':
                rx = self.constants.get(key)
            if key == 'ry':
                ry = self.constants.get(key)
            if key == 'rz':
                rz = self.constants.get(key)
            if key == 'qw':
                qw = self.constants.get(key)
            if key == 'qx':
                qx = self.constants.get(key)
            if key == 'qy':
                qy = self.constants.get(key)
            if key == 'qz':
                qz = self.constants.get(key)
            if key == 'tx':
                tx = self.constants.get(key)
            if key == 'ty':
                ty = self.constants.get(key)
            if key == 'tz':
                tz = self.constants.get(key)

        for key in self.state.keys():
            if key == 'rx':
                rx = self.state.get(key)
            if key == 'ry':
                ry = self.state.get(key)
            if key == 'rz':
                rz = self.state.get(key)
            if key == 'qw':
                qw = self.state.get(key)
            if key == 'qx':
                qx = self.state.get(key)
            if key == 'qy':
                qy = self.state.get(key)
            if key == 'qz':
                qz = self.state.get(key)
            if key == 'tx':
                tx = self.state.get(key)
            if key == 'ty':
                ty = self.state.get(key)
            if key == 'tz':
                tz = self.state.get(key)
        if self.convention == 'euler':
            return TransformationMatrix(rx=rx, ry=ry, rz=rz, conv='xyz', tx=tx, ty=ty, tz=tz)
        if self.convention == 'quaternion':
            return TransformationMatrix(qw=qw, qx=qx, qy=qy, qz=qz, conv='quat', tx=tx, ty=ty, tz=tz)
        raise RuntimeError("No Convention.")


class KinematicGroup():

    @staticmethod
    def object_list_to_key_lists(object_lst):
        return list(map(lambda obj: list(obj.keys()), object_lst))

    def __init__(self, name: str, virtual_transformations: List[Transformation], actuated_state: List[Dict[str, float]] = None, actuated_to_virtual: Callable = None, virtual_to_actuated: Callable = None, f_args=None, g_args=None, parent=None):

        self.__name = name

        # Adds itself as child to parent
        if parent != None:
            self.__parent = parent
            parent.__add_child(self)

        self.__virtual_transformations = deepcopy(virtual_transformations)

        # if there are actuated states a actuated_to_virtual and a virtual_to_actuated are nessesary
        if (actuated_to_virtual or virtual_to_actuated) and not actuated_state:
            raise ValueError(
                'Error: Actuated state is missing. You provided a mapping to actuate the group but no state to be actuated.')
        if (actuated_to_virtual and not virtual_to_actuated) or (not actuated_to_virtual and virtual_to_actuated):
            raise ValueError(
                'Error: Only one mapping provided. You need mappings for both ways. Consider to pass a trivial mapping.')
        if actuated_state and not (actuated_to_virtual or virtual_to_actuated):
            raise ValueError(
                'Error: Mappings missing. You provided an actuated state but no mappings. If you want a trivial mapping you don\'t need to pass an actuated state. Trip will generate one for you.')

        virtual_state = []
        for transformation in virtual_transformations:
            virtual_state.append(transformation.state)

        self.__virtual_state = virtual_state

        if actuated_state:      # if there is an actuated state there has to be an actuated_to_virtual and an virtual_to_actuated
            self.__actuated_state = deepcopy(actuated_state)
            self.__original_actuated_to_virtual = actuated_to_virtual
            if f_args:
                self.__actuated_to_virtual = lambda state: actuated_to_virtual(state, *f_args)
            self.__actuated_to_virtual = actuated_to_virtual

            actuated_to_virtual_to_check = self.__actuated_to_virtual(actuated_state)

            if KinematicGroup.object_list_to_key_lists(actuated_to_virtual_to_check) != KinematicGroup.object_list_to_key_lists(virtual_state):
                raise RuntimeError("actuated_to_virtual does not fit virtual state")

            self.___original_virtual_to_actuated = virtual_to_actuated
            if g_args:
                self.__virtual_to_actuated = lambda state: virtual_to_actuated(state, *g_args)
            self.__virtual_to_actuated = virtual_to_actuated

            virtual_to_actuated_to_check = virtual_to_actuated(virtual_state)

            if KinematicGroup.object_list_to_key_lists(virtual_to_actuated_to_check) != KinematicGroup.object_list_to_key_lists(actuated_state):
                raise RuntimeError("virtual_to_actuated does not fit actuated state")

            # Check if inital values fit actuated_to_virtual's and virtual_to_actuated's calculated values. Only if actuated_state, actuated_to_virtual and virtual_to_actuated are passed

            for i in range(len(actuated_to_virtual_to_check)):
                state = actuated_to_virtual_to_check[i]
                init_values_do_not_match = False
                for key in state.keys():
                    if state[key] != virtual_state[i][key]:
                        init_values_do_not_match = True
            if init_values_do_not_match:
                print("Calculated state values do not match given values! Using set_state() before forward_kinematics() or inverse_kinematics() is recommended.")
        else:
            # Virtual state contains keys
            if any(virtual_state):
                # trivial mappings
                actuated_state_dummy = {}
                f_map = {}
                g_map = {}

                for index, transformation in enumerate(virtual_transformations):

                    state = transformation.state

                    for key, value in state.items():
                        concat_key = transformation.get_name() + '_' + key
                        actuated_state_dummy.setdefault(concat_key, value)

                        f_map.setdefault(concat_key, (key, index))
                        g_map.setdefault((key, index), concat_key)

                    # create trivial mappings
                def trivial_actuated_to_virtual(state):
                    # Generate array
                    out = [dict() for x in range(len(virtual_transformations))]

                    for concat_key, value in state[0].items():
                        key, index = f_map[concat_key]
                        out[index].setdefault(key, value)

                    return out

                def trivial_virtual_to_actuated(states):
                    out = [{}]
                    for index, state in enumerate(states):
                        for key, value in state.items():
                            combined_key = (key, index)
                            key = g_map[combined_key]
                            out[0].setdefault(key, value)

                    return out

                self.__actuated_to_virtual = trivial_actuated_to_virtual
                self.__virtual_to_actuated = trivial_virtual_to_actuated
                self.__actuated_state = [actuated_state_dummy]

            else:   # This is a static group
                self.__actuated_state = None
                self.__actuated_to_virtual = None
                self.__virtual_to_actuated = None

    def set_virtual_state(self, state: List[Dict[str, float]]):

        if self.__actuated_state == None:
            raise RuntimeError(
                "This is a static group! There is no state to be set")

        if KinematicGroup.object_list_to_key_lists(self.__virtual_state) == KinematicGroup.object_list_to_key_lists(state):
            self.__virtual_state = deepcopy(state)
            self.__update_chain()
            self.__actuated_state = self.__virtual_to_actuated(self.__virtual_state)
        else:
            raise ValueError(
                "Error: State not set! Keys do not match! Make sure that your state includes the same keys as your intial virtual transformations.")

    def set_actuated_state(self, state: List[Dict[str, float]]):

        if self.__actuated_state == None:
            raise RuntimeError(
                "This is a static group! There is no state to be set.")

        if KinematicGroup.object_list_to_key_lists(self.__actuated_state) == KinematicGroup.object_list_to_key_lists(state):
            self.__actuated_state = deepcopy(state)
            self.__virtual_state = self.__actuated_to_virtual(self.__actuated_state)
            self.__update_chain()
        else:
            raise ValueError(
                "Error: State not set! Keys do not match! Make sure that your state includes the same keys as the intial actuated state.")

    def __str__(self):
        return self.__name

    def get_name(self):
        return deepcopy(self.__name)

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

    def get_transformation_matrix(self) -> TransformationMatrix:

        # Identity matrix
        transformation = TransformationMatrix()
        for part in self.__virtual_transformations:
            hmt = part.get_transformation_matrix()
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
        g_map = self.___original_virtual_to_actuated
        self.__virtual_to_actuated = lambda state: g_map(state, *argv)

    def pass_arguments_f(self, argv):
        f_map = self.__original_actuated_to_virtual
        self.__actuated_to_virtual = lambda state: f_map(state, *argv)
