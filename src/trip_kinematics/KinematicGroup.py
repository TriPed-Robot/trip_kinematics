from typing import Dict, List, Callable
from trip_kinematics.KinematicChainPart import KinematicChainPart
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix
from copy import deepcopy
from trip_kinematics.Utils import sort_kinematic_chain_parts


def virtual_state_to_keys(virtual_state):
    return list(map(lambda obj: obj.keys(), virtual_state))


class KinematicGroup(KinematicChainPart):

    def __init__(self, name: str, open_chain: List[KinematicChainPart], initial_state: Dict[str, float], f_mapping: Callable, g_mapping: Callable, parent: KinematicChainPart = None) -> None:
        super().__init__(name, parent)
        self.__open_chain = open_chain
        self.__state = deepcopy(initial_state)
        self.virtual_state = []

        sorted_open_chain = sort_kinematic_chain_parts(open_chain)

        for part in sorted_open_chain:
            self.virtual_state.append(part.get_state())

        self.__open_chain = sorted_open_chain

        self.__f_mapping = f_mapping
        self.__g_mapping = g_mapping

        state_to_check = self.__f_mapping(initial_state)

        if virtual_state_to_keys(state_to_check) != virtual_state_to_keys(self.virtual_state):
            raise RuntimeError("f_mapping doesn't fit virtual state")

        self.virtual_state = state_to_check

    def set_state(self, dir, state: Dict[str, float]) -> None:

        if self.__state.keys() == state.keys():
            self.__state = deepcopy(state)
            self.__virtual_state = self.__f_mapping(self.__state)

        elif virtual_state_to_keys(self.__virtual_state) == virtual_state_to_keys(state):
            self.__virtual_state = deepcopy(state)
            self.__state = self.__g_mapping(self.__virtual_state)
        else:
            raise ValueError("State does not match!")

    def get_state(self) -> List[Dict[str, float]]:
        return deepcopy(self.__virtual_state)

    def get_transformation(self) -> Homogenous_transformation_matrix:
        transformation = Homogenous_transformation_matrix()
        for part in self.__open_chain:
            transformation = transformation * part.get_transformation()
        return transformation
