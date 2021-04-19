from typing import Dict, List, Union
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix


def sort_kinematic_chain_parts(lst_of_parts):
    sorted_chain = []
    for part in lst_of_parts:
        if part.get_parent() == None:
            sorted_chain.append(part)

    if len(sorted_chain) > 1:
        raise RuntimeError("To many loose ends inside chain.")

    buffer = sorted_chain[0]

    while buffer.get_child() != None:
        sorted_chain.append(buffer)
        buffer = buffer.get_child()
    return sorted_chain


class KinematicChainPart:

    __name = ''
    __state = None
    __parent = None
    __child = None

    def __init__(self, name: str, parent=None):
        self.__name = name
        if parent != None:
            self.__parent = parent
            parent.__add_child(self)

    def get_name(self) -> str:
        return self.__name

    def __add_child(self, child) -> None:
        self.__child = child

    def get_parent(self):
        return self.__parent

    def get_child(self):
        return self.__child

    def set_state(self, state):
        raise NotImplementedError()

    def get_state(self):
        raise NotImplementedError()

    def get_transformation(self) -> Homogenous_transformation_matrix:
        raise NotImplementedError()
