from typing import Dict, List, Union
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix


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
