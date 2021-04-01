
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix
from typing import Union
from typing import Dict, List


class State:
    """Representation of the state of a group of kinematic joints

    Attributes:
        __constants (Dict[str, float]): All the constants values inside the group of joints
        variables (Dict[str, float]): All the variables (DOF) of a group of joints
    """

    def __init__(self, values: Dict[str, float], adjustable: List[str]) -> None:
        """Sorts values into either constants or variables

        Args:
            values (Dict[str, float]): Dictionary of all values both constants and variables
            adjustable (List[str]): List of all the variables inside values

        Raises:
            ValueError: An error is raised if there is a key in adjustable that doesn't match any from values
        """
        if not set(adjustable) <= set(values.keys()):
            raise ValueError(
                "Key(s) from adjustable not present inside values")

        self.__constants = {}
        self.variables = {}

        adjust = '#'.join(adjustable)

        for key in values.keys():
            if adjust.find(key) != -1:
                self.variables.setdefault(key, values.get(key))
            else:
                self.__constants.setdefault(key, values.get(key))

    def get_constants(self) -> Dict[str, float]:
        """Getter for constants

        Returns:
            Dict[str, float]: All the constants inside a dict
        """
        return self.__constants


class JointState(State):
    """Representation of the state of a single joint

    Attributes:
        __constants (Dict[str, float]): All the constant values of a joint
        variables (Dict[str, float]): All the variables (DOF) of a joint
    """

    def __init__(self, values: Dict[str, float], adjustable: List[str]) -> None:
        """Checks if the given keys of the values match the standard keys

        Args:
            values (Dict[str, float]): Dictionary of all values both constants and variables
            adjustable (List[str]): List of all the variables inside values

        Raises:
            ValueError: Raises error if value keys do not match the standard keys
        """
        possible_keys = 'x#y#z#alpha#beta#gamma'
        for key in values.keys():
            if possible_keys.find(key) == -1:
                raise ValueError("Invalid key.")

        super().__init__(values, adjustable)


class Joint:
    """[summary]
    """

    def __init__(self, name: str, state: JointState, parents: [] = []) -> None:
        """[summary]

        Args:
            name (str): [description]
            constants (Constants): [description]
            state (State): [description]
            parents ([type], optional): [description]. Defaults to [].
        """
        self.__name: str = name
        self.__parents: [] = parents
        if len(parents) != 0:
            for parent in parents:
                parent.__add_child(self)

        self.__children: [] = []
        self.__state = state

    def get_transformation(self) -> Homogenous_transformation_matrix:
        """[summary]

        Returns:
            Homogenous_transformation_matrix: [description]
        """
        pass

    def change_value(self, state: JointState) -> None:
        """[summary]

        Args:
            state (State): [description]
        """
        self.__state = state

    def get_name(self) -> str:
        """Returns name of joint

        Returns:
            str: [Name of this joint]
        """
        return self.__name

    def __add_child(self, child) -> None:
        """Add a child joint

        Args:
            child (Joint): [Joint that is connected to this joint]
        """
        self.__children.append(child)
