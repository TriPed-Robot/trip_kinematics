from typing import Dict, List, Callable


class ParameterSpecification:
    """Used for initilizing a KinematicObject

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

        self.constants = {}
        self.variables = {}

        adjust = '#'.join(adjustable)

        for key in values.keys():
            if adjust.find(key) != -1:
                self.variables.setdefault(key, values.get(key))
            else:
                self.constants.setdefault(key, values.get(key))


class Group:
    """[summary]
    """

    def __init__(self, name: str, params: ParameterSpecification, set_state: Callable, get_state: Callable, parents: List = []) -> None:
        """[summary]

        Args:
            name (str): [description]
            state (State): [description]
            set_state (Callable): [description]
            get_state (Callable): [description]
            parents (List[Group], optional): [description]. Defaults to [].
        """        """[summary]

        Args:
            name (str): [description]
            state (State): [description]
            parents (List[Group], optional): [description]. Defaults to [].
        """
        self.__name: str = name
        self.__parents: List[Group] = parents
        if len(parents) != 0:
            for parent in parents:
                parent.__add_child(self)

        self.__children: List[Group] = []
        self.__state: ParameterSpecification = params
        self.set_state: Callable = set_state
        self.get_state: Callable = get_state

    def get_name(self) -> str:
        """[summary]

        Returns:
            str: [description]
        """
        return self.__name

    def __add_child(self, child) -> None:
        """[summary]

        Args:
            child (Group): [description]
        """
        self.__children.append(child)
