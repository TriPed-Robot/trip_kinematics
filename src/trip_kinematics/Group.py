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


class Group:
    def __init__(self):

        pass
