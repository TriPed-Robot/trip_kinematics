
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix
from typing import Dict, List, Union
from trip_kinematics.KinematicChainPart import KinematicChainPart
from typing import Dict, List, Union


class KinematicObject(KinematicChainPart):
    """Representation of either a joint or a link inside a kinematic chain
    with state variables it is considert a joint without it is considert a link

    """

    def __init__(self, name: str, values: Dict[str, float], stateVariables: List[str] = [], parent: KinematicChainPart = None) -> None:
        """[summary]

        Args:
            name (str): name
            values (Dict[str, float]): The initial values of both the constants and state variables
            stateVariables (List[str], optional): List of the values that are part of the state. Defaults to [].
            parent (KinematicChainPart, optional): The parent object inside the kinematic chain. Defaults to None.

        Raises:
            ValueError: Raises if the declared state variables do not match the keys inside values
            ValueError: Raises if the values do not match x, y, z, q0, q1, q2, q3
        """
        super().__init__(name, parent)

        if not set(stateVariables) <= set(values.keys()):
            raise ValueError(
                "Key(s) from stateVariables not present inside values")

        constants = {}
        state = {}

        adjust = '#'.join(stateVariables)

        for key in values.keys():
            if 'x#y#z#q0#q1#q2#q3'.find(key) == -1:
                raise ValueError("Invalid key.")

            if adjust.find(key) != -1:
                state.setdefault(key, values.get(key))
            else:
                constants.setdefault(key, values.get(key))

        self.__state = state
        self.__constants = constants

    def set_state(self, state):
        if set(state.keys) != set(self.__state.keys()):
            raise ValueError("State does not match initilized state.")

        self.__state = state

    def get_state(self):
        return self.__state

    def get_transformation(self) -> Homogenous_transformation_matrix:
        q0 = 0
        q1 = 0
        q2 = 0
        q3 = 0
        x = 0
        y = 0
        z = 0

        for key in self.__constants.keys():
            if key == 'q0':
                q0 = self.__constants.get(key)
            if key == 'q1':
                q1 = self.__constants.get(key)
            if key == 'q2':
                q2 = self.__constants.get(key)
            if key == 'q3':
                q3 = self.__constants.get(key)
            if key == 'x':
                x = self.__constants.get(key)
            if key == 'y':
                y = self.__constants.get(key)
            if key == 'z':
                z = self.__constants.get(key)

        for key in self.__state.keys():
            if key == 'q0':
                q0 = self.__state.get(key)
            if key == 'q1':
                q1 = self.__state.get(key)
            if key == 'q2':
                q2 = self.__state.get(key)
            if key == 'q3':
                q3 = self.__state.get(key)
            if key == 'x':
                x = self.__state.get(key)
            if key == 'y':
                y = self.__state.get(key)
            if key == 'z':
                z = self.__state.get(key)

        return Homogenous_transformation_matrix(a=q0, b=q1, c=q2, d=q3, conv='quad', tx=x, ty=y, tz=z)
