
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix
from typing import Union


class Constants:
    """[summary]
    """

    def __init__(self, x=0, y=0, z=0, alpha=0, beta=0, gamma=0) -> None:
        """[summary]

        Args:
            x (int, optional): [description]. Defaults to 0.
            y (int, optional): [description]. Defaults to 0.
            z (int, optional): [description]. Defaults to 0.
            alpha (int, optional): [description]. Defaults to 0.
            beta (int, optional): [description]. Defaults to 0.
            gamma (int, optional): [description]. Defaults to 0.
        """
        self.x: Union[float, object] = x
        self.y: Union[float, object] = y
        self.z: Union[float, object] = z
        self.alpha: Union[float, object] = alpha
        self.beta: Union[float, object] = beta
        self.gamma: Union[float, object] = gamma


class State:
    """[summary]
    """

    def __init__(self, x=None, y=None, z=None, alpha=None, beta=None, gamma=None) -> None:
        self.state = dict()

        if(x):
            self.state.update({"x": x})
        if(y):
            self.state.update({"y": y})
        if(z):
            self.state.update({"z": z})
        if(alpha):
            self.state.update({"alpha": alpha})
        if(beta):
            self.state.update({"beta": beta})
        if(x):
            self.state.update({"gamma": gamma})


class Joint:
    """[summary]
    """

    def __init__(self, name: str, constants: Constants, state: State, parents: [] = []) -> None:
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
        self.__constants: Constants = constants
        self.__state = state

    def get_transformation(self) -> Homogenous_transformation_matrix:
        """[summary]

        Returns:
            Homogenous_transformation_matrix: [description]
        """
        tx = self.__constants.x
        ty = self.__constants.y
        tz = self.__constants.z

        rx = self.__constants.alpha
        ry = self.__constants.beta
        rz = self.__constants.gamma

        return Homogenous_transformation_matrix(tx=tx, ty=ty, tz=tz, rx=rx, ry=ry, rz=rz, conv='xyz')

    def change_value(self, state: State) -> None:
        """[summary]

        Args:
            state (State): [description]
        """
        self.__state = State

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
