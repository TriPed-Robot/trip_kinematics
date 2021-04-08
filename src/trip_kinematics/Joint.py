
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix
from typing import Dict, List, Union
from trip_kinematics.Group import ParameterSpecification, Group


from typing import Dict, List, Union


class JointState:
    """Representation of the state of a single joint

    Attributes:
        q0 (Union(None,float)): Rotation represented as quaterions
        q1 (Union(None,float)): Rotation represented as quaterions
        q2 (Union(None,float)): Rotation represented as quaterions
        q3 (Union(None,float)): Rotation represented as quaterions
        x (Union(None,float)): Translation in x direction 
        y (Union(None,float)): Translation in y direction
        z (Union(None,float)): Translation in z direction
    """

    def __init__(self, values: Dict[str, float]) -> None:
        """Checks if the given keys of the values match the standard keys

        Args:
            values (Dict[str, float]): Dictionary of the state variables

        Raises:
            ValueError: Raises error if value keys do not match the standard keys
        """

        self.__q0 = None
        self.__q1 = None
        self.__q2 = None
        self.__q3 = None
        self.__x = None
        self.__y = None
        self.__z = None

        for key in values.keys():
            if 'x#y#z#q0#q1#q2#q3'.find(key) == -1:
                raise ValueError("Invalid key.")

            if key == 'q0':
                self.__q0 = values.get(key)
            if key == 'q1':
                self.__q1 = values.get(key)
            if key == 'q2':
                self.__q2 = values.get(key)
            if key == 'q3':
                self.__q3 = values.get(key)
            if key == 'x':
                self.__x = values.get(key)
            if key == 'y':
                self.__y = values.get(key)
            if key == 'z':
                self.__z = values.get(key)

    def get_state(self) -> Dict[str, float]:
        out = dict()
        if self.__q0 != None:
            out.setdefault('q0', self.__q0)
        if self.__q1 != None:
            out.setdefault('q1', self.__q1)
        if self.__q2 != None:
            out.setdefault('q1', self.__q2)
        if self.__q3 != None:
            out.setdefault('q1', self.__q3)
        if self.__x != None:
            out.setdefault('q1', self.__x)
        if self.__y != None:
            out.setdefault('q1', self.__y)
        if self.__z != None:
            out.setdefault('q1', self.__z)
        return out


class Joint(Group):
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
        super().__init__(values, adjustable)
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
