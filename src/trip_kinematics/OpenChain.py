from trip_kinematics.Joint import Joint
from typing import Union


def dummy_function():
    pass


class OpenChain:
    """[summary]
    """

    def __init__(self, joints, mapping) -> None:
        """[summary]

        Args:
            joints ([type]): [description]
            mapping ([type]): [description]
        """
        self.__joints = []
        self.__mapping = {'test': dummy_function}
        pass

    def forward(self):
        """[summary]
        """
        pass

    def inverse(self):
        """[summary]
        """
        pass

    def get_actuated_joints(self):
        """[summary]
        """
        pass

    def get_joints(self):
        """[summary]
        """
        pass
