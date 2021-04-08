from trip_kinematics.KinematicChainPart import KinematicChainPart
from typing import Union, List


class Robot:
    """[summary]

    Returns:
        [type]: [description]
    """    """[summary]
    """

    def __init__(self, kinematic_chain: List[KinematicChainPart]) -> None:
        """[summary]

        Args:
            kinematic_chain (List[KinematicChainPart]): [description]
        """
        self.__kinematic_chain = kinematic_chain

    def get_parts(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        return self.__kinematic_chain
