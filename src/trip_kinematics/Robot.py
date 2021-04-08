from trip_kinematics.KinematicChainPart import KinematicChainPart
from typing import Union, List
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix


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


def forward_kinematic(robot: Robot):
    transformation = Homogenous_transformation_matrix()
    for part in robot.get_parts():
        transformation * part
    return transformation.get_translation()
