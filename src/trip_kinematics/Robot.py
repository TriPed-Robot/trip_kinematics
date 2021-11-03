from typing import Dict, List
from copy import deepcopy
from casadi import SX

from trip_kinematics.Utility import identity_transformation
from trip_kinematics.KinematicGroup import KinematicGroup, OpenKinematicGroup
from trip_kinematics.Transformation import Transformation


class Robot:
    """A class representing the kinematic model of a robot.

    Args:
        kinematic_chain (List[KinematicGroup]): A list of Kinematic Groups and Transformations
                                                which make up the robot.
                                                Transformations are automatically
                                                converted to groups

    Raises:
        KeyError: "More than one robot actuator has the same name!
                   Please give each actuator a unique name"
                  if there are actuated states with the same names between the
                  :py:class`KinematicGroup` objects of the :py:class`Robot`
        KeyError: if there are joints with the same names between
                  the :py:class`KinematicGroup` objects of the :py:class`Robot`
    """

    def __init__(self, kinematic_chain: List[KinematicGroup]) -> None:

        self._group_dict = {}
        self._actuator_group_mapping = {}
        self._virtual_group_mapping = {}
        for i, group in enumerate(kinematic_chain):
            if isinstance(group, Transformation):
                # create group from trafo, lifting the parent and child relations to group level
                group_children = group.children
                group_parent = group.parent

                # KinematicGroup expects endeffector of virtual children to have no children
                group.children = []
                # KinematicGroup expects root of virtual children to have itself as a parent
                group.parent = str(group)

                group = OpenKinematicGroup(name=str(group), virtual_chain=[group],
                                           parent=self._group_dict[str(kinematic_chain[i-1])])

                # carefull this circumvents the usual checks for correct group and parent names!
                group.children = group_children
                group.parent = group_parent

            self._group_dict[str(group)] = group
            if group.get_virtual_state() != {}:
                group_actuators = group.get_actuated_state().keys()
                for key in group_actuators:
                    if key in self._actuator_group_mapping.keys():
                        raise KeyError(
                            "More than one robot actuator has the same name!" +
                            " Please give each actuator a unique name")
                    self._actuator_group_mapping[key] = str(group)

                for key in group.get_virtual_state().keys():
                    if key in self._virtual_group_mapping.keys():
                        raise KeyError(
                            "More than one transformation of a virtual chain has the same name!" +
                            " Please give each virtual transformation of a robot a unique name")
                    self._virtual_group_mapping[key] = str(group)

    def get_groups(self):
        """Returns a dictionary of the py:class`KinematicGroup` managed by the :py:class`Robot`_
           Since Transformations are internally converted to Groups, this also returns all
           Transformations.

        Returns:
            Dict[str, KinematicGroup]: The dictionary of py:class`KinematicGroup` objects.
        """
        return deepcopy(self._group_dict)

    def pass_group_arg_v_to_a(self, argv_dict: Dict):
        """Passes optional virtual_to_actuated mapping arguments
           to :py:class`KinematicGroup` objects of the robot.

        Args:
            argv_dict (Dict): A dictionary containing the mapping arguments keyed with the
                              :py:class`KinematicGroup` names.

        Raises:
            KeyError: If no group with the name given in the argument is part of the robot.
        """
        for key in argv_dict.keys():
            if key not in self._group_dict.keys():
                raise KeyError("No group with name "+str(key)+"in this robot")
            self._group_dict[key].pass_arg_v_to_a(argv_dict[key])

    def pass_group_arg_a_to_v(self, argv_dict):
        """Passes optional actuated_to_virtual mapping arguments
           to :py:class`KinematicGroup` objects of the robot.

        Args:
            argv_dict (Dict): A dictionary containing the mapping arguments keyed with the
                              :py:class`KinematicGroup` names.

        Raises:
            KeyError: If no group with the name given in the argument is part of the robot.
        """
        for key in argv_dict.keys():
            if key not in self._group_dict.keys():
                raise KeyError("No group with name "+str(key)+"in this robot")
            self._group_dict[key].pass_arg_a_to_v(argv_dict[key])

    def set_virtual_state(self, state: Dict[str, Dict[str, float]]):
        """Sets the virtual state of multiple virtual joints of the robot.

        Args:
            state (Dict[str,Dict[str, float]]): A dictionary containing the members of
                                                 :py:attr:`__virtual_state` that should be set.
                                                The new values need to be valid state
                                                for the state of the joint.
        """
        for key in state.keys():
            virtual_state = {key: state[key]}
            self._group_dict[self._virtual_group_mapping[key]
                             ].set_virtual_state(virtual_state)

    def set_actuated_state(self, state: Dict[str, float]):
        """Sets the virtual state of multiple actuated joints of the robot.

        Args:
            state (Dict[str, float]):  A dictionary containing the members of
                                        :py:attr:`__actuated_state` that should be set.
        """
        grouping = {}
        for key in state.keys():
            if self._actuator_group_mapping[key] not in grouping.keys():
                grouping[self._actuator_group_mapping[key]] = {}
            grouping[self._actuator_group_mapping[key]][key] = state[key]
        for key, group_state in grouping.items():
            self._group_dict[key].set_actuated_state(group_state)

    def get_actuated_state(self):
        """Returns the actuated state of the :py:class`Robot` comprised
           of the actuated states of the individual :py:class`KinematicGroup`.

        Returns:
            Dict[str, float]: combined actuated state of all :py:class`KinematicGroup` objects.
        """
        actuated_state = {}
        for group in self._group_dict.values():
            actuated_group_state = group.get_actuated_state()
            if actuated_group_state is not None:
                for actuated_key in actuated_group_state:
                    actuated_state[actuated_key] = actuated_group_state[actuated_key]
        return actuated_state

    def get_virtual_state(self):
        """Returns the virtual state of the :py:class`Robot` comprised
           of the virtual states of the individual :py:class`KinematicGroup`.

        Returns:
            Dict[str,Dict[str, float]]: combined virtual state of all
                                         :py:class`KinematicGroup` objects.
        """
        virtual_state = {}
        for group_key in self._group_dict.keys():
            group_state = self._group_dict[group_key].get_virtual_state()
            if group_state != {}:
                for key in group_state.keys():
                    virtual_state[key] = group_state[key]
        return virtual_state

    def get_symbolic_rep(self, endeffector: str):
        """This Function returnes a symbolic representation of the virtual chain.

        Args:
            endeffector (str): The name of the group whose virtual chain
                               models the desired endeffector

        Raises:
            KeyError: If the endeffector argument is not the name of a transformation or group

        Returns:
            SX: A 4x4 symbolic casadi matrix containing the transformation from base to endeffector
        """

        matrix = identity_transformation()

        symbolic_state = []
        symbolic_keys = []

        group_dict = self.get_groups()
        if endeffector not in group_dict.keys():
            raise KeyError(
                "The endeffector must be a valid group or transformation name." +
                " Valid names for this robot are: "+str(group_dict.keys()))

        endeff_group = group_dict[endeffector]
        current_parent = endeff_group.parent
        current_key = endeffector
        group_key_list = [endeffector]

        while current_parent != current_key:
            next_group = group_dict[current_parent]
            current_key = current_parent
            current_parent = next_group.parent
            group_key_list.append(current_key)

        group_key_list.reverse()
        for group_key in group_key_list:
            group = group_dict[group_key]
            virtual_trafo = group.get_virtual_chain()

            for virtual_key in virtual_trafo.keys():
                virtual_transformation = virtual_trafo[virtual_key]
                state = virtual_transformation.get_state()

                if state != {}:
                    for key in state.keys():
                        state[key] = SX.sym(virtual_key+'_'+key)
                        symbolic_state.append(state[key])
                        symbolic_keys.append([virtual_key, key])
                symbolic_transformation = deepcopy(virtual_transformation)
                symbolic_transformation.set_state(state)

                hmt = symbolic_transformation.get_transformation_matrix()
                matrix = matrix @ hmt

        hom_matrix = SX.zeros(4, 4)
        for i in range(4):
            for j in range(4):
                hom_matrix[i, j] = matrix[i, j]

        return hom_matrix, symbolic_state, symbolic_keys

    def get_endeffectors(self):
        """Returns a list of possible endeffectors.
           These are the names of all :py:class:`KinematicGroup` objects.
           Since Transformations are internally converted to Groups,
           this includes the names of all Transformations.

        Returns:
            list(str): list of possible endeffectors.
        """
        return list(self.get_groups().keys())


def forward_kinematics(robot: Robot, endeffector):
    """Calculates a robots transformation from base to endeffector using its current state

    Args:
        robot (Robot): The robot for which the forward kinematics should be computed

    Returns:
        numpy.array : The Transformation from base to endeffector
    """
    transformation = identity_transformation()
    group_dict = robot.get_groups()
    if endeffector not in group_dict.keys():
        raise KeyError(
            "The endeffector must be a valid group name. Valid group names for this robot are: " +
            str(group_dict.keys()))
    endeff_group = group_dict[endeffector]
    current_parent = endeff_group.parent
    current_key = endeffector
    group_key_list = [endeffector]
    while current_parent != current_key:
        next_group = group_dict[current_parent]
        current_key = current_parent
        current_parent = next_group.parent
        group_key_list.append(current_key)

    group_key_list.reverse()
    for group_key in group_key_list:
        group = group_dict[group_key]
        hmt = group.get_transformation_matrix()
        transformation = transformation @ hmt
    return transformation
