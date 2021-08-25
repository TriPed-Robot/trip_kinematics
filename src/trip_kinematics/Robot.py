from typing import Dict, List, Callable, Union
from trip_kinematics.HomogenTransformationMatrix import TransformationMatrix
from casadi import Opti
from trip_kinematics.KinematicGroup import KinematicGroup



class Robot:
    """A class managing multiple :py:class`KinematicGroup` objects pable of building tree like kinematic topologies.

    Args:
        kinematic_chain (List[KinematicGroup]): A list of Kinematic Groups with make up the robot.

    Raises:
        KeyError: "More than one robot actuator has the same name! Please give each actuator a unique name" 
                  if there are actuated states with the same names between the :py:class`KinematicGroup` objects of the :py:class`Robot`
        KeyError: "More than one robot virtual transformation has the same name! Please give each virtual transformation a unique name"
                  if there are joints with the same names between the :py:class`KinematicGroup` objects of the :py:class`Robot`
    """

    def __init__(self, kinematic_chain: List[KinematicGroup]) -> None:

        self.__group_dict = {}
        self.__actuator_group_mapping = {}
        self.__virtual_group_mapping = {}

        for group in kinematic_chain:
            self.__group_dict[str(group)]=group

            group_actuators = group.get_actuated_state().keys()
            for key in group_actuators:
                if key in self.__actuator_group_mapping.keys():
                    raise KeyError("More than one robot actuator has the same name! Please give each actuator a unique name")
                self.__actuator_group_mapping[key]=str(group)

            group_virtuals = []
            for key in group.get_virtual_state().keys():
                if key in self.__virtual_group_mapping.keys():
                    raise KeyError("More than one robot virtual transformation has the same name! Please give each virtual transformation a unique name")
                self.__virtual_group_mapping[key]=str(group)



        


    def get_groups(self):
        """Returns a dictionary of the py:class`KinematicGroup` managed by the :py:class`Robot`-

        Returns:
            Dict[str, KinematicGroup]: The dictionary of py:class`KinematicGroup` objects.
        """
        return self.__group_dict

    def set_virtual_state(self, state: Dict[str,Dict[str, float]]):
        """Sets the virtual state of multiple virtual joints of the robot.

        Args:
            state (Dict[str,Dict[str, float]]): A dictionary containing the members of :py:attr:`__virtual_state` that should be set. 
                                                The new values need to be valid state for the state of the joint.
        """
        for key in state.keys():
            virtual_state = {key:state[key]}
            self.__group_dict[self.__virtual_group_mapping[key]].set_virtual_state(virtual_state)
    
    def set_actuated_state(self, state: Dict[str, float]):
        """Sets the virtual state of multiple actuated joints of the robot.

        Args:
            state (Dict[str, float]):  A dictionary containing the members of :py:attr:`__actuated_state` that should be set. 
        """
        #TODO first group them according to their group then send them as packages
        #TODO detect when grouping is incomplete!!!!!
        grouping = {}
        for key in state.keys():
            if self.__actuator_group_mapping[key] not in grouping.keys():
                grouping[self.__actuator_group_mapping[key]] = {}
            grouping[self.__actuator_group_mapping[key]][key]=state[key]
        for key in grouping.keys():
            self.__group_dict[key].set_actuated_state(grouping[key])


    def get_actuated_state(self):
        """Returns the actuated state of the :py:class`Robot` comprised of the actuated states of the individual :py:class`KinematicGroup`.

        Returns:
            Dict[str, float]: combined actuated state of all :py:class`KinematicGroup` objects.
        """
        actuated_state={}
        for key in self.__group_dict.keys():
            actuated_group = self.__group_dict[key].get_actuated_state()
            for actuated_key in actuated_group:
                actuated_state[actuated_key]=actuated_group[actuated_key]
        return actuated_state

    def get_virtual_state(self):
        """Returns the virtual state of the :py:class`Robot` comprised of the virtual states of the individual :py:class`KinematicGroup`.

        Returns:
            Dict[str,Dict[str, float]]: combined virtual state of all :py:class`KinematicGroup` objects.
        """
        virtual_state={}
        for group_key in self.__group_dict.keys():
            group_state = self.__group_dict[group_key].get_virtual_state()
            for key in group_state.keys():
                virtual_state[key]=group_state[key]
        return virtual_state


    def get_symbolic_rep(self,opti_obj,endeffector):
        """This Function returnes a symbolic representation of the virtual chain.


        Returns:
            TransformationMatrix: The :py:class:`TransformationMatrix` containing symbolic objects
        """
        matrix = TransformationMatrix()

        symbolic_state = {}

        groups = self.get_groups()
        
        for group_key in groups.keys():
            group         = groups[group_key]
            virtual_trafo = group.get_virtual_transformations()


            for virtual_key in virtual_trafo.keys():
                virtual_transformation = virtual_trafo[virtual_key]
                state = virtual_transformation.state

                
                for key in state.keys():

                    start_value = state[key]
                    state[key] = opti_obj.variable()
                    opti_obj.set_initial(state[key], start_value)

                if state != {}:
                    symbolic_state[virtual_key]=state

                hmt = virtual_transformation.get_transformation_matrix()
                matrix = matrix * hmt



        return matrix, symbolic_state

    @staticmethod
    def solver_to_virtual_state(sol,symbolic_state):
        """This Function maps the solution of a opti solver to the virtual state of the robot

        Args:
            sol ([type]): A opti solver object
            symbolic_state ([type]): the description of the symbolic state that corresponds to the solver values

        Returns:
            Dict[str,Dict[str, float]]: a :py:attr:`virtual_state` of a robot.
        """
        solved_states = {}

        for joint_key in symbolic_state.keys():
            states = {}
            virtual_joint = symbolic_state[joint_key]
            for key in virtual_joint.keys():
                states[key]=sol.value(symbolic_state[joint_key][key])

            solved_states[joint_key]=states
        
        return solved_states




def forward_kinematics(robot: Robot):
    """Calculates a robots transformation from base to endeffector using its current state

    Args:
        robot (Robot): The robot for which the forward kinematics should be computed

    Returns:
        numpy.array : The Transformation from base to endeffector 
    """
    transformation = TransformationMatrix()
    groups = robot.get_groups()
    for group_key in groups.keys():
        group = groups[group_key]
        hmt = group.get_transformation_matrix()
        transformation = transformation * hmt
    return transformation.matrix


def inverse_kinematics(robot: Robot, end_effector_position):
    """Simple Inverse kinematics algorithm that computes the actuated state necessairy for the endeffector to be at a specified position

    Args:
        robot (Robot): The robot for which the inverse kinematics should be computed 
        end_effector_position ([type]): the desrired endeffector position

    Returns:
        Dict[str, float]: combined actuated state of all :py:class`KinematicGroup` objects.
    """

    opti = Opti()
    matrix, states_to_solve_for = robot.get_symbolic_rep(opti,"placeholder_endeffector")

    # position only inverse kinematics
    translation = matrix.get_translation()
    equation = ((translation[0] - end_effector_position[0])**2 + 
                (translation[1] - end_effector_position[1])**2 + 
                (translation[2] - end_effector_position[2])**2)


    opti.minimize(equation)
    p_opts = {"print_time": False}
    s_opts = {"print_level": 0, "print_timing_statistics": "no"}


    opti.solver('ipopt', p_opts, s_opts)
    sol = opti.solve()

    #print(robot.get_virtual_state())

    solved_states = robot.solver_to_virtual_state(sol,states_to_solve_for)
    robot.set_virtual_state(solved_states)
    actuated_state = robot.get_actuated_state()
    return actuated_state
