from typing import Dict, List, Callable, Union
from trip_kinematics.HomogenTransformationMatrix import TransformationMatrix
from casadi import Function, SX, nlpsol, vertcat
import numpy as np
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


    def get_symbolic_rep(self):
        """This Function returnes a symbolic representation of the virtual chain.


        Returns:
            TransformationMatrix: The :py:class:`TransformationMatrix` containing symbolic objects
        """
        matrix = TransformationMatrix()

        symbolic_state = []
        symbolic_keys  = []

        groups = self.get_groups()
        
        for group_key in groups.keys():
            group         = groups[group_key]
            virtual_trafo = group.get_virtual_transformations()


            for virtual_key in virtual_trafo.keys():
                virtual_transformation = virtual_trafo[virtual_key]
                state = virtual_transformation.state

                
                for key in state.keys():

                    start_value = state[key]
                    state[key] = SX.sym(virtual_key+"_"+key)
                    symbolic_state.append(state[key])
                    symbolic_keys.append([virtual_key,key])
                    #opti_obj.set_initial(state[key], start_value)

                #if state != {}:
                #    symbolic_keys[virtual_key]=state

                hmt = virtual_transformation.get_transformation_matrix()
                matrix = matrix * hmt

        hom_matrix = SX.zeros(4,4)
        for i in range(4):
            for j in range(4):
                hom_matrix[i,j] = matrix.matrix[i,j]   

        return hom_matrix, symbolic_state, symbolic_keys

    def get_inv_kin_handle(self,orientation=False,type="simple"):
        supported_types = ["simple"]
        if type not in supported_types:
            raise KeyError("The specified type does not correspond to any inv kin solver. Supported types are "+str(supported_types))
            
        matrix, symboles, symbolic_keys = self.get_symbolic_rep()
        end_effector_position = SX.sym("end_effector_pos",3)
        objective = ((matrix[0,3] - end_effector_position[0])**2 + 
                    (matrix[1,3] - end_effector_position[1])**2 + 
                    (matrix[2,3] - end_effector_position[2])**2)

        nlp  = {'x':vertcat(*symboles),'f':objective,'p':end_effector_position}
        opts = {'ipopt.print_level':0, 'print_time':0}
        inv_kin_solver = nlpsol('inv_kin','ipopt',nlp,opts)
        return (inv_kin_solver, symbolic_keys)

    @staticmethod
    def solver_to_virtual_state(sol,symbolic_keys):
        """This Function maps the solution of a casadi solver to the virtual state of the robot

        Args:
            sol ([type]): A solution of a nlp solver
            symbolic_state ([type]): A dictionary containing the corresponding virtual state keys.
        Returns:
            Dict[str,Dict[str, float]]: a :py:attr:`virtual_state` of a robot.
        """
        solved_states = {}
        sol = np.array(sol) #convert casadi DM to usable datatype
        for i in range(len(sol)):
            outer_key = symbolic_keys[i][0]
            inner_key = symbolic_keys[i][1]
            if outer_key not in solved_states.keys():
                solved_states[outer_key] = {}

            solved_states[outer_key][inner_key] = sol[i][0] 
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



def inverse_kinematics(robot: Robot, target_position, inv_kin_handle = None, orientation=False,type="simple"):
    supported_types = ["simple"]

    if type not in supported_types:
        raise KeyError("The specified type does not correspond to any inv kin solver. Supported types are "+str(supported_types))

    if inv_kin_handle != None:
        inv_kin_solver = inv_kin_handle[0]
        symbolic_keys  = inv_kin_handle[1]
    else:
        inv_kin_solver,symbolic_keys = robot.get_inv_kin_handle(orientation,type)

    solution = inv_kin_solver(x0= [0,0,0,0],p=target_position)
    

    solved_states = robot.solver_to_virtual_state(solution['x'],symbolic_keys)
    robot.set_virtual_state(solved_states)
    actuated_state = robot.get_actuated_state()
    return actuated_state
