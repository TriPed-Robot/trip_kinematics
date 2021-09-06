from typing import Dict, List
from trip_kinematics.HomogenTransformationMatrix import TransformationMatrix
from casadi import SX, nlpsol, vertcat
from numpy import array
from trip_kinematics.KinematicGroup import OpenKinematicGroup, KinematicGroup, Transformation



class Robot:
    """A class managing multiple :py:class`KinematicGroup` objects pable of building tree like kinematic topologies.

    Args:
        kinematic_chain (List[KinematicGroup]): A list of Kinematic Groups and Transformations with make up the robot.
                                                Transformations are automatically converted to groups

    Raises:
        KeyError: "More than one robot actuator has the same name! Please give each actuator a unique name" 
                  if there are actuated states with the same names between the :py:class`KinematicGroup` objects of the :py:class`Robot`
        KeyError: "More than one robot virtual transformation has the same name! Please give each virtual transformation a unique name"
                  if there are joints with the same names between the :py:class`KinematicGroup` objects of the :py:class`Robot`
    """

    def __init__(self, kinematic_chain: List[KinematicGroup]) -> None:

        self._group_dict = {}
        self._actuator_group_mapping = {}
        self._virtual_group_mapping = {}
        for i in range(len(kinematic_chain)):
            group = kinematic_chain[i]
            if isinstance(group,Transformation):
                print("Warning: Transformation "+str(group)+" was converted to a OpenKinematicGroup with parent "+str(kinematic_chain[i-1]))
                if i >0:
                    group = OpenKinematicGroup(name=str(group),virtual_transformations=[group],
                                           parent= self._group_dict[str(kinematic_chain[i-1])])
                else:
                    group = OpenKinematicGroup(str(group),[group])

            self._group_dict[str(group)]=group
            if group.get_virtual_state() != {}:
                group_actuators = group.get_actuated_state().keys()
                for key in group_actuators:
                    if key in self._actuator_group_mapping.keys():
                        raise KeyError("More than one robot actuator has the same name! Please give each actuator a unique name")
                    self._actuator_group_mapping[key]=str(group)

                for key in group.get_virtual_state().keys():
                    if key in self._virtual_group_mapping.keys():
                        raise KeyError("More than one robot virtual transformation has the same name! Please give each virtual transformation a unique name")
                    self._virtual_group_mapping[key]=str(group)
     
    def get_groups(self):
        """Returns a dictionary of the py:class`KinematicGroup` managed by the :py:class`Robot`-

        Returns:
            Dict[str, KinematicGroup]: The dictionary of py:class`KinematicGroup` objects.
        """
        return self._group_dict

    def set_virtual_state(self, state: Dict[str,Dict[str, float]]):
        """Sets the virtual state of multiple virtual joints of the robot.

        Args:
            state (Dict[str,Dict[str, float]]): A dictionary containing the members of :py:attr:`__virtual_state` that should be set. 
                                                The new values need to be valid state for the state of the joint.
        """
        for key in state.keys():
            virtual_state = {key:state[key]}
            self._group_dict[self._virtual_group_mapping[key]].set_virtual_state(virtual_state)
    
    def set_actuated_state(self, state: Dict[str, float]):
        """Sets the virtual state of multiple actuated joints of the robot.

        Args:
            state (Dict[str, float]):  A dictionary containing the members of :py:attr:`__actuated_state` that should be set. 
        """
        grouping = {}
        for key in state.keys():
            if self._actuator_group_mapping[key] not in grouping.keys():
                grouping[self._actuator_group_mapping[key]] = {}
            grouping[self._actuator_group_mapping[key]][key]=state[key]
        for key in grouping.keys():
            self._group_dict[key].set_actuated_state(grouping[key])


    def get_actuated_state(self):
        """Returns the actuated state of the :py:class`Robot` comprised of the actuated states of the individual :py:class`KinematicGroup`.

        Returns:
            Dict[str, float]: combined actuated state of all :py:class`KinematicGroup` objects.
        """
        actuated_state={}
        for key in self._group_dict.keys():
            actuated_group = self._group_dict[key].get_actuated_state()
            if actuated_group != None:
                for actuated_key in actuated_group:
                    actuated_state[actuated_key]=actuated_group[actuated_key]
        return actuated_state

    def get_virtual_state(self):
        """Returns the virtual state of the :py:class`Robot` comprised of the virtual states of the individual :py:class`KinematicGroup`.

        Returns:
            Dict[str,Dict[str, float]]: combined virtual state of all :py:class`KinematicGroup` objects.
        """
        virtual_state={}
        for group_key in self._group_dict.keys():
            group_state = self._group_dict[group_key].get_virtual_state()
            if group_state != {}:
                for key in group_state.keys():
                    virtual_state[key]=group_state[key]
        return virtual_state


    def get_symbolic_rep(self,endeffector: str):
        """This Function returnes a symbolic representation of the virtual chain.

        Args:
            endeffector (str):  The name of the group whose virtual chain models the desired endeffector
        Returns:
            TransformationMatrix: The :py:class:`TransformationMatrix` containing symbolic objects
        """
        matrix = TransformationMatrix()

        symbolic_state = []
        symbolic_keys  = []

        group_dict = self.get_groups()
        if endeffector not in group_dict.keys():
            raise KeyError("The endeffector must be a valid group name. Valid group names for this robot are: "+str(group_dict.keys()))

        endeff_group   = group_dict[endeffector]
        current_parent = endeff_group.parent
        current_key    = endeffector
        group_key_list = [endeffector]

        while current_parent != current_key:
            next_group     = group_dict[current_parent]
            current_key    = current_parent
            current_parent = next_group.parent
            group_key_list.append(current_key)

        group_key_list.reverse()
        for group_key in group_key_list:
            group = group_dict[group_key]
            virtual_trafo  = group.get_virtual_transformations()

            for virtual_key in virtual_trafo.keys():
                virtual_transformation = virtual_trafo[virtual_key]
                state = virtual_transformation.get_state()

                if state != {}:
                    for key in state.keys():
                        state[key] = SX.sym(virtual_key+'_'+key)
                        symbolic_state.append(state[key])
                        symbolic_keys.append([virtual_key,key])


                hmt = virtual_transformation.get_transformation_matrix()
                matrix = matrix * hmt

        hom_matrix = SX.zeros(4,4)
        for i in range(4):
            for j in range(4):
                hom_matrix[i,j] = matrix.matrix[i,j]   

        return hom_matrix, symbolic_state, symbolic_keys

    

class SimpleInvKinHandle:
    """[summary]
    """
    def __init__(self,robot : Robot,endeffector: str,orientation=False):
            
        matrix, symboles, self._symbolic_keys = robot.get_symbolic_rep(endeffector)
        self.endeffector = endeffector
        self._robot =robot

        if orientation == False:
            end_effector_position = SX.sym("end_effector_pos",3)
            objective = ((matrix[0,3] - end_effector_position[0])**2 + 
                        (matrix[1,3] - end_effector_position[1])**2 + 
                        (matrix[2,3] - end_effector_position[2])**2)

            nlp  = {'x':vertcat(*symboles),'f':objective,'p':end_effector_position}
            opts = {'ipopt.print_level':0, 'print_time':0}
            self.inv_kin_solver = nlpsol('inv_kin','ipopt',nlp,opts)
        pass

    def solve_virtual(self,initial_tip,target):
        x0 = self._virtual_to_solver_state(initial_tip)
        if len(x0) != len(self._symbolic_keys):
            raise RuntimeError("The initial state has "+str(len(x0))+ " values, while the solver expected ",str(len(self._symbolic_keys)))
        solution = self.inv_kin_solver(x0= x0,p=target)
        return self._solver_to_virtual_state(solution['x'])

    def solve_actuated(self,initial_tip,target):
        virtual_state = self.solve_virtual(initial_tip=initial_tip,target=target)
        self._robot.set_virtual_state(virtual_state)
        actuated_state = self._robot.get_actuated_state()
        return actuated_state

    def _solver_to_virtual_state(self,solver_state):
        """This Function maps the solution of a casadi solver to the virtual state of a robot

        Args:
            solver_state ([type]): A solution of a nlp solver
        Returns:
            Dict[str,Dict[str, float]]: a :py:attr:`virtual_state` of a robot.
        """
        virtual_state = {}
        solver_state = array(solver_state) #convert casadi DM to usable datatype
        for i in range(len(solver_state)):
            outer_key = self._symbolic_keys[i][0]
            inner_key = self._symbolic_keys[i][1]
            if outer_key not in virtual_state.keys():
                virtual_state[outer_key] = {}

            virtual_state[outer_key][inner_key] = solver_state[i][0] 
        return virtual_state

    def _virtual_to_solver_state(self,virtual_state):
            solver_state = []
            for i in range(len(self._symbolic_keys)):
                solver_state.append(virtual_state[self._symbolic_keys[i][0]][self._symbolic_keys[i][1]])
            return solver_state

def forward_kinematics(robot: Robot,endeffector):
    """Calculates a robots transformation from base to endeffector using its current state

    Args:
        robot (Robot): The robot for which the forward kinematics should be computed

    Returns:
        numpy.array : The Transformation from base to endeffector 
    """
    transformation = TransformationMatrix()
    group_dict = robot.get_groups()
    if endeffector not in group_dict.keys():
        raise KeyError("The endeffector must be a valid group name. Valid group names for this robot are: "+str(group_dict.keys()))
    endeff_group   = group_dict[endeffector]
    current_parent = endeff_group.parent
    current_key    = endeffector
    group_key_list = [endeffector]
    while current_parent != current_key:
        next_group     = group_dict[current_parent]
        current_key    = current_parent
        current_parent = next_group.parent
        group_key_list.append(current_key)

    group_key_list.reverse()
    for group_key in group_key_list:
        group = group_dict[group_key]
        hmt = group.get_transformation_matrix()
        transformation = transformation * hmt
    return transformation.matrix



def inverse_kinematics(robot: Robot, endeffector, target_position, inv_kin_handle = None, orientation=False,initial_tip = None,type="simple"):
    supported_types = ["simple"]

    if type not in supported_types:
        raise KeyError("The specified type does not correspond to any inv kin solver. Supported types are "+str(supported_types))
    elif type == "simple":
        if inv_kin_handle == None:
            inv_kin_handle = SimpleInvKinHandle(robot,endeffector,orientation,type)
        elif endeffector != inv_kin_handle.endeffector:
            raise KeyError("The specified endeffector "+endeffector+
                           "does not match the endeffector "+inv_kin_handle.endeffector+"of the kinematic handle")
        if initial_tip == None:
            initial_tip = robot.get_virtual_state()

        return inv_kin_handle.solve_actuated(initial_tip=initial_tip,target=target_position)
