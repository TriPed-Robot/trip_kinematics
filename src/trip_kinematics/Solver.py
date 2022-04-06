from typing import Dict
from copy import deepcopy
from casadi import SX, nlpsol, vertcat, jacobian, Function, pinv
from numpy import array
from numpy.linalg import norm

from trip_kinematics.Robot import Robot, forward_kinematics
from trip_kinematics.Utility import get_translation


class SimpleInvKinSolver:
    """A Simple Kinematic Solver Class that calculates
       the inverse kinematics for a given endeffector.

    Args:
        robot (Robot): The Robot for which the kinematics should be calculated
        endeffector (str): the name of the endeffector
        orientation (bool, optional): Boolean flag deciding if inverse kinematics
                                      targets also specify orientation.
                                      Defaults to False.
        update_robot (bool, optional): Boolean flag decding if the inverse kinematics should
                                       immediately update the robot model.
                                       Defaults to False.
    """

    def __init__(self, robot: Robot, endeffector: str, orientation=False, update_robot=False):

        matrix, symboles, self._symbolic_keys = robot.get_symbolic_rep(
            endeffector)
        self.endeffector = endeffector
        if update_robot:
            self._robot = robot
        else:
            self._robot = deepcopy(robot)

        if orientation is False:
            end_effector_position = SX.sym("end_effector_pos", 3)
            objective = ((matrix[0, 3] - end_effector_position[0])**2 +
                         (matrix[1, 3] - end_effector_position[1])**2 +
                         (matrix[2, 3] - end_effector_position[2])**2)

            nlp = {'x': vertcat(*symboles), 'f': objective,
                   'p': end_effector_position}
            opts = {'ipopt.print_level': 0, 'print_time': 0}
            self.inv_kin_solver = nlpsol('inv_kin', 'ipopt', nlp, opts)

    def solve_virtual(self, target: array, initial_tip=None):
        """Returns the virtual state needed for the endeffector to be in the target position
        Args:
            target (numpy.array): The target state of the endeffector.
                                  Either a 3 dimensional position or a 4x4 homogenous transformation
            initial_tip ((Dict(str,Dict(str,float))), optional): Initial state of the solver
                                                                 as a virtual state.
                                                                 Defaults to None
                                                                 in which case zeros are used.

        Returns:
            Dict(str,Dict(str,float)): The virtual state leading the endeffector
                                       to the target position.
        """
        if initial_tip is None:
            initial_guess = [0]*len(self._symbolic_keys)
        else:
            initial_guess = self._virtual_to_solver_state(initial_tip)
        if len(initial_guess) != len(self._symbolic_keys):
            raise RuntimeError("The initial state has "+str(len(initial_guess)) +
                               " values, while the solver expected ", str(len(self._symbolic_keys)))
        solution = self.inv_kin_solver(x0=initial_guess, p=target)
        return self._solver_to_virtual_state(solution['x'])

    def solve_actuated(self, target: array, initial_tip=None, mapping_argument=None):
        """Returns the actuated state needed for the endeffector to be in the target position
        Args:
            target (numpy.array): The target state of the endeffector.
                                  Either a 3 dimensional position or a 4x4 homogenous transformation
            initial_tip (Dict[str,Dict[str, float]], optional): Initial state of the solver.
                                                                In this case refers to a
                                                                virtual state.
                                                                Defaults to None
                                                                in which case zeros are used.
            mapping_argument ([type], optional): Optional arguments for the virtual_to_actuated
                                                 mappings of the robot.
                                                 Defaults to None.

        Returns:
            Dict(str,float): The actuated state leading the endeffector to the target position.
        """
        virtual_state = self.solve_virtual(
            target=target, initial_tip=initial_tip)
        if mapping_argument is not None:
            self._robot.pass_group_arg_v_to_a(mapping_argument)

        self._robot.set_virtual_state(virtual_state)
        actuated_state = self._robot.get_actuated_state()
        return actuated_state

    def _solver_to_virtual_state(self, solver_state):
        """This function maps the solution of a casadi solver to the virtual state of a robot.

        Args:
            solver_state ([type]): A solution of a nlp solver
        Returns:
            Dict[str,Dict[str, float]]: a :py:attr:`virtual_state` of a robot.
        """
        virtual_state = {}
        # convert casadi DM to usable datatype
        solver_state = array(solver_state)
        for i, solver_state_value in enumerate(solver_state):
            outer_key = self._symbolic_keys[i][0]
            inner_key = self._symbolic_keys[i][1]
            if outer_key not in virtual_state:
                virtual_state[outer_key] = {}

            virtual_state[outer_key][inner_key] = solver_state_value[0]
        return virtual_state

    def _virtual_to_solver_state(self, virtual_state: Dict[str, Dict[str, float]]):
        """This function maps the virtual state of a robot to the solution of a casadi solver.
        Args:
            virtual_state (Dict(str,Dict(str,float))): A virtual state of the robot

        Returns:
            [type]: A solution of a nlp solver
        """
        solver_state = []
        for key in self._symbolic_keys:
            solver_state.append(
                virtual_state[key[0]][key[1]])
        return solver_state


class NewtonSolver:
    """A Inverse Kinematic Solver based  on Newtons method.

    Args:
        robot (Robot): The Robot for which the kinematics should be calculated
        endeffector (str): the name of the endeffector
        orientation (bool, optional): Boolean flag deciding if inverse kinematics
                                      targets also specify orientation.
                                      Defaults to False.
        update_robot (bool, optional): Boolean flag decding if the inverse kinematics should
                                       immediately update the robot model.
                                       Defaults to False.
        options (Dict, optional): A dictionary containing options for the Newton solver. Possible keys:
                                  stepsize:       the step length along the gradient
                                  max_iterations: the maximum number of iterations
                                                  before terminating
                                  precision:      the minimum amount of joint value change
                                                  before terminating
    """

    def __init__(self, robot: Robot, endeffector: str,
                 orientation=False, update_robot=False, options=None):

        self.stepsize = 0.05
        self.max_iterations = 1000
        self.precision = 0.01

        if isinstance(options, Dict):
            if 'stepsize' in options:
                self.stepsize = options['stepsize']
            if 'max_iterations' in options:
                self.max_iterations = options['max_iterations']
            if 'precision' in options:
                self.precision = options['precision']

        matrix, symboles, self._symbolic_keys = robot.get_symbolic_rep(
            endeffector)
        self.endeffector = endeffector
        self.orientation = orientation
        if update_robot:
            self._robot = robot
        else:
            self._robot = deepcopy(robot)

        if orientation is False:
            objective = vertcat(matrix[0, 3], matrix[1, 3], matrix[2, 3])

        # define gradient function for descent
        joint_symboles = vertcat(*symboles)
        jacobi_matrix = jacobian(objective, joint_symboles)
        self.p_inv_jacobian = Function(
            'gradient', [joint_symboles], [pinv(jacobi_matrix)])
        self.pose = Function('pose',[joint_symboles],[objective])

    def solve_virtual(self, target: array, initial_tip=None):
        """Returns the virtual state needed for the endeffector to be in the target position
        Args:
            target (numpy.array): The target state of the endeffector.
                                  Either a 3 dimensional position or a 4x4 homogenous transformation
            initial_tip ((Dict(str,Dict(str,float))), optional): Initial state of the solver
                                                                 as a virtual state.
                                                                 Defaults to None
                                                                 in which case zeros are used.

        Returns:
            Dict(str,Dict(str,float)): The virtual state leading the endeffector
                                       to the target position.
        """

        if initial_tip is None:
            joint_values = [0]*len(self._symbolic_keys)
        else:
            joint_values = self._virtual_to_solver_state(initial_tip)

        
        for _ in range(self.max_iterations):
            # compute error between current state and target
            endeffector_pose = self.pose(joint_values)
            pose_error = target-endeffector_pose

            new_joint_values = [0]*len(self._symbolic_keys)
            new_joint_values = joint_values + self.stepsize * \
                (self.p_inv_jacobian(joint_values)
                    @pose_error)

            if norm(pose_error) <= self.precision:
                break
            joint_values = new_joint_values
        print(self._solver_to_virtual_state(array(joint_values)))
        return self._solver_to_virtual_state(array(joint_values))

    def solve_actuated(self, target: array, initial_tip=None, mapping_argument=None):
        """Returns the actuated state needed for the endeffector to be in the target position
        Args:
            target (numpy.array): The target state of the endeffector.
                                  Either a 3 dimensional position or a 4x4 homogenous transformation
            initial_tip (Dict[str,Dict[str, float]], optional): Initial state of the solver.
                                                                In this case refers to a
                                                                virtual state.
                                                                Defaults to None
                                                                in which case zeros are used.
            mapping_argument ([type], optional): Optional arguments for the virtual_to_actuated
                                                 mappings of the robot.
                                                 Defaults to None.

        Returns:
            Dict(str,float): The actuated state leading the endeffector to the target position.
        """
        virtual_state = self.solve_virtual(
            target=target, initial_tip=initial_tip)
        if mapping_argument is not None:
            self._robot.pass_group_arg_v_to_a(mapping_argument)

        self._robot.set_virtual_state(virtual_state)
        actuated_state = self._robot.get_actuated_state()
        return actuated_state

    def _solver_to_virtual_state(self, solver_state):
        """This function maps the solution of a casadi solver to the virtual state of a robot.

        Args:
            solver_state ([type]): A solution of the ccd solver
        Returns:
            Dict[str,Dict[str, float]]: a :py:attr:`virtual_state` of a robot.
        """
        virtual_state = {}
        # convert casadi DM to usable datatype
        solver_state = array(solver_state)
        for i, solver_state_value in enumerate(solver_state):
            outer_key = self._symbolic_keys[i][0]
            inner_key = self._symbolic_keys[i][1]
            if outer_key not in virtual_state:
                virtual_state[outer_key] = {}

            virtual_state[outer_key][inner_key] = solver_state_value[0]
        return virtual_state

    def _virtual_to_solver_state(self, virtual_state: Dict[str, Dict[str, float]]):
        """This function maps the virtual state of a robot to the solution of a casadi solver.
        Args:
            virtual_state ( Dict(str,Dict(str,float))): A virtual state of the robot

        Returns:
            [type]: A solution of the Newton solver
        """
        solver_state = []
        for key in self._symbolic_keys:
            solver_state.append(
                virtual_state[key[0]][key[1]])
        return solver_state
