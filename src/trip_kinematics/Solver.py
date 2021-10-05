from copy import deepcopy
from casadi import SX, nlpsol, vertcat
from numpy import array

from trip_kinematics.Robot import Robot


class SimpleInvKinSolver:
    """[summary]
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

    def solve_virtual(self, target, initial_tip=None):
        if initial_tip is None:
            x0 = [0]*len(self._symbolic_keys)
        else:
            x0 = self._virtual_to_solver_state(initial_tip)
        if len(x0) != len(self._symbolic_keys):
            raise RuntimeError("The initial state has "+str(len(x0)) +
                               " values, while the solver expected ", str(len(self._symbolic_keys)))
        solution = self.inv_kin_solver(x0=x0, p=target)
        return self._solver_to_virtual_state(solution['x'])

    def solve_actuated(self, target, initial_tip=None, mapping_argument=None):
        virtual_state = self.solve_virtual(
            target=target, initial_tip=initial_tip)
        if mapping_argument is not None:
            self._robot.pass_group_arg_v_to_a(mapping_argument)

        self._robot.set_virtual_state(virtual_state)
        actuated_state = self._robot.get_actuated_state()
        return actuated_state

    def _solver_to_virtual_state(self, solver_state):
        """This Function maps the solution of a casadi solver to the virtual state of a robot

        Args:
            solver_state ([type]): A solution of a nlp solver
        Returns:
            Dict[str,Dict[str, float]]: a :py:attr:`virtual_state` of a robot.
        """
        virtual_state = {}
        # convert casadi DM to usable datatype
        solver_state = array(solver_state)
        for i in range(len(solver_state)):
            outer_key = self._symbolic_keys[i][0]
            inner_key = self._symbolic_keys[i][1]
            if outer_key not in virtual_state.keys():
                virtual_state[outer_key] = {}

            virtual_state[outer_key][inner_key] = solver_state[i][0]
        return virtual_state

    def _virtual_to_solver_state(self, virtual_state):
        solver_state = []
        for i in range(len(self._symbolic_keys)):
            solver_state.append(
                virtual_state[self._symbolic_keys[i][0]][self._symbolic_keys[i][1]])
        return solver_state
