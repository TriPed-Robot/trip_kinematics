from typing import Dict, List, Callable
from copy import deepcopy

from trip_kinematics.Utility import identity_transformation
from trip_kinematics.Transformation import Transformation


class KinematicGroup():
    """Initializes a :py:class:`KinematicGroup` object.

    Args:
        name (str): The unique name identifying the group.
                    No two :py:class:`KinematicGroup` objects of a :py:class`Robot` should have the
                    same name
        virtual_chain (List[Transformation]): A list of :py:class:`Transformation`
                                              objects forming a serial Kinematic chain.
        actuated_state (List[Dict[str, float]]): The State of the Groups actuated joints.
        actuated_to_virtual (Callable): Maps the :py:attr:`actuated_state` to the
                                        :py:attr:`virtual_state` of the :py:attr:`virtual_chain`.
        virtual_to_actuated (Callable): Maps the :py:attr:`virtual_state` of the
                                        :py:attr:`virtual_chain` to the
                                        :py:attr:`actuated_state`.
        act_to_virt_args ([type], optional): Arguments that can be passed to
                                             :py:attr:`actuated_to_virtual` during the initial
                                             testing of the function. Defaults to None.
        virt_to_act_args ([type], optional): Arguments that can be passed to
                                             :py:attr:`virtual_to_actuated` during the initial
                                             testing of the function. Defaults to None.
        parent (Union(Transformation,KinematicGroup), optional): The transformation or group
                                                                 preceding the
                                                                 :py:class:`KinematicGroup`.
                                                                 Defaults to None.

    Raises:
        ValueError: 'Error: Actuated state is missing.
                     You provided a mapping to actuate the group but no state to be actuated.'
                    if there is no :py:attr:`actuated_state` despite a mapping being passed
        ValueError: 'Error: Only one mapping provided. You need mappings for both ways.
                     Consider to pass a trivial mapping.'
                    if either :py:attr:`actuated_to_virtual` or :py:attr:`virtual_to_actuated`
                    was not set despite providing a :py:attr:`actuated_state`.
        ValueError: 'Error: Mappings missing. You provided an actuated state but no mappings.
                    If you want a trivial mapping you don\'t need to pass an actuated state.
                    Trip will generate one for you.'
                    if both :py:attr:`actuated_to_virtual` and :py:attr:`virtual_to_actuated`
                    were not set despite providing a :py:attr:`actuated_state`.
        RuntimeError: "actuated_to_virtual does not fit virtual state" if the
                      :py:attr:`actuated_to_virtual` function does not return a valid
                      :py:attr:`virtual_state` dictionary.
        RuntimeError: "virtual_to_actuated does not fit actuated state" if the
                      :py:attr:`virtual_to_actuated` function does not return a valid
                      :py:attr:`actuated_state` dictionary.

    """

    @staticmethod
    def object_list_to_key_lists(object_lst):
        """Helper function which transforms dictionary into list of keys.

        Args:
            object_lst (Dict): The dictionary to be transformed

        Returns:
            list(str): A list of keys
        """
        return list(object_lst.keys())

    def __init__(self, name: str, virtual_chain: List[Transformation],
                 actuated_state: Dict[str, float],
                 actuated_to_virtual: Callable,
                 virtual_to_actuated: Callable,
                 act_to_virt_args=None, virt_to_act_args=None,
                 parent=None):

        self._name = name
        self.children = []

        if parent is None:
            self.parent = name
        elif isinstance(parent, (KinematicGroup, Transformation)):
            self.parent = str(parent)
            parent.add_children(name)
        else:
            raise TypeError(
                "The parent of a group must be a either a KinematicGroup or a Transformation")

        self._virtual_chain = {}
        virtual_state = {}
        root = None
        endeffector = None
        for transformation in virtual_chain:
            self._virtual_chain[str(transformation)] = transformation
            if transformation.get_state() != {}:
                virtual_state[str(transformation)] = transformation.get_state()
            if len(transformation.children) >= 2:
                raise ValueError("The Transformation "+str(transformation) +
                                 "contains more than one child." +
                                 " In a virtual chain each transfromation can only have one child")
            if transformation.children == []:
                endeffector = str(transformation)
            if transformation.parent == str(transformation):
                if root is None:
                    root = str(transformation)
                else:
                    raise ValueError("Transformations "+root+" and "+str(transformation) +
                                     " are both connected to the ground." +
                                     " In a virtual chain only one transformation can be" +
                                     " connected to the ground")

        if virtual_state:
            if (actuated_to_virtual or virtual_to_actuated) and not actuated_state:
                raise ValueError('Error: Actuated state is missing.' +
                                 ' You provided a mapping to actuate the group but' +
                                 ' no state to be actuated.')
            if ((actuated_to_virtual and not virtual_to_actuated) or
                    (not actuated_to_virtual and virtual_to_actuated)):
                raise ValueError('Error: Only one mapping provided.' +
                                 ' You need mappings for both ways.' +
                                 ' Consider to pass a trivial mapping.')
            if not (actuated_to_virtual or virtual_to_actuated):
                raise ValueError('Error: Mappings missing.' +
                                 ' You provided an actuated state but no mappings.' +
                                 ' If you want a trivial mapping you can use' +
                                 ' the OpenKinematicGroup.')
        else:
            if actuated_state or (actuated_to_virtual or virtual_to_actuated):
                raise ValueError('Error: Allthough the Group is static, a actuated state and' +
                                 ' or mapping was provided that is not a None object.')

        # build a sorted list of transformation keys from root to endeffector
        # later used in get_transformation_matrix
        current_key = endeffector
        self._chain_keys = [endeffector]
        current_parent = self._virtual_chain[current_key].parent
        while current_parent != current_key:
            next_trafo = self._virtual_chain[current_parent]
            current_key = current_parent
            current_parent = next_trafo.parent
            self._chain_keys.append(current_key)
        self._chain_keys.reverse()

        # check the consistency of the provided mapping functions if the group is not static
        if actuated_state:
            self._original_actuated_to_virtual = actuated_to_virtual
            if act_to_virt_args:
                self._actuated_to_virtual = lambda state: actuated_to_virtual(
                    state, *act_to_virt_args)
            self._actuated_to_virtual = actuated_to_virtual
            actuated_to_virtual_to_check = self._actuated_to_virtual(
                actuated_state)

            if KinematicGroup.object_list_to_key_lists(actuated_to_virtual_to_check) != \
               KinematicGroup.object_list_to_key_lists(virtual_state):
                raise RuntimeError(
                    "actuated_to_virtual does not fit virtual state")
            self._original_virtual_to_actuated = virtual_to_actuated
            if virt_to_act_args:
                self._virtual_to_actuated = lambda state: virtual_to_actuated(
                    state, *virt_to_act_args)
            self._virtual_to_actuated = virtual_to_actuated
            virtual_to_actuated_to_check = virtual_to_actuated(virtual_state)

            if KinematicGroup.object_list_to_key_lists(virtual_to_actuated_to_check) != \
               KinematicGroup.object_list_to_key_lists(actuated_state):
                raise RuntimeError(
                    "virtual_to_actuated does not fit actuated state")

            # Check if inital values fit actuated_to_virtual's
            # and virtual_to_actuated's calculated values.
            for key in actuated_to_virtual_to_check.keys():
                state = actuated_to_virtual_to_check[key]
                init_values_do_not_match = False
                for state_key in state.keys():
                    if state[state_key] != virtual_state[key][state_key]:
                        init_values_do_not_match = True
            if init_values_do_not_match:
                print('Calculated state values do not match given values!' +
                      'Using set_state() before forward_kinematics() or' +
                      ' inverse_kinematics() is recommended.')

        self.virtual_state = deepcopy(virtual_state)
        self.actuated_state = deepcopy(actuated_state)

    def set_virtual_state(self, state: Dict[str, Dict[str, float]]):
        """Sets the :py:attr:`virtual_state` of the Group and
           automatically updates the corresponding :py:attr:`actuated_state`.

        Args:
            state (Dict[str,Dict[str, float]]): A dictionary containing the members of
                                                :py:attr:`virtual_state` that should be set.
                                                The new values need to be valid state
                                                for the state of the joint.

        Raises:
            RuntimeError: if all  :py:class:`Transformation` objects
                          of :py:attr:`_virtual_chain` are static.
            ValueError: if the state to set is not part of keys of :py:attr:`virtual_state`
        """

        if self.actuated_state is None:
            raise RuntimeError(
                "This is a static group! There is no state to be set")

        if all(key in self.virtual_state.keys()for key in state.keys()):
            for key in state.keys():
                self.virtual_state[key] = state[key]
            self._update_chain()
            self.actuated_state = self._virtual_to_actuated(self.virtual_state)
        else:
            raise ValueError('Error: One or more keys are not part of the virtual state. ' +
                             'correct keys are: '+str(self.virtual_state.keys()))

    def set_actuated_state(self, state: Dict[str, float]):
        """Sets the :py:attr:`actuated_state` of the Group and
           automatically updates the corresponding :py:attr:`virtual_state`.

        Args:
            state (Dict[str, float]): A dictionary containing the members
                                      of :py:attr:`actuated_state` that should be set.


        Raises:
            RuntimeError:  if all  :py:class:`Transformation` objects
                           of :py:attr:`_virtual_chain` are static.
            ValueError: if the state to set is not part of keys of :py:attr:`actuated_state`
        """

        if self.actuated_state is None:
            raise RuntimeError(
                "This is a static group! There is no state to be set.")

        if all(key in self.actuated_state.keys()for key in state.keys()):
            for key in state.keys():
                self.actuated_state[key] = state[key]
            new_virtual_state = self._actuated_to_virtual(self.actuated_state)
            for key in new_virtual_state.keys():
                self.virtual_state[key] = new_virtual_state[key]
            self._update_chain()
        else:
            raise ValueError(
                "Error: One or more keys are not part of the actuated state. correct keys are: "
                + str(self.actuated_state.keys()))

    def __str__(self):
        return self._name

    def get_name(self):
        """Returns the :py:attr:`_name` of the :py:class:`KinematicGroup`

        Returns:
            str: the :py:attr:`_name` attribute
        """
        return deepcopy(self._name)

    def get_virtual_state(self):
        """Returns a copy of the :py:attr:`virtual_state`
           attribute of the :py:class:`KinematicGroup` object.

        Returns:
            Dict[str,Dict[str,float]]: a copy of the :py:attr:`virtual_state`
        """
        return deepcopy(self.virtual_state)

    def get_actuated_state(self):
        """Returns a copy of the :py:attr:`actuated_state`
           attribute of the :py:class:`KinematicGroup` object.

        Returns:
            Dict[str,float]: a copy of the :py:attr:`actuated_state`
        """
        if self.actuated_state:
            return deepcopy(self.actuated_state)
        return None

    def get_transformation_matrix(self):
        """Calculates the full transformationmatrix from
           the start of the virtual chain to its endeffector.

        Returns:
            array: The homogenous transformation matrix from
                   the start of the virtual chain to its endeffector.
        """

        # Identity matrix
        transformation = identity_transformation()
        transformations = self._virtual_chain
        for key in self._chain_keys:
            part = transformations[key]
            hmt = part.get_transformation_matrix()
            transformation = transformation @ hmt

        return transformation

    def get_virtual_chain(self):
        """Returns a copy of the :py:attr:`_virtual_chain`
           attribute of a :py:class:`KinematicGroup` object.

        Returns:
            Dict[str,Transformation]: a copy of the :py:attr:`_virtual_chain`
        """
        return deepcopy(self._virtual_chain)

    def _update_chain(self):
        """Propagates changes from the :py:attr:`virtual_state`
           to the underlying :py:class:`Transformation` objects.
        """
        for key in self.virtual_state.keys():
            self._virtual_chain[key].set_state(self.virtual_state[key])

    def add_children(self, child: str):
        """Adds the name of a :py:class:`KinematicGroup` or :py:class:`Transformation`
           as a child.

        Args:
            child (str): the name of a :py:class:`KinematicGroup` or :py:class:`Transformation`
        """
        self.children.append(child)

    def pass_arg_v_to_a(self, argv):
        """Allows arguments to be passed the :py:attr:`virtual_to_actuated` mapping.

        Args:
            argv ([type]): arguments to be passed.
        """
        g_map = self._original_virtual_to_actuated
        self._virtual_to_actuated = lambda state: g_map(state, *argv)

    def pass_arg_a_to_v(self, argv):
        """Allows arguments to be passed the :py:attr:`actuated_to_virtual` mapping.

        Args:
            argv ([type]): arguments to be passed.
        """
        f_map = self._original_actuated_to_virtual
        self._actuated_to_virtual = lambda state: f_map(state, *argv)


class OpenKinematicGroup(KinematicGroup):
    """A subclass of the :py:class:`KinematicGroup` that assumes that all states
       of the virtual_chain are actuated and automatically generates mappings.
       Typically only used internally by the :py:class`Robot` class to convert
       :py:class`Transformation` objects to :py:class`KinematicGroup`s.

    Args:
        name (str): The unique name identifying the group.
                    No two :py:class:`KinematicGroup` objects of a :py:class`Robot` should have the
                    same name
        virtual_chain (List[Transformation]): A list of :py:class:`Transformation`
                                              objects forming a serial Kinematic chain.
        parent (Union(Transformation,KinematicGroup), optional): The transformation or group
                                                                 preceding the
                                                                 :py:class:`KinematicGroup`.
                                                                 Defaults to None.

    """

    def __init__(self, name: str, virtual_chain: List[Transformation], parent=None):
        virtual_state = {}
        virtual_trafo_dict = {}
        for transformation in virtual_chain:
            virtual_trafo_dict[str(transformation)] = transformation
            if transformation.get_state() != {}:
                virtual_state[str(transformation)] = transformation.get_state()

        if any(virtual_state):  # group is dynamic
            actuated_state_dummy = {}
            f_map = {}
            g_map = {}

            for virtual_key, transformation in virtual_trafo_dict.items():

                state = transformation.get_state()
                for key, value in state.items():
                    concat_key = transformation.get_name() + '_' + key
                    actuated_state_dummy.setdefault(concat_key, value)
                    f_map.setdefault(concat_key, (key, virtual_key))
                    g_map.setdefault((key, virtual_key), concat_key)

            # create trivial mappings
            def trivial_actuated_to_virtual(state):
                out = {}
                for concat_key, value in state.items():
                    key, index = f_map[concat_key]
                    out[index] = {key: value}
                return out

            def trivial_virtual_to_actuated(states):
                out = {}
                for virtual_key in states.keys():
                    transformation = states[virtual_key]
                    for key in transformation.keys():
                        combined_key = (key, virtual_key)
                        new_key = g_map[combined_key]
                        out[new_key] = states[virtual_key][key]
                return out

            actuated_to_virtual = trivial_actuated_to_virtual
            virtual_to_actuated = trivial_virtual_to_actuated
            actuated_state = actuated_state_dummy
        else:   # group is static
            actuated_state = None
            actuated_to_virtual = None
            virtual_to_actuated = None
        super(OpenKinematicGroup, self).__init__(name=name,
                                                 virtual_chain=virtual_chain,
                                                 actuated_state=actuated_state,
                                                 actuated_to_virtual=actuated_to_virtual,
                                                 virtual_to_actuated=virtual_to_actuated,
                                                 parent=parent)
