from typing import Dict, List, Callable
from trip_kinematics.Utility import hom_translation_matrix, x_axis_rotation_matrix, y_axis_rotation_matrix, z_axis_rotation_matrix, quat_rotation_matrix, identity_transformation
from copy import deepcopy
from numpy import array


def array_find(arr, obj) -> int:
    index = -1
    try:
        index = arr.index(obj)
        return index
    except:
        return -1



class Transformation():
    """Initializes the :py:class:`Transformation` class. 

    Args:
        name (str): The unique name identifying the . No two :py:class:`Transformation` objects of a :py:class`Robot` should have the same name
        values (Dict[str, float]): A parametric description of the transformation. 
        state_variables (List[str], optional): This list describes which state variables are dynamically changable. 
                                               This is the case if the :py:class:`Transformation` represents a joint. Defaults to [].

    Raises:
        ValueError: A dynamic state was declared that does not correspond to a parameter declared in ``values``.
    """

    @staticmethod
    def get_convention(state: Dict[str, float]):
        """Returns the connvention which describes how the matrix  of a :py:class:`Transformation` is build from its state. 

        Args:
            state (Dict[str, float]): :py:attr:'state'

        Raises:
            ValueError: "Invalid key." If the dictionary kontains keys that dont correspond to a parameter of the transformation.
            ValueError: "State can't have euler angles and quaternions!" If the dictionary contains keys correspondig to multiple mutually exclusive conventions.

        Returns:
            [type]: A string describing the convention 
        """

        valid_keys = ['tx', 'ty', 'tz', 'qw', 'qx',
                      'qy', 'qz', 'rx', 'rz', 'ry']

        quaternion_keys = valid_keys[3:7]
        euler_keys = valid_keys[7:]
        got_quaternion = False
        got_euler = False

        for key in state.keys():
            if array_find(valid_keys, key) == -1:
                raise ValueError("Invalid key, acceptable keys are: "+str(valid_keys))
            if array_find(quaternion_keys, key) >= 0:
                got_quaternion = True
            if array_find(euler_keys, key) >= 0:
                got_euler = True

        if got_euler and got_quaternion:
            raise ValueError("State can't have euler angles and quaternions!")

        if got_euler:
            return "euler"
        else:
            return "quaternion"

    def __init__(self, name: str, values: Dict[str, float], state_variables: List[str] = []):

        self._name = name

        if not set(state_variables) <= set(values.keys()):
            raise ValueError(
                "Element of state_variables do not correspond to key(s) of values dictionary. This can happen if both follow different conventions")

        constants = {}
        state = {}

        self.convention = Transformation.get_convention(values)

        for key in values.keys():
            if array_find(state_variables, key) != -1:
                state.setdefault(key, values.get(key))
            else:
                constants.setdefault(key, values.get(key))

        self._state     = state
        self._constants = constants

    def __str__(self):
        return self._name

    def get_state(self):
        return deepcopy(self._state)
    def set_state(self, state: Dict[str, float]):
        for key in state.keys():
            if not key in self._state.keys():
                raise KeyError("The specified keys is not part of the Transformations state. Maybe the Transofmration uses a different convention?")
            self._state[key] = state[key]

            
    def get_name(self):
        return deepcopy(self._name)

    def get_transformation_matrix(self):
        """Returns a homogeneous transformation matrix build from the :py:attr:`state` and :py:attr:`constants`

        Raises:
            RuntimeError: If the convention used in :py:attr:`state` is not supported. Should normally be catched during initialization.

        Returns:
            [type]: A transformation matrix build using the parameters of the  :py:class:`Transformation` :py:attr:`state`
        """

        # collect transformation parameters from state and constants respectively
        tx = self._constants.get('tx') if 'tx' in self._constants.keys() else self._state.get('tx',0)
        ty = self._constants.get('ty') if 'ty' in self._constants.keys() else self._state.get('ty',0)
        tz = self._constants.get('tz') if 'tz' in self._constants.keys() else self._state.get('tz',0)

        matrix = hom_translation_matrix(tx,ty,tz)

        if self.convention == 'euler':
            rx = self._constants.get('rx') if 'rx' in self._constants.keys() else self._state.get('rx',0)
            ry = self._constants.get('ry') if 'ry' in self._constants.keys() else self._state.get('ry',0)
            rz = self._constants.get('rz') if 'rz' in self._constants.keys() else self._state.get('rz',0)

            matrix[:3, :3] = x_axis_rotation_matrix(
                rx) @ y_axis_rotation_matrix(ry) @ z_axis_rotation_matrix(rz)

        elif self.convention == 'quaternion':
            qw = self._constants.get('qw') if 'qw' in self._constants.keys() else self._state.get('qw',0)
            qx = self._constants.get('qx') if 'qx' in self._constants.keys() else self._state.get('qx',0)
            qy = self._constants.get('qy') if 'qy' in self._constants.keys() else self._state.get('qy',0)
            qz = self._constants.get('qz') if 'qz' in self._constants.keys() else self._state.get('qz',0)

            matrix[:3, :3] = quat_rotation_matrix(qw, qx, qy, qz)
        else:
            raise RuntimeError("No Convention. This should normally be catched during initialization. " + 
                               "Did you retroactively change the keys of the Transformation state?")
        
        return matrix

class KinematicGroup():
    """Initializes a :py:class:`KinematicGroup` object.

    Args:
        name (str): The unique name identifying the group. No two :py:class:`KinematicGroup` objects of a :py:class`Robot` should have the same name
        virtual_transformations (List[Transformation]): A list of :py:class:`Transformation` objects forming a serial Kinematic chain. 
        actuated_state (List[Dict[str, float]], optional): The State of the Groups actuated joints. Defaults to None.
        actuated_to_virtual (Callable, optional): Maps the :py:attr:`actuated_state` to the :py:attr:`virtual_state` of the :py:attr:`virtual_transformations`. Defaults to None.
        virtual_to_actuated (Callable, optional): Maps the :py:attr:`virtual_state` of the :py:attr:`virtual_transformations` to the :py:attr:`actuated_state`.
        act_to_virt_args ([type], optional): Arguments that can be passed to :py:attr:`actuated_to_virtual` during the initial testing of the function. Defaults to None.
        virt_to_act_args ([type], optional): Arguments that can be passed to :py:attr:`virtual_to_actuated` during the initial testing of the function. Defaults to None.
        parent ([type], optional): [description]. Defaults to None.

    Raises:
        ValueError: 'Error: Actuated state is missing. You provided a mapping to actuate the group but no state to be actuated.' 
                    if there is no :py:attr:`actuated_state` despite a mapping being passed
        ValueError: 'Error: Only one mapping provided. You need mappings for both ways. Consider to pass a trivial mapping.'
                    if either :py:attr:`actuated_to_virtual` or :py:attr:`virtual_to_actuated` was not set despite providing a :py:attr:`actuated_state`.
        ValueError: 'Error: Mappings missing. You provided an actuated state but no mappings. If you want a trivial mapping you don\'t need to pass an actuated state. Trip will generate one for you.'
                    if both :py:attr:`actuated_to_virtual` and :py:attr:`virtual_to_actuated` were not set despite providing a :py:attr:`actuated_state`.
        RuntimeError: "actuated_to_virtual does not fit virtual state" if the :py:attr:`actuated_to_virtual` function does not return a valid :py:attr:`virtual_state` dictionary
        RuntimeError: "virtual_to_actuated does not fit actuated state" if the :py:attr:`virtual_to_actuated` function does not return a valid :py:attr:`actuated_state` dictionary

    """

    @staticmethod
    def object_list_to_key_lists(object_lst):
        return list(object_lst.keys())

    def __init__(self, name: str, virtual_transformations: List[Transformation], actuated_state: Dict[str, float], actuated_to_virtual: Callable, virtual_to_actuated: Callable, act_to_virt_args=None, virt_to_act_args=None, parent=None):
        self._name   = name
        self.children = []

        if parent == None:
            self.parent = name
        elif isinstance(parent,KinematicGroup):
            self.parent = str(parent)
            parent.add_children(name)
        else:
            raise TypeError("The parent of a group must be a another group object")

        self._virtual_transformations = {}
        virtual_state = {}
        for transformation in virtual_transformations:
            self._virtual_transformations[str(transformation)]=transformation
            if transformation.get_state() != {}:
                virtual_state[str(transformation)]=transformation.get_state()

        if virtual_state:
            if (actuated_to_virtual or virtual_to_actuated) and not actuated_state:
                raise ValueError(
                    'Error: Actuated state is missing. You provided a mapping to actuate the group but no state to be actuated.')
            if (actuated_to_virtual and not virtual_to_actuated) or (not actuated_to_virtual and virtual_to_actuated):
                raise ValueError(
                    'Error: Only one mapping provided. You need mappings for both ways. Consider to pass a trivial mapping.')
            if not (actuated_to_virtual or virtual_to_actuated):
                raise ValueError(
                    'Error: Mappings missing. You provided an actuated state but no mappings. If you want a trivial mapping you can use the OpenKinematicGroup.')
        else:
            if actuated_state or (actuated_to_virtual or virtual_to_actuated):
                raise ValueError('Error: Allthough the Group is static, a actuated state and or mapping was provided that is not a None object.')

        if actuated_state:
            self._original_actuated_to_virtual = actuated_to_virtual
            if act_to_virt_args:
                self._actuated_to_virtual = lambda state: actuated_to_virtual(state, *act_to_virt_args)
            self._actuated_to_virtual   = actuated_to_virtual
            actuated_to_virtual_to_check = self._actuated_to_virtual(actuated_state)
            
            if KinematicGroup.object_list_to_key_lists(actuated_to_virtual_to_check) != KinematicGroup.object_list_to_key_lists(virtual_state):
                raise RuntimeError("actuated_to_virtual does not fit virtual state")
            self._original_virtual_to_actuated = virtual_to_actuated
            if virt_to_act_args:
                self._virtual_to_actuated = lambda state: virtual_to_actuated(state, *virt_to_act_args)
            self._virtual_to_actuated   = virtual_to_actuated
            virtual_to_actuated_to_check = virtual_to_actuated(virtual_state)

            if KinematicGroup.object_list_to_key_lists(virtual_to_actuated_to_check) != KinematicGroup.object_list_to_key_lists(actuated_state):
                raise RuntimeError("virtual_to_actuated does not fit actuated state")

            # Check if inital values fit actuated_to_virtual's and virtual_to_actuated's calculated values. Only if actuated_state, actuated_to_virtual and virtual_to_actuated are passed

            for key in actuated_to_virtual_to_check.keys():
                state = actuated_to_virtual_to_check[key]  #TODO not list but dictionary!
                init_values_do_not_match = False
                for state_key in state.keys():
                    if state[state_key] != virtual_state[key][state_key]:
                        init_values_do_not_match = True
            if init_values_do_not_match:
                print("Calculated state values do not match given values! Using set_state() before forward_kinematics() or inverse_kinematics() is recommended.")
        
        
        self.virtual_state  = deepcopy(virtual_state)
        self.actuated_state = deepcopy(actuated_state)
        

    def set_virtual_state(self, state: Dict[str,Dict[str, float]]):
        """Sets the :py:attr:`virtual_state` of the Group and automatically updates the corresponding :py:attr:`actuated_state`

        Args:
            state (Dict[str,Dict[str, float]]): A dictionary containing the members of :py:attr:`virtual_state` that should be set. 
                                                The new values need to be valid state for the state of the joint.

        Raises:
            RuntimeError: "This is a static group! There is no state to be set" if all  :py:class:`Transformation` objects of :py:attr:`_virtual_transformations` are static. 
            ValueError: "Error: State not set! Keys do not match! Make sure that your state includes the same keys as your intial virtual transformations." 
                        if the state to set is not part of keys of :py:attr:`virtual_state`
        """

        if self.actuated_state == None:
            raise RuntimeError("This is a static group! There is no state to be set")

        if KinematicGroup.object_list_to_key_lists(self.virtual_state) == KinematicGroup.object_list_to_key_lists(state):
            for key in state.keys():
                self.virtual_state[key] = state[key]
            self._update_chain()
            self.actuated_state = self._virtual_to_actuated(self.virtual_state)  
        else:
            raise ValueError(
                "Error: State not set! Keys do not match! Make sure that your state includes the same keys as your intial virtual transformations.")

    def set_actuated_state(self, state: Dict[str, float]):
        """Sets the :py:attr:`actuated_state` of the Group and automatically updates the corresponding :py:attr:`virtual_state`

        Args:
            state (Dict[str, float]): A dictionary containing the members of :py:attr:`actuated_state` that should be set. 


        Raises:
            RuntimeError: "This is a static group! There is no state to be set" if all  :py:class:`Transformation` objects of :py:attr:`_virtual_transformations` are static. 
            ValueError: Error: State not set! Keys do not match! Make sure that your state includes the same keys as your intial virtual transformations." 
                        if the state to set is not part of keys of :py:attr:`actuated_state`
        """

        if self.actuated_state == None:
            raise RuntimeError(
                "This is a static group! There is no state to be set.")

        if KinematicGroup.object_list_to_key_lists(self.actuated_state) == KinematicGroup.object_list_to_key_lists(state):
            for key in state.keys():
                self.actuated_state[key] = state[key]
            new_virtual_state = self._actuated_to_virtual(self.actuated_state)
            for key in new_virtual_state.keys():
                self.virtual_state[key] = new_virtual_state[key]
            self._update_chain()
        else:
            raise ValueError(
                "Error: State not set! Keys do not match! Make sure that your state includes the same keys as the intial actuated state.")

    def __str__(self):
        return self._name

    def get_name(self):
        return deepcopy(self._name)

    def get_virtual_state(self):
        return deepcopy(self.virtual_state)

    def get_actuated_state(self):
        if self.actuated_state:
            return deepcopy(self.actuated_state)
        else:
            return None

    def get_transformation_matrix(self):
        """Calculates the full transformationmatrix from the start of the virtual chain to its endeffector.

        Returns:
            array: The homogenous transformation matrix from the start of the virtual chain to its endeffector.
        """

        # Identity matrix
        transformation = identity_transformation()
        transformations = self._virtual_transformations
        for key in transformations:
            part           = transformations[key]
            hmt            = part.get_transformation_matrix()
            transformation = transformation @ hmt

        return transformation

    def get_virtual_transformations(self):
        return deepcopy(self._virtual_transformations)

    def _update_chain(self):
        """propagates changes from the :py:attr:`virtual_state` to the underlying :py:class:`Transformation` objects.
        """
        for key in self.virtual_state.keys():
            self._virtual_transformations[key].set_state(self.virtual_state[key])

    def add_children(self, child: str):
        self.children.append(child)


    def pass_arguments_g(self, argv):
        g_map = self._original_virtual_to_actuated
        self._virtual_to_actuated = lambda state: g_map(state, *argv)

    def pass_arguments_f(self, argv):
        f_map = self._original_actuated_to_virtual
        self._actuated_to_virtual = lambda state: f_map(state, *argv)

class OpenKinematicGroup(KinematicGroup):
    def __init__(self, name: str, virtual_transformations: List[Transformation], parent=None):
        virtual_state      = {}
        virtual_trafo_dict = {}
        for transformation in virtual_transformations:
            virtual_trafo_dict[str(transformation)]=transformation
            if transformation.get_state() != {}:
                virtual_state[str(transformation)]=transformation.get_state()

        if any(virtual_state):
            # trivial mappings
            actuated_state_dummy = {}
            f_map = {}
            g_map = {}

            for virtual_key in virtual_trafo_dict.keys():

                transformation = virtual_trafo_dict[virtual_key]
                state = transformation.get_state()
                for key, value in state.items():
                    concat_key = transformation.get_name() + '_' + key
                    actuated_state_dummy.setdefault(concat_key, value)
                    f_map.setdefault(concat_key, (key, virtual_key))
                    g_map.setdefault((key, virtual_key), concat_key)

            # create trivial mappings
            def trivial_actuated_to_virtual(state):
                # Generate array
                out = {}
                for concat_key, value in state.items():
                    key, index = f_map[concat_key]
                    out[index] = {key:value}
                return out

            def trivial_virtual_to_actuated(states):
                out = {}
                for virtual_key in states.keys():
                    transformation = states[virtual_key]
                    for key in transformation.keys():
                        combined_key = (key, virtual_key)
                        new_key      = g_map[combined_key]
                        out[new_key] = states[virtual_key][key]
                return out

            actuated_to_virtual = trivial_actuated_to_virtual
            virtual_to_actuated = trivial_virtual_to_actuated
            actuated_state      = actuated_state_dummy
        else:   # This is a static group
            actuated_state = None
            actuated_to_virtual = None
            virtual_to_actuated = None
        super(OpenKinematicGroup,self).__init__(name = name, 
                                                virtual_transformations = virtual_transformations, 
                                                actuated_state = actuated_state, 
                                                actuated_to_virtual = actuated_to_virtual, 
                                                virtual_to_actuated = virtual_to_actuated,
                                                parent = parent)