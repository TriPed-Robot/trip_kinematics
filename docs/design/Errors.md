# Errors

    Class: TransformationParameters

        Description: TransformationParameters have mixed keys from euler and quaternion
        Msg: "State can't have euler angles and quaternions!"


        Description: State-keys given to constructor are not present in the given values
        Msg: "Key(s) from stateVariables not present inside values"


    Class: KinematicGroup

        Description: At initializing you have to give either actuated_state,f_mapping and g_mapping or neither.
        Msg: "For actuated states a forward and an inverse mapping are required."

        Description: The f_mapping has to return a state that fits the virtual_state
        Msg: "f_mapping does not fit virtual state"

        Description: The g_mapping hast to return a state that fits the actuated_state
        Msg:"g_mapping does not fit actuated state"

    at set_state

        Description the given state has to match either the virtual_state or the actuated_state
        Msg: "State does not match!"

        Class: TransformationMatrix

        Description: For the convention parameter there is something other specified than quat or any permutation of xyz
        Msg: "ConventionError: Expect x,y,z got: "

Warnings:

    Class: KinematicGroup

        Description: If the initalvalues of the virtual_state do not match the values calculated when first running the f_mapping
        Msg: "Calculated state values do not match given values! Using set_state() before forward_kinematics() or inverse_kinematics() is recommended"
