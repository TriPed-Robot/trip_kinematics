from trip_kinematics.Joint import Joint, Constants, State
from math import radians
from trip_kinematics.OpenChain import OpenChain
from trip_kinematics.HomogenTransformationMartix import Homogenous_transformation_matrix

if __name__ == '__main__':

    # openchain
    gimbal_joint = State(alpha=0, beta=0, gamma=0)

    A_CCS_P = Joint("A_CCS_P", constants=Constants(x=0.265, z=0.014),
                    state=gimbal_joint)

    print(A_CCS_P.get_transformation())

    A_P_LL_Joint = Joint("A_P_LL_Joint,", constants=Constants(
        x=1.640, z=-0.037, beta=radians(-30)), state=State(beta=0), parents=[A_CCS_P])

    A_LL_Joint_FCS = Joint(
        "A_LL_Joint_FCS", constants=Constants(x=-1.5), state=State(), parents=[A_P_LL_Joint])

    def fun():
        pass

    open_chain = OpenChain([A_CCS_P, A_P_LL_Joint, A_LL_Joint_FCS], [fun])

    gimbal_joint.alpha = 1000
    A_CCS_P.change_value(gimbal_joint)
    print(A_CCS_P.get_transformation())

    print("ye")
