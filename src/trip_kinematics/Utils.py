from trip_kinematics.KinematicChainPart import KinematicChainPart
from typing import List


def sort_kinematic_chain_parts(lst_of_parts: List[KinematicChainPart]):
    sorted_chain = []
    for part in lst_of_parts:
        if part.get_parent() == None:
            sorted_chain.append(part)

    if len(sorted_chain) > 1:
        raise RuntimeError("To many loose ends inside chain.")

    buffer = sorted_chain[0]

    while buffer.get_child() != None:
        sorted_chain.append(buffer)
        buffer = buffer.get_child()
    return sorted_chain
