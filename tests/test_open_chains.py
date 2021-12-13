import trip_kinematics as trip
from trip_robots.triped import triped


def test_open_chain_builder():
    first_trafo = trip.Transformation(name='first_trafo',
                                      values={'tx': 0},
                                      state_variables=['tx'])
    second_trafo = trip.Transformation(name='second_trafo',
                                       parent=first_trafo,
                                       values={'ty': 0},
                                       state_variables=['ty'])
    third_trafo = trip.Transformation(name='third_trafo',
                                      parent=second_trafo,
                                      values={'rx': 0},
                                      state_variables=['rx'])
    fourth_trafo = trip.Transformation(name='fourth_trafo',
                                       parent=second_trafo,
                                       values={'qw': 0},
                                       state_variables=['qw'])

    transformations = [first_trafo, second_trafo, third_trafo, fourth_trafo]
    robot = trip.Robot(transformations)

    expected_virtual_state = {'first_trafo': {'tx': 0},
                              'second_trafo': {'ty': 0},
                              'third_trafo': {'rx': 0},
                              'fourth_trafo': {'qw': 0}}

    expected_actuated_state = {'first_trafo_tx': 0,
                               'second_trafo_ty': 0,
                               'third_trafo_rx': 0,
                               'fourth_trafo_qw': 0}

    virtual_as_expceted = robot.get_virtual_state() == expected_virtual_state
    actuated_as_expected = robot.get_actuated_state() == expected_actuated_state
    assert virtual_as_expceted and actuated_as_expected


def test_child_relation():
    groups = triped.get_groups()
    children_list = []
    for group in groups.values():
        children_list.append(group.children)

    expected_children = [['leg_0_A_P_LL'],
                         ['leg_0_A_LL_Joint_FCS'],
                         [],
                         ['leg_0_zero_angle_convention'],
                         ['leg_0_extend_joint'],
                         ['leg_1_A_P_LL'],
                         ['leg_1_A_LL_Joint_FCS'],
                         [],
                         ['leg_1_zero_angle_convention'],
                         ['leg_1_extend_joint'],
                         ['leg_2_A_P_LL'],
                         ['leg_2_A_LL_Joint_FCS'],
                         [],
                         ['leg_2_zero_angle_convention'],
                         ['leg_2_extend_joint']]

    assert sorted(expected_children) == sorted(children_list)
