from trip.Joint import Joint


def dummy_function():
    pass


class OpenChain:

    def __init__(self):
        self.__joints = []
        self.__mapping = {'test': dummy_function}
        pass

    def forward(self):
        pass

    def inverse(self):
        pass

    def get_actuated_joints(self):
        pass

    def get_joints(self):
        pass
