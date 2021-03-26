
from trip.HomogenTransformationMartix import Homogenous_transformation_matrix


class Parameter:
    def __init__(self, x=0, y=0, z=0, alpha=0, beta=0, gamma=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma


class Joint:

    def __init__(self, name, parent, child, params, state):
        self.__name: str = name
        self.__parent = parent
        self.__child = child

        self.__parameter: Parameter = Parameter(**params)
        self.__state = []

    def get_transformation(self) -> Homogenous_transformation_matrix:

        tx = self.__parameter.x
        ty = self.__parameter.y
        tz = self.__parameter.z

        rx = self.__parameter.alpha
        ry = self.__parameter.beta
        rz = self.__parameter.gamma

        return Homogenous_transformation_matrix(tx=tx, ty=ty, tz=tz, rx=rx, ry=ry, rz=rz, conv='xyz')

    def change_value(self) -> None:
        pass

    def get_name(self) -> str:
        return self.__name
