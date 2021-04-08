from typing import Dict, List, Callable


class Group:
    """[summary]
    """

    def __init__(self, name: str, params: ParameterSpecification, set_state: Callable, get_state: Callable, parents: List = []) -> None:
        """[summary]

        Args:
            name (str): [description]
            state (State): [description]
            set_state (Callable): [description]
            get_state (Callable): [description]
            parents (List[Group], optional): [description]. Defaults to [].
        """        """[summary]

        Args:
            name (str): [description]
            state (State): [description]
            parents (List[Group], optional): [description]. Defaults to [].
        """
        self.__name: str = name
        self.__parents: List[Group] = parents
        if len(parents) != 0:
            for parent in parents:
                parent.__add_child(self)

        self.__children: List[Group] = []
        self.__state: ParameterSpecification = params
        self.set_state: Callable = set_state
        self.get_state: Callable = get_state

    def get_name(self) -> str:
        """[summary]

        Returns:
            str: [description]
        """
        return self.__name

    def __add_child(self, child) -> None:
        """[summary]

        Args:
            child (Group): [description]
        """
        self.__children.append(child)
