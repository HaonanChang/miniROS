from enum import Enum
from functools import total_ordering


@total_ordering
class OrderedEnum(Enum):
    """
    An enum that can be compared to other enums of the same type.
    """

    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value

    def __gt__(self, other):
        if self.__class__ is other.__class__:
            return self.value > other.value
