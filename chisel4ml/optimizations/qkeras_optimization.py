from tensorflow.keras.layers import Layer as KerasLayer

from abc import ABC
from abc import abstractmethod
from typing import List


def _check_num_layers(call_fn):
    def wrap_call_fn(self, layers):
        assert len(layers) == self.num_layers, \
            f"The number of layers for the {self.__class__} optimizations should be {self.num_layers}. The provided" \
            f" number of layers to the optimizer was {len(layers)}."
        return call_fn(self, layers)
    return wrap_call_fn


class QKerasOptimization(ABC):
    num_layers: int

    def __init_subclass__(cls):
        """
            We decorate all child classes to check if the length of the tuple matches
            the num_layers variable.
        """
        cls.__call__ = _check_num_layers(cls.__call__)  # type:ignore

    @abstractmethod
    @_check_num_layers
    def __call__(self, layers: List[KerasLayer]) -> List[KerasLayer]:
        return []
