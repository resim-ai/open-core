from __future__ import annotations
import numpy
import typing
__all__ = ['Quaternion']
class Quaternion:
    @typing.overload
    def __init__(self, vector: numpy.ndarray[numpy.float64[4, 1]]) -> None:
        ...
    @typing.overload
    def __init__(self, w: float, x: float, y: float, z: float) -> None:
        ...
    def w(self) -> float:
        ...
    def x(self) -> float:
        ...
    def y(self) -> float:
        ...
    def z(self) -> float:
        ...
