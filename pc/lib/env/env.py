import typing as tp
from dataclasses import dataclass
import numpy as np


@dataclass
class SensorData:
    data: tp.Any


class ImuData(SensorData):
    data: tp.Dict[str, float]


class ServoPositionData(SensorData):
    data: tp.Dict[int, float]


class CameraData(SensorData):
    data: np.ndarray


Observation = tp.Dict[str, SensorData]
Action = tp.Dict[str, tp.Tuple[float, float]]


class RobotEnv:
    def get_observation(self) -> Observation:
        raise NotImplementedError

    def step(self, action: Action) -> Observation:
        raise NotImplementedError

    def reset(self) -> Observation:
        raise NotImplementedError
