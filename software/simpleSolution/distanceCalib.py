from enum import Enum

class State(Enum):
    TOO_FAR = 0
    TOO_SHORT = 1
    GOAL = 2


class Calibrator:
    def __init__(self) -> None:
        pass


    def getEstimateBasedOnDistance(self, distance: int) -> int:
        return -1


    def updateEstimateForDistance(self, distance: int, state: State) -> None:
        pass