from enum import Enum
import scipy as sp
import pandas as pd


class State(Enum):
    TOO_FAR = 0
    TOO_SHORT = 1
    GOAL = 2


class DataPoint:
    def __init__(self, dist, under, over):
        self.dist = dist
        self.under = under
        self.over = over
    
    def __str__(self):
        return f"{{dist: {self.dist}, under: {self.under}, over: {self.over}}}"


def getEstimate(dist, p1, p2):
    if (p1.dist == p2.dist):
        return 0
    elif p1.dist >= dist and p2.dist < dist:
        return -1
    elif p1.dist < dist and p2.dist >= dist:
        return 1
    elif (p1.dist >= dist and p2.dist >= dist) or (p1.dist < dist and p2.dist < dist):
        return p1.dist - p2.dist
    else:
        raise ValueError("Katki")


def comparatorGenerator(distance):
    return lambda p1, p2: getEstimate(distance, p1, p2)


def cmp_to_key(mycmp):
    'Convert a cmp= function into a key= function'
    class K(object):
        def __init__(self, obj, *args):
            self.obj = obj
        def __lt__(self, other):
            return mycmp(self.obj, other.obj) < 0
        def __gt__(self, other):
            return mycmp(self.obj, other.obj) > 0
        def __eq__(self, other):
            return mycmp(self.obj, other.obj) == 0
        def __le__(self, other):
            return mycmp(self.obj, other.obj) <= 0
        def __ge__(self, other):
            return mycmp(self.obj, other.obj) >= 0
        def __ne__(self, other):
            return mycmp(self.obj, other.obj) != 0
    return K


class Calibrator:
    def __init__(self, filename) -> None:
        self.datapoints = [DataPoint(0, 0, None), DataPoint(4200, 2400, 2400)]
        self.filename = filename


    def read(self) -> None:
        df = pd.read_csv(self.filename)
        self.datapoints = []
        for i in range(len(df['dist'])):
            self.datapoints.append(DataPoint(df['dist'][i], df['under'][i], df['over'][i]))

    
    def write(self) -> None:
        df = pd.DataFrame()
        for dp in self.datapoints:
            newData = pd.DataFrame({"dist": [dp.dist], "under": [dp.under], "over": [dp.over]})
            df = pd.concat([df, newData], ignore_index = True)
        df.to_csv(self.filename, index=False)


    def getEstimateBasedOnDistance(self, distance: int) -> int:
        self.datapoints.sort(key=cmp_to_key(comparatorGenerator(distance)))
        underpoint = self.datapoints[-1]
        overpoint = self.datapoints[0]
        return sp.interp1d([underpoint.dist, overpoint.dist], [underpoint.under, overpoint.over])(distance)



    def updateEstimateForDistance(self, distance:int, velocity:int, under:int, over:int, state: State) -> None:
        newData = None
        if state == State.TOO_FAR:
            newData = DataPoint(distance, under, velocity)
        elif state == State.TOO_SHORT:
            newData = DataPoint(distance, velocity, over)
        else:
            newData = DataPoint(distance, velocity, velocity)
        self.datapoints.append(newData)


if __name__ == "__main__":
    Calibrator("test.txt").read()
