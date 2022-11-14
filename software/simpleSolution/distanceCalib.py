from enum import Enum
import scipy.interpolate as sp
import pandas as pd
import matplotlib.pyplot as plt
import time
import random
import numpy as np

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
    def __init__(self, filename, plottable = True) -> None:
        self.datapoints = [DataPoint(0, 0, 0), DataPoint(4200, 2400, 2400)]
        self.filename = filename
        self.plottable = plottable
        if self.plottable:
            self.figure = plt.figure("Calibrator")
            self.figure.canvas.mpl_connect('close_event', lambda ev: quit(0))
            plt.axis([0, 4500, 0, 2400])
            self.updateData()

    
    def updateData(self):
        x_data = [x.dist for x in self.datapoints]
        average = [(x.under + x.over) / 2 for x in self.datapoints]
        plt.clf()
        plt.plot(x_data, [x.under for x in self.datapoints], 'bo')
        plt.plot(x_data, [x.over for x in self.datapoints], 'go')
        plt.plot(x_data, average)
        plt.pause(0.05)
    
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

    def get_underpoint_index(self, distance: int):
        for i in range(len(self.datapoints)):
            if self.datapoints[i].dist >= distance:
                return i - 1
        raise ValueError("Something broken")


    def getEstimateBasedOnDistance(self, distance: int) -> int:
        i = self.get_underpoint_index(distance)
        # self.datapoints.sort(key=cmp_to_key(comparatorGenerator(distance)))
        underpoint = self.datapoints[i]
        overpoint = self.datapoints[i + 1]
        return sp.interp1d([underpoint.dist, overpoint.dist], [underpoint.under, overpoint.over])(distance), underpoint.under, overpoint.over

    def updateEstimateForDistance(self, distance:int, velocity:int, under:int, over:int, state: State) -> None:
        newData = None
        if state == State.TOO_FAR:
            newData = DataPoint(distance, under, velocity)
        elif state == State.TOO_SHORT:
            newData = DataPoint(distance, velocity, over)
        else:
            newData = DataPoint(distance, velocity, velocity)
        self.datapoints.append(newData)
        self.datapoints.sort(key= lambda x: x.dist)


def custom_sqrt(x):
    return ((x / 10) ** 0.5) * 100

if __name__ == "__main__":
    c = Calibrator("test.txt")# .read()

    k = 0
    while k < 20:
        k += 1
        print(k)
        curDist = random.random() * 4200

        speedGuess, under, over = c.getEstimateBasedOnDistance(curDist)
        actualSpeed = custom_sqrt(curDist)
        if (speedGuess < actualSpeed):
            c.updateEstimateForDistance(curDist, speedGuess, under, over, State.TOO_SHORT)
        elif speedGuess > actualSpeed:
            c.updateEstimateForDistance(curDist, speedGuess, under, over, State.TOO_FAR)
        else:
            c.updateEstimateForDistance(curDist, speedGuess, under, over, State.GOAL)
    
    # c.read()
    c.updateData()
    points_no = 10000
    data = np.linspace(0, 4200, points_no)
    values = np.zeros(points_no)
    prevPoints = []
    for i in range(len(data)):
        dist = data[i]
        print(i)
        val, _, _ = c.getEstimateBasedOnDistance(dist)
        prevPoints.append(val)
        if len(prevPoints) > 40:
            prevPoints.pop(0)
        avg = sum(prevPoints) / len(prevPoints)
        values[i] = avg
    #plt.clf()
    plt.plot(data, values, 'r')
    plt.pause(0.05)
    input("Done! >")
