from libnaoqipy import *


'''
class QIData(object):
    def dumpFile(self, fname):
        pass


class QIImage(QIData):
    def __init__(self, width, height, layers, color_space=0, data=[]):
        self.width = width
        self.height = height
        self.layers = layers
        self.color_space = color_space
        self.camera_id = 0
        self.time_stamp = 0
        self.data = data
        self.data_numpy = np.array(data)

    def calcBufferSize(width, height, layers):
        pass

    def fromFile(self, fname):
        pass


class QIGazeAnalysisData(QIData):
    def __init__(self, looking_at_score, time_stamp):
        self.looking_at_score = looking_at_score
        self.time_stamp = time_stamp

    def isLookingAt(self):
        return self.looking_at_score > 0

    def calcBufferSize():
        pass


class QIClient(object):
    def openSharedMemory(self):
        pass


class QICameraClient(QIClient):
    def __init__(self, name):
        self.name = name

    def getImage(self):
        pass


class QIGazeAnalysisDataClient(QIClient):
    def __init__(self, name):
        self.name = name

    def getData(self):
        pass
'''
