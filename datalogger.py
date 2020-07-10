import matplotlib.pyplot as plt
import numpy as np
class datalogger:
    def __init__(self):
        self.x = []
        self.y = []
        self.capture = False
    def addpoint(self,x,y):
        self.x.append(x)
        self.y.append(y)
        self.capture = False
    def createPlot(self):
        plt.figure(figsize = (16,4))
        plt.plot(self.x,self.y)
        plt.show()




