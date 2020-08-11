#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 24 14:29:37 2020

Graphing and Analysis code

@author: jack
"""
#%%###########################

import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.animation as mpla
import numpy as np
#%%###########################


def plothusly(ax, x, y, xtitle = '', ytitle ='', \
              datalabel = '', title=''):
    """
    A little function to make graphing less of a pain.
    Creates a plot with titles and axis labels.
    Adds a new line to a blank figure and labels it.

    Parameters
    ----------
    ax : The graph object
    x : X axis data
    y : Y axis data
    xtitle : Optional x axis data title. The default is ''.
    ytitle : Optional y axis data title. The default is ''.
    datalabel : Optional label for data. The default is ''.
    title : Graph Title. The default is ''.

    Returns
    -------
    out : Resultant graph.

    """
    """
    
    """
    
    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_title(title)
    out = ax.plot(x, y,zorder=1, label=datalabel)
    return out

def plothus(ax, x, y, datalabel = ''):
    """
    A little function to make graphing less of a pain
    
    Adds a new line to a blank figure and labels it
    """
    out = ax.plot(x, y, zorder=1, label=datalabel)
    return out

#%%###########################


class Plotter():
    def __init__(self, repository_name):
        self.repository_name = repository_name
        self.df = pd.read_csv(repository_name)
    
    def RefreshData(self):
        self.df = pd.read_csv(self.repository_name)

    def ShowPosition(self):
        
        figure = matplotlib.figure.Figure()
        zplot = figure.add_subplot(111)
        plothusly(zplot, self.df["Time"], -1*self.df["Z Position"], "Time in seconds",\
                  "Position in metres", "Z Position", "Drone Position")
        plothus(zplot, self.df["Time"], self.df["Y Position"], "Y Position")
        plothus(zplot, self.df["Time"], self.df["X Position"], "X Position")
        plt.grid()
        plt.legend(loc="best")
        return figure

    def Show3DPosition(self):

        fig = plt.figure()
        threedplot = fig.add_subplot(111, projection='3d')
        threedplot.plot(self.df["X Position"], self.df["Y Position"], -1*self.df["Z Position"])
        # threedplot.set_xlim(-3, 3)
        # threedplot.set_ylim(-3, 3)
        # threedplot.set_zlim(-10, 0)
        threedplot.set_xlabel('X Position')
        threedplot.set_ylabel('Y Position')
        threedplot.set_zlabel('Z Position')
        return fig

    def ShowVelocity(self):
        figure = matplotlib.figure.Figure()
        velplot = figure.add_subplot(111)
        plothusly(velplot, self.df["Time"], self.df["Z Velocity"], "Time in seconds",\
                      "Z velocity in metres/s", "Drone 1", "Z Velocity")
        return figure

    def ShowAcceleration(self):
        figure = matplotlib.figure.Figure()
        accplot = figure.add_subplot(111)
        plothusly(accplot, self.df["Time"], self.df["Z Acceleration"], "Time in seconds",\
                  "Z velocity in metres/s", "Drone 1", "Z Acceleration")
        return figure

    def ShowSignal(self):
        
        figure = matplotlib.figure.Figure()
        signalplot = figure.add_subplot(111)
        
        plothusly(signalplot, self.df["Time"], self.df["Motor 0 Signal"], "Time",\
                  "Motor Signal", "Motor 0", "Motor Signals")
        plothus(signalplot, self.df["Time"], self.df["Motor 1 Signal"], "Motor 1")
        plothus(signalplot, self.df["Time"], self.df["Motor 2 Signal"], "Motor 2")
        plothus(signalplot, self.df["Time"], self.df["Motor 3 Signal"], "Motor 3")
        plt.grid()
        plt.legend(loc="best")
        return figure

    def ShowEuler(self):

        figure = matplotlib.figure.Figure()
        angleplot = figure.add_subplot(111)
        plothusly(angleplot, self.df["Time"], self.df["Pitch"], "Time in seconds", \
                  "Angle from neutral position in radians", "Pitch", "Euler angle plot")
        plothus(angleplot, self.df["Time"], self.df["Yaw"], "Yaw")
        plothus(angleplot, self.df["Time"], self.df["Roll"], "Roll")
        plt.grid()
        plt.legend(loc="best")
        return figure

    def GraphAll(self):
        self.RefreshData()
        self.ShowPosition()
        self.ShowVelocity()
        self.ShowAcceleration()
        self.ShowSignal()
        self.ShowEuler()
if __name__== '__main__':
    p = Plotter('last_test.csv')
    p.GraphAll()
#%%###########################

# T = np.linspace(0, 2*np.pi, 100)
# S = np.sin(T)
# line, = plt.plot(T,S)
# def animate(i):
#     line.set_ydata(np.sin(T+i/50))
# anim = mpla.FuncAnimation(
#     plt.gcf(), animate, interval=5)
# plt.show()