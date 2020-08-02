#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 29 13:04:17 2020

Simulator UI Code

@author: jack
"""
#%%###########################
import wx
import numpy as np
import pandas as pd
import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
import GrapherCode as Gc

#%%###########################

class SimFrame(wx.Frame):
    def __init__(self):
        super().__init__(parent=None,title="UAV Simulation Frame")
        self.UIINIT()
    def UIINIT(self):
        self.repository_name = 'last_test.csv'
        
        self.sizer1 = wx.BoxSizer(wx.VERTICAL)
        self.sizer2 = wx.BoxSizer(wx.VERTICAL)
        
        
        self.sp = wx.SplitterWindow(self)
        self.p1 = wx.Panel(self.sp, style = wx.SUNKEN_BORDER)
        self.p2 = wx.Panel(self.sp, style = wx.SUNKEN_BORDER)
        self.sp.SplitVertically(self.p1, self.p2, 100)
        self.statusbar = self.CreateStatusBar()
        self.statusbar.SetStatusText("UI INITIALIZING")
        self.plotter = Gc.Plotter(self.repository_name)
        
        # self.figure = plotter.ShowSignal()
        # self.figure = matplotlib.figure.Figure()
        # self.axes = self.figure.add_subplot(111)
        # self.df = pd.read_csv("last_test.csv")
        # self.axes.plot(self.df["Time"], self.df["X Position"])
        self.canvas = FigureCanvas(self.p1, -1, self.plotter.ShowSignal())
        self.no_button = wx.Button(self.p2, -1, "Graph Position", pos = (10,10))
        self.no_button.Bind(wx.EVT_BUTTON, self.GraphPosition)
        self.statusbar.SetStatusText("UI INITIALIZED")
        
        self.sizer1.Add(self.canvas, 0, wx.ALL|wx.EXPAND, 5)
        self.sizer2.Add(self.no_button, 0, wx.ALL|wx.CENTER, 5)
        self.p1.SetSizer(self.sizer1)
        self.p2.SetSizer(self.sizer2)
    
    def RefreshGraph(self):
        self.sizer1.Clear()
        self.sizer1.Add(self.canvas, 0, wx.ALL|wx.EXPAND, 5)
        self.p1.SetSizer(self.sizer1)
        self.Refresh()
        self.Show()
        
    def GraphPosition(self, event):
        self.plotter.RefreshData(self.repository_name)
        self.canvas = FigureCanvas(self.p1, -1, self.plotter.ShowPosition())
        self.RefreshGraph()
        self.statusbar.SetStatusText("Position Graphed")
        
    
#%%###########################
# def plothusly(ax, x, y, xtitle = '', ytitle ='', \
#               datalabel = '', title=''):
#     """
#     A little function to make graphing less of a pain.
#     Creates a plot with titles and axis labels.
#     Adds a new line to a blank figure and labels it.

#     Parameters
#     ----------
#     ax : The graph object
#     x : X axis data
#     y : Y axis data
#     xtitle : Optional x axis data title. The default is ''.
#     ytitle : Optional y axis data title. The default is ''.
#     datalabel : Optional label for data. The default is ''.
#     title : Graph Title. The default is ''.

#     Returns
#     -------
#     out : Resultant graph.

#     """
#     """
    
#     """
    
#     ax.set_xlabel(xtitle)
#     ax.set_ylabel(ytitle)
#     ax.set_title(title)
#     out = ax.plot(x, y,zorder=1, label=datalabel)
#     return out

# def plothus(ax, x, y, datalabel = ''):
#     """
#     A little function to make graphing less of a pain
    
#     Adds a new line to a blank figure and labels it
#     """
#     out = ax.plot(x, y, zorder=1, label=datalabel)
#     return out
            
#%%###########################    
if __name__ == '__main__':
    app = wx.App(redirect=False)
    frame = SimFrame()
    frame.Show()
    app.MainLoop()