#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug  9 17:44:56 2020

@author: jack
"""

from pymultiwii import MultiWii

serialPort = "/dev/ttyUSB1"
board = MultiWii(serialPort)
while True:
	print(board.getData(MultiWii.ATTITUDE))