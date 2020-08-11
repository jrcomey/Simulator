#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug  9 16:02:38 2020

Vehicle Creator Program

author @JRC

"""

import numpy as np
import pandas as pd
import Simulation as sim
class VehicleCreator():
    
    def __init__(self):
        pass
    
    def SetVehicleProperties(self, file_name, vehicle_type = 'custom_rotary',\
                             vehicle_mass = 0,\
                             motor_positioning = np.array([0,0,0]),\
                             motor_type = 'T2206-2150',\
                             inertial_mat = np.array([0,0,0]),\
                             motor_mixer = np.array([0,0,0,0])):
        global column_basics
        column_basics = {"Mass" : vehicle_mass,
                         "Inertial Matrix" : inertial_mat,
                         "Category" : vehicle_type}
        
        self.df = pd.DataFrame(columns = column_basics)
        self.repository_file_name = file_name
        
        
        self.df.append(column_basics, ignore_index=True)
        
        if vehicle_type == 'custom_rotary':
            pass
        
        
        elif vehicle_type == 'quad_x':
            self.motor_positioning = motor_positioning
            for i in range(len(motor_positioning)):
                # column_name = f'Motor {i} Position'
                # self.df[column_name] = motor_positioning[i]
                print(motor_positioning[i])
        
        
        elif vehicle_type == 'quad_plus':
            pass
        elif vehicle_type == 'hex_circular':
            pass
        elif vehicle_type == 'hex_straight':
            pass
        elif vehicle_type == 'hex_Y':
            pass
        elif vehicle_type == 'octo_circular':
            pass
        elif vehicle_type == 'octo_straight':
            pass
        elif vehicle_type == 'octo_x':
            pass
        elif vehicle_type == 'octo_plus':
            pass
        elif vehicle_type == 'one_eng_fixed':
            pass
        elif vehicle_type == 'two_eng_fixed':
            pass
        elif vehicle_type == 'quad_eng_fixed':
            pass
        elif vehicle_type == 'one_eng_biplane':
            pass
        elif vehicle_type == 'two_eng_biplane':
            pass
    
    def ExportVehicleData(self):
        pass

class BodyCreator():
    def __init__(self):
        pass


def __main__():
    global creator
    creator = VehicleCreator()
    creator.SetVehicleProperties("TestDroneNo1.csv", vehicle_type="quad_x", 
                                 vehicle_mass=1, motor_positioning=np.array([[0.25, 0.25, 0],
                                                                             [0.25, -0.25, 0],
                                                                             [-0.25, 0.25, 0],
                                                                             [-0.25, -0.25, 0]]),
                                 inertial_mat=np.array([0.1, 0.2, 0.1]))
    creator.ExportVehicleData()
    
    
    
    
    
if __name__ == "__main__":
    __main__()