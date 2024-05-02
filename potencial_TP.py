import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import time
from mpl_toolkits.mplot3d import Axes3D
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def readSensorData(
                    range_data_signal_id="hokuyo_range_data", 
                    angle_data_signal_id="hokuyo_angle_data"):

    string_range_data = sim.getStringSignal(range_data_signal_id)
    string_angle_data = sim.getStringSignal(angle_data_signal_id)
    raw_range_data = sim.unpackFloatTable(string_range_data)
    raw_angle_data = sim.unpackFloatTable(string_angle_data)

    return raw_range_data, raw_angle_data

def Rz(theta):
    return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                     [ np.sin(theta), np.cos(theta) , 0 ],
                     [ 0            , 0             , 1 ]])

# Normalize angle to the range [-pi,pi)
def normalizeAngle(angle):
    return np.mod(angle+np.pi, 2*np.pi) - np.pi

def att_force(q, goal, katt=0.1):
    return katt*(goal - q)

def rep_force(q, obs, R=3, krep=.1):
    
    # Obstáculo: (x, y, r)

    v = q - obs
    d = np.linalg.norm(v)
    
    rep = (1/d**2)*((1/d)-(1/R))*(v/d)    
    
    invalid = np.squeeze(d > R)
    rep[invalid, :] = 0
    
    return krep*rep

client = RemoteAPIClient()
sim = client.require('sim')

sim.startSimulation() 

# Handle para os dados do LASER e do Robô
robotname = './Pioneer_p3dx'
robotname_R_Motor = './Pioneer_p3dx/Pioneer_p3dx_rightMotor'
robotname_L_Motor = './Pioneer_p3dx/Pioneer_p3dx_leftMotor'
laser_range_data = "hokuyo_range_data"
laser_angle_data = "hokuyo_angle_data"

robotHandle = sim.getObjectHandle(robotname)    

robotLeftMotorHandle  = sim.getObjectHandle(robotname_L_Motor)
robotRightMotorHandle = sim.getObjectHandle(robotname_R_Motor)

robotPos = sim.getObjectPosition(robotHandle, sim.handle_world)
robotOri = sim.getObjectOrientation(robotHandle, sim.handle_world)

goalFrame = sim.getObjectHandle('./ReferenceFrame')

position_goal = sim.getObjectPosition(goalFrame, sim.handle_world)
orientation_goal = sim.getObjectOrientation(goalFrame, sim.handle_world) 

sim.setObjectPosition(goalFrame, [position_goal[0], position_goal[1], 0], sim.handle_world)
sim.setObjectOrientation(goalFrame, [orientation_goal[0], orientation_goal[1], orientation_goal[2]], sim.handle_world) 

L = 0.381
r = 0.0975
maxv = 1.0
maxw = np.deg2rad(45)

rho = np.inf

while True :
        
    while rho > .05:

        robotPos = sim.getObjectPosition(robotHandle, sim.handle_world)
        robotOri = sim.getObjectOrientation(robotHandle, sim.handle_world)        
        robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])        
        dx, dy = position_goal[:2] - robotConfig[:2]
        rho = np.sqrt(dx**2 + dy**2)

        position_data1, laser_data1 = readSensorData(laser_range_data, laser_angle_data)
        laser_data = np.array([position_data1, laser_data1]).T
        distLaser = []
        força_AtracTot = 0.0
        força_RepulsTot = 0.0

        for i in laser_data:
            f1 = 0
            f2 = 0
            d1 = i[0]  # Correção aqui
            ang1 = i[1]  # Correção aqui

            x = d1 * np.cos(ang1)
            y = d1 * np.sin(ang1)
            distLaser = np.array([x ,y, 0])
            f1 = att_force(distLaser, position_goal,0.1)
            f2 = rep_force(position_goal, distLaser, 3, 0.1)
            força_AtracTot += f1
            força_RepulsTot += f2

        
        frente = int(len(laser_data)/2)
        lado_direito = int(len(laser_data)*1/4) 
        lado_esquerdo = int(len(laser_data)*3/4)

        if laser_data[frente, 1] > 2:
            v1 = força_AtracTot + força_RepulsTot
            v = v1
            w = 0
        elif laser_data[lado_direito, 1] > 2:
            v = 0
            w = np.deg2rad(-30)
        elif laser_data[lado_esquerdo, 1] > 2:
            v = 0
            w = np.deg2rad(30)

        alpha = normalizeAngle(-robotConfig[2] + np.arctan2(dy,dx))
        beta = normalizeAngle(orientation_goal[2] - np.arctan2(dy,dx))
        
        kr = 1
        kt = 2
        
        v = kr*(dx*np.cos(robotConfig[2]) + dy*np.sin(robotConfig[2]))
        w = kt*(np.arctan2(dy,dx) - robotConfig[2])
                
        # Limit v,w to +/- max
        v = max(min(v, maxv), -maxv)
        w = max(min(w, maxw), -maxw)        
        
        vr = ((2.0*v) + (w*L))/(2.0*r)
        vl = ((2.0*v) - (w*L))/(2.0*r)
        
        sim.setJointTargetVelocity(robotRightMotorHandle, vr)
        sim.setJointTargetVelocity(robotLeftMotorHandle, vl)

    break

sim.setJointTargetVelocity(robotRightMotorHandle, 0)
sim.setJointTargetVelocity(robotLeftMotorHandle, 0)