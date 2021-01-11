import platform
import os
import sys
import ac
import acsys
from sim_info import SimInfo
import idna
import time
import struct
import math

if platform.architecture()[0] == "64bit":
    libdir = 'lib64'
else:
    libdir = 'lib'
sys.path.insert(0, os.path.join(os.path.dirname(__file__), libdir))
os.environ['PATH'] = os.environ['PATH'] + ";."

import socket
import json
import copy
import traceback

info = SimInfo()
appName = "aclab"
width, height = 300 , 150

trackName=''
trackConfig=''
trackLength=None
ego_stat={}
env_stat={}
ai_lane={}
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(("127.0.0.1",4420))
s.setblocking(0)
client_stat = 'wait'
def acMain(ac_version):
    global appWindow, l_sockstat, ego_stat, env_stat, s, ai_lane, fileDest

    appWindow = ac.newApp(appName)
    ac.setTitle(appWindow, appName)
    ac.setSize(appWindow, width, height)

    global trackName
    trackName = ac.getTrackName(0) 
    
    global trackConfig
    trackConfig = ac.getTrackConfiguration(0)
    
    global trackLength
    trackLength = ac.getTrackLength(0)

    ego_stat = {
            'trackName' : trackName,
            'trackConfig' : trackConfig,
            'gas' : None,
            'brake' : None,
            'steerAngle' : None,
            'velocity' : [None,None,None],
            'accG' : [None,None,None],
            'wheelSlip' : [None,None,None,None],
            'wheelLoad' : [None,None,None,None],
            'heading' : None,
            'pitch' : None,
            'roll' : None,
            'localAngularVel' : [None,None,None],
            'localVelocity' : [None,None,None],
            'normalizedCarPosition' : None,
            'carCoordinates' : [None,None,None],
            'surfaceGrip' : None,
            'env_stat' : {}
            }
    env_stat = {
                'velocity' : [None,None,None],
                'normalizedCarPosition' : None,
                'carCoordinates' : [None,None,None]
                }
    
    fileDest = os.path.join(os.getcwd(),'content','tracks',trackName,'ai','fast_lane.ai')
    if not trackConfig == "" and os.path.isdir("content/tracks/%s/%s/ai"%(trackName, trackConfig)):
        fileDest = os.path.join(os.getcwd(),'content','tracks',trackName,trackConfig,'ai','fast_lane.ai')

    # ai_lane = acParse(fileDest)
    ac.console(fileDest)

    l_sockstat = ac.addLabel(appWindow, "waiting for client...")

    return appName


def acUpdate(deltaT):
    global s, addr, client_stat, ai_lane, fileDest
    try:
        recv = s.recvfrom(1024)
        client_stat = recv[0].decode().split('\n')[0]
        addr = recv[1]
        if client_stat == 'init':
            s.sendto((fileDest+'\n').encode(),addr)
        elif client_stat == 'query':
            s.sendto((json.dumps(ego_stat)+'\n').encode(),addr)
    except:
        # ac.console(client_stat)
        ego_stat['gas'] = info.physics.gas
        ego_stat['brake'] = info.physics.brake
        ego_stat['steerAngle'] = info.physics.steerAngle
        ego_stat['velocity'] = info.physics.velocity[:]
        ego_stat['accG'] = info.physics.accG[:]
        ego_stat['wheelSlip'] = info.physics.wheelSlip[:]
        ego_stat['wheelLoad'] = info.physics.wheelLoad[:]
        ego_stat['heading'] = info.physics.heading
        ego_stat['pitch'] = info.physics.pitch
        ego_stat['roll'] = info.physics.roll
        ego_stat['localAngularVel'] = info.physics.localAngularVel[:]
        ego_stat['localVelocity'] = info.physics.localVelocity[:]

        ego_stat['carCoordinates'] = info.graphics.carCoordinates[:]
        ego_stat['surfaceGrip'] = info.graphics.surfaceGrip
        ego_stat['numCars'] = info.static.numCars
        for car in range(info.static.numCars):
            if car is 0:
                continue
            env_stat['velocity'] = ac.getCarState(car, acsys.CS.Velocity)
            env_stat['normalizedCarPosition'] = ac.getCarState(car, acsys.CS.NormalizedSplinePosition)
            env_stat['carCoordinates'] = ac.getCarState(car, acsys.CS.WorldPosition)
            ego_stat['env_stat'][car] = copy.deepcopy(env_stat)
            # ac.console(json.dumps(ego_stat)+'\n')
    
    

def acParse(aiFile):
    with open(aiFile,'rb') as ai:
        header = struct.unpack('IIII', ai.read(16))
        lineLen = header[1]
        fmt = {}
        fmt['pathX'] = [0]*lineLen
        fmt['pathY'] = [0]*lineLen
        fmt['pathZ'] = [0]*lineLen
        fmt['travel'] = [0]*lineLen
        fmt['speed'] = [0]*lineLen
        fmt['accel'] = [0]*lineLen
        fmt['yawrate'] = [0]*lineLen
        fmt['radius'] = [0]*lineLen
        fmt['offLeft'] = [0]*lineLen
        fmt['offRight'] = [0]*lineLen
        fmt['nPidx'] = [0]*lineLen # Int
        fmt['direction'] = [0]*lineLen
        fmt['pathLeftX'] = [0]*lineLen
        fmt['pathLeftY'] = [0]*lineLen
        fmt['pathRightX'] = [0]*lineLen
        fmt['pathRightY'] = [0]*lineLen
        
        for i in range(lineLen):
            temp = struct.unpack('ffffI', ai.read(20))
            fmt['pathX'][i] = temp[0]
            fmt['pathZ'][i] = temp[1]
            fmt['pathY'][i] = temp[2]
            fmt['travel'][i] = temp[3]
            fmt['nPidx'][i] = temp[3]/trackLength

        for i in range(lineLen):
            temp = struct.unpack('f'*18, ai.read(72))
            fmt['speed'][i] = temp[1]
            fmt['accel'][i] = temp[2]
            fmt['yawrate'][i] = temp[4]
            fmt['radius'][i] = temp[5]
            fmt['offLeft'][i] = temp[6]
            fmt['offRight'][i] = temp[7]
            fmt['direction'][i] = math.atan2(temp[16], temp[14])
            fmt['pathLeftX'][i] = fmt['pathX'][i] + math.sin(fmt['direction'][i])*temp[6]
            fmt['pathLeftY'][i] = fmt['pathY'][i] - math.cos(fmt['direction'][i])*temp[6]
            fmt['pathRightX'][i] = fmt['pathX'][i] - math.sin(fmt['direction'][i])*temp[6]
            fmt['pathRightY'][i] = fmt['pathY'][i] + math.cos(fmt['direction'][i])*temp[6]
    ac.console('End Parsing AI lane')
    return fmt
    