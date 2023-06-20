#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import sim
import time
import math
import os

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *
import numpy as np

PROTOCOL_VERSION = 2
LEN_PRO_GOAL_POSITION = 4
LEN_PRO_TORQUE_ENABLE = 1

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# define ADDR_PRO_ACCELERATION_LIMIT     26
# define ADDR_PRO_TORQUE_LIMIT           30
ADDR_PRO_VELOCITY_LIMIT = 32
ADDR_PRO_TORQUE_ENABLE = 562
ADDR_PRO_POSITION_P_GAIN = 594
ADDR_PRO_GOAL_POSITION = 596
ADDR_PRO_GOAL_VELOCITY = 600
ADDR_PRO_PRESENT_POSITION = 611

# Communication Result
COMM_SUCCESS = 0  # tx or rx packet communication success
COMM_PORT_BUSY = -1000  # Port is busy (in use)
COMM_TX_FAIL = -1001  # Failed transmit instruction packet
COMM_RX_FAIL = -1002  # Failed get status packet
COMM_TX_ERROR = -2000  # Incorrect instruction packet
COMM_RX_WAITING = -3000  # Now recieving status packet
COMM_RX_TIMEOUT = -3001  # There is no status packet
COMM_RX_CORRUPT = -3002  # Incorrect status packet
COMM_NOT_AVAILABLE = -9000  #

DXL_MOVING_STATUS_THRESHOLD = 20

BAUDRATE = 1000000  # Dynamixel default baudrate : 57600
DEVICENAME = 'COM5'  # A MODIFIER EN FONCTION DU PORT COM DU ROBOT ROBOTIS

pi = math.pi

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address = ('localhost', 54001)
print('starting up on {} port {}'.format(*server_address))
sock.bind(server_address)


# ----------------------Functions---------------------
def InitDynamixel():
    # Set the port path
    portHandler = PortHandler(DEVICENAME)

    # Set the protocol version
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_GOAL_POSITION)
    connectionSucceded = False

    # Open port
    try:
        portHandler.openPort()
        print("Succeeded to open the port")
        connectionSucceded = True
    except:
        print("Failed to open the port")
        connectionSucceded = False

    if connectionSucceded == True:
        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    return connectionSucceded, portHandler, packetHandler, groupSyncWrite, groupSyncRead


def Codeur2Rad(id, codeur):
    if ((id == 1) or (id == 2) or (id == 3) or (id == 4)):
        rad = pi * codeur / 251000.0

    if ((id == 5) or (id == 6)):
        rad = pi * codeur / 151875.0

    return rad


def Rad2Codeur(id, angle):
    # print("angle :", angle)
    if (id == 1):
        if (angle > pi):
            angle = pi
        elif (angle < -pi):
            angle = -pi

    if (id == 2):
        if (angle > (pi / 2.0)):
            angle = pi / 2.0
        elif (angle < (-pi / 2.0)):
            angle = (-pi / 2.0)

    if (id == 3):
        if (angle > (pi * 0.75)):
            angle = pi * 0.75
        elif (angle < (-pi / 2.0)):
            angle = (-pi / 2.0)

    if (id == 4):
        if (angle > (pi)):
            angle = pi
        elif (angle < (-pi)):
            angle = (-pi)

    if (id == 5):
        if (angle > (pi / 2.0)):
            angle = pi / 2.0
        elif (angle < (-pi / 2.0)):
            angle = (-pi / 2.0)

    if (id == 6):
        if (angle > (pi)):
            angle = pi
        elif (angle < (-pi)):
            angle = (-pi)

    if ((id == 1) or (id == 2) or (id == 3) or (id == 4)):
        value32 = math.trunc(angle / pi * 251000.0)

    if ((id == 5) or (id == 6)):
        value32 = math.trunc(angle / pi * 151875.0)

    return value32


def SetGoalPos(packetHandler, portHandler, id, val):
    value = Rad2Codeur(id, val)

    packetHandler.write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION, value);


def SetGoalPosAll(packetHandler, portHandler, groupSyncWrite, groupSyncRead, q):
    for i in range(6):
        # z = q[i]
        # print("i :", i + 1, "  z:", z, "    - qi : ", q)

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(Rad2Codeur(i + 1, q[i]))),
                               DXL_HIBYTE(DXL_LOWORD(Rad2Codeur(i + 1, q[i]))),
                               DXL_LOBYTE(DXL_HIWORD(Rad2Codeur(i + 1, q[i]))),
                               DXL_HIBYTE(DXL_HIWORD(Rad2Codeur(i + 1, q[i])))]

        # print("  codeur :", Rad2Codeur(i + 1, q[i]))
        dxl_addparam_result = groupSyncWrite.addParam(i + 1, param_goal_position)
        if (dxl_addparam_result != 1):
            print(" groupSyncWrite addparam failed")
            quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if (dxl_comm_result != COMM_SUCCESS):
        # printTxRxResult(PROTOCOL_VERSION2, dxl_comm_result)
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("fail")

    groupSyncWrite.clearParam()


def TorqueOnAll(packetHandler, portHandler):
    for i in range(6):
        packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)


def TorqueOffAll(packetHandler, portHandler):
    for i in range(6):
        packetHandler.write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)


def init_vrep():
    listObjects_name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'ROBOTIQ_85_active1',
                        'ROBOTIQ_85_active2']
    listObjects_handle = [0, 0, 0, 0, 0, 0, 0, 0]

    sim.simxFinish(-1)
    # try :
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    #   connexionVREP = True
    # except :
    #   print("Connexion a VREP impossible")
    #  connexionVREP = False

    if clientID > -1:
        print('Connected to remote API server')
        sim.simxSynchronous(clientID, False)
        sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, 0.01, sim.simx_opmode_oneshot)
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

        print('Retrieving all the joint handles.......')
        # récupération des handles des articulations (joint1..joint6)
        for idobj in range(0, 8):
            print(listObjects_name[idobj])

        all_ok = True;
        for i in range(0, 8):
            [error, handle] = sim.simxGetObjectHandle(clientID, listObjects_name[i], sim.simx_opmode_oneshot_wait)
            all_ok = all_ok & (error == sim.simx_return_ok)
            listObjects_handle[i] = handle

        if all_ok == False:
            print('An error occured while retrieving the object handles ==> stop simulation')
    else:
        print("Connetion to VREP impossible")

    return clientID, listObjects_handle


def close_vrep(clientID):
    time.sleep(5);

    # Arrêt de la simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait);

    # Ferme la connexion réseau à VREP
    sim.simxFinish(-1);


def simulate(clientID, listObjects_handle, q):
    # Pause vrep
    # sim.simxPauseCommunication(clientID, 0);
    # envoie une consigne
    sim.simxSetJointTargetPosition(clientID, listObjects_handle[0], q[0], sim.simx_opmode_oneshot);
    sim.simxSetJointTargetPosition(clientID, listObjects_handle[1], q[1], sim.simx_opmode_oneshot);
    sim.simxSetJointTargetPosition(clientID, listObjects_handle[2], q[2], sim.simx_opmode_oneshot);
    sim.simxSetJointTargetPosition(clientID, listObjects_handle[3], q[3], sim.simx_opmode_oneshot);
    sim.simxSetJointTargetPosition(clientID, listObjects_handle[4], q[4], sim.simx_opmode_oneshot);
    sim.simxSetJointTargetPosition(clientID, listObjects_handle[5], q[5], sim.simx_opmode_oneshot);
    # [returncode, pos] = sim.simxGetJointPosition(clientID, listObjects_handle[0], sim.simx_opmode_blocking)
    # print(pos);

    # Active 1 pas de simulation :
    # sim.simxSynchronousTrigger(clientID)

    # réactive VREP et applique toutes les consignes simultanément
    # sim.simxPauseCommunication(clientID, 0)


def gripper(clientID, closing, listObjectHandle):
    [error, p1] = sim.simxGetJointPosition(clientID, listObjectHandle[6], sim.simx_opmode_blocking)
    [error, p2] = sim.simxGetJointPosition(clientID, listObjectHandle[7], sim.simx_opmode_blocking)

    if closing == 1:
        if p1 < (p2 - 0.008):
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[6], -0.01, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[7], -0.04, sim.simx_opmode_blocking)
        else:
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[6], -0.4, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[7], -0.4, sim.simx_opmode_blocking)
    else:
        if (p1 < p2):
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[6], 0.4, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[7], 0.2, sim.simx_opmode_blocking)
        else:
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[6], 0.2, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, listObjectHandle[7], 0.4, sim.simx_opmode_blocking)


# Wait for hand stabilisation over the LeapMotion and get coordonate : base = [Xhand,Yhand,Zhand]
def LeapStarter():
    cpt = 0
    test = True  # Test becomes False when hand is stabilizated
    base = [0, 0, 0]
    print('\nPlease have connected the Leap Motion and run the imageSample C program')
    print('\nPlace one hand over the Leap motion and dont move for few seconds')
    while test:
        # print('\nwaiting to receive message')
        data, address = sock.recvfrom(54000)
        data = data[:-4]
        C = data.split()  # C = [Xhand,Zhand,Yhand, grabbing percent]
        Dd = base  # save precedent pos
        base[0] = C[0]
        base[1] = C[2]
        base[2] = C[1]
        # If the hand is stable as +-5pt in leapMotion coordinate system
        if abs(float(Dd[0]) - float(base[0])) < 5 and abs(float(Dd[1]) - float(base[1])) < 5 and abs(
                float(Dd[2]) - float(base[2])) < 5:
            cpt = cpt + 1
        else:
            cpt = 0  # Reset counter if unstable
        # if the hand is stable during 20 loop
        if cpt > 20:
            test = False
    print('\nHand position validated')
    return base


# Get hand position from LeapMotion image sample program by socket
# Use base from LeapStarter function (create a coordinate system from your hand position)
# poseInitRob is the first robot position
# return the robot position calculated with the hand position
def getHandPos(base, poseInitRob):
    base[0] = float(base[0])
    base[1] = float(base[1])
    base[2] = float(base[2])
    # Limit robot pos in meter
    XMAX = 0.640
    XMIN = -0.640
    YMAX = 0.640
    YMIN = -0.640
    ZMAX = 0.676
    ZMIN = 0.05
    # Limit hand pos in Leap Motion coordinate system
    Lxmax = 250
    Lxmin = -250
    Lymax = 200
    Lymin = -200
    Lzmax = 500
    Lzmin = 0
    Pd = [0, 0, 0]
    # Reception of hand data
    data, address = sock.recvfrom(54000)
    data = data[:-4]
    C = data.split()  # C = [X,Z,Y, grabbingPercent]
    C[0] = float(C[0])
    C[1] = float(C[1])
    C[2] = float(C[2])

    # Transform hand position from Leap Motion coordinate system to meters
    # Pd = [X,Y,Z] real coordinate system
    # base = [X,Y,Z] Leap Motion coordinate system from first hand position
    Pd[0] = (XMAX / Lxmax) * (C[0] - base[0]) + poseInitRob[0]
    if Pd[0] > XMAX:
        Pd[0] = XMAX
    elif Pd[0] < XMIN:
        Pd[0] = XMIN

    Pd[1] = (YMAX / Lymax) * (C[2] - base[1]) + poseInitRob[1]
    if Pd[1] > YMAX:
        Pd[1] = YMAX
    elif Pd[1] < YMIN:
        Pd[1] = YMIN

    Pd[2] = (ZMAX / Lzmax) * (C[1] - base[2]) + poseInitRob[2]
    if Pd[2] > ZMAX:
        Pd[2] = ZMAX
    elif Pd[2] < ZMIN:
        Pd[2] = ZMIN

    grabbingPercent = float(C[3])
    return Pd, grabbingPercent


# MGI simplified to control 4Dof robot
def MGI(Pd):
    qi = np.array([0, -pi / 4, pi / 4, 0, pi / 4, 0])  # Pd de départ tiré du SDK robotis
    x = math.sqrt(Pd[0] ** 2 + Pd[1] ** 2) - 0.123
    qi[0] = math.acos((Pd[0] / (x + 0.123)))
    thet3 = (x ** 2 + (-Pd[2] + 0.159) ** 2 - (l1 ** 2 + l2 ** 2)) / (2 * l1 * l2)
    if thet3 > 1:
        thet3 = 0
    else:
        thet3 = math.acos(thet3)

    qi[2] = thet3 - 0.5583647993
    qi[1] = -math.acos(((l1 + l2 * math.cos(thet3)) * x + l2 * math.sin(thet3) * (-Pd[2] + 0.159)) / (
            x ** 2 + (-Pd[2] + 0.159) ** 2)) + 1.4576453
    qi[4] = (-qi[1] - qi[2] + 2.3561944) - pi / 2

    # q limit verification from Robotis SDK
    if qi[0] < -pi:
        qi[0] = -pi
    elif qi[0] > pi:
        qi[0] = pi
    elif qi[1] < -pi / 2:
        qi[1] = -pi / 2
    elif qi[1] > pi / 2:
        qi[1] = pi / 2
    elif qi[2] < -pi / 2:
        qi[2] = -pi / 2
    elif qi[2] > 3 * pi / 4:
        qi[2] = 3 * pi / 4
    elif qi[3] < -pi:
        qi[3] = -pi
    elif qi[3] > pi:
        qi[3] = pi
    elif qi[4] < -pi / 2:
        qi[4] = -pi / 2
    elif qi[4] > pi / 2:
        qi[4] = pi / 2
    elif qi[5] < -pi:
        qi[5] = -pi
    elif qi[5] > pi:
        qi[5] = pi
    return qi


# ----------------------Main Program---------------------
if __name__ == "__main__":
    l1 = 0.26569
    l2 = 0.25974
    l3 = 0.123
    [clientID, listObjects_handle] = init_vrep()
    #qi = [0, -pi / 4, pi / 4, 0, pi / 4, 0]
    # Robot position init calculated by Matlab with qi (see above)
    poseInitRob = [0.230351383121998, 4.97453992535804e-18, 0.570759597806951]
    PdPrime = [0, 0, 0]
    Pd = poseInitRob
    qi = MGI(Pd)
    simulate(clientID, listObjects_handle, qi)
    gripper(clientID, 0, listObjects_handle)
    [connectionSucceded, portHandler, packetHandler, groupSyncWrite, groupSyncRead] = InitDynamixel()
    if connectionSucceded == True:
        TorqueOnAll(packetHandler, portHandler)
        SetGoalPosAll(packetHandler, portHandler, groupSyncWrite, groupSyncRead, qi)
    base = LeapStarter()
    # print('\nInit Position in Leap coordinate system : X = ', base[0], '; Y = ', base[1], '; Z = ', base[2])
    gripperClose = False

    while (True):
        PdPrime = Pd
        [Pd, grabbingPercent] = getHandPos(base, poseInitRob)
        # print('\nReachable Position : X = ',Pd[0],'; Y = ', Pd[1],'; Z = ',Pd[2])
        # print("\nGrabbingPercent = ",grabbingPercent)
        # print('\nPdiffX = ', abs(Pd[0] - PdPrime[0]), '; PdiffY = ', abs(Pd[1] - PdPrime[1]), '; PdiffZ = ', abs(Pd[2] - PdPrime[2]))

        #   Filters out shaking
        if abs(Pd[0] - PdPrime[0]) > 0.0001 and abs(Pd[1] - PdPrime[1]) > 0.0001 and abs(Pd[2] - PdPrime[2]) > 0.0001:
            qi = MGI(Pd)
            # control vrep
            simulate(clientID, listObjects_handle, qi)
            # control robot
            if connectionSucceded == True:
                TorqueOnAll(packetHandler, portHandler)
                SetGoalPosAll(packetHandler, portHandler, groupSyncWrite, groupSyncRead, qi)
        # Close Gripper
        if grabbingPercent > 0.45 and gripperClose == False:
            gripper(clientID, 1, listObjects_handle)
            gripperClose = True
        # Open Gripper
        elif grabbingPercent < 0.45 and gripperClose == True:
            gripper(clientID, 0, listObjects_handle)
            gripperClose = False

    close_vrep(clientID)
