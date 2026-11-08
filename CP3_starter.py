import pybullet as p
import pybullet_data
import numpy as np
import time, math


REST_POSES = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 1.32, 0.02, 0.02] # Values from pybullet panda example, otherwise it's trial and error
HOME_X = 0
HOME_Y = 0.5
HOME_Z = 0.5

### Setup Pybullet ###

def setup_pybullet():
    # We don't use this but we need it.
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=-90, cameraPitch=-20, cameraTargetPosition=[0.5, 0.2, 1.0])
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    pandaId = p.loadURDF("franka_panda/panda.urdf", startPos, startOrientation, useFixedBase=1)
    blackboard_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 1, 0.5]),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 1, 0.5], rgbaColor=[0, 0, 0, 1]),
        basePosition=[0.5, 0.5, 0.5]
    )
    # Note: the blackboard is located at x=0.5, y=0.5

    return pandaId


### Panda ###

class Panda:
    """
    This class sets up your manipulator. It includes one built in function draw_on_blackboard.
    You are free to add any additional functions if you'd like.
    """
    def __init__(self, pandaId):
        self.pandaId = pandaId

    def get_cart(self):
        end_effector_index = 11
        link_state = p.getLinkState(self.pandaId, end_effector_index, computeForwardKinematics=True)
        # Position andd orientation of the link in the base frame.
        link_position = link_state[0]
        tip_position = list(link_position)
        print(tip_position)
        return tip_position

    def draw_on_blackboard(self):
        """
        This function draws a point in 3D space at the current end-effector of the robot
        """
        end_effector_index = 11
        link_state = p.getLinkState(self.pandaId, end_effector_index, computeForwardKinematics=True)
        # Position andd orientation of the link in the base frame.
        link_position = link_state[0]
        tip_position = list(link_position)
        tip_position[0] -=0.01 # in case they solve directly on black board, help this display
        vid = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[255, 255, 255, 1])
        p.createMultiBody(baseVisualShapeIndex=vid, basePosition=tip_position)

    def move_arm(self, pid, point):
        lower_limits = [-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671, 0.0, 0.0, 0.0, 0.0]
        upper_limits = [2.9671, 1.8326, 2.9671, 0.0, 2.9671, 3.8223, 2.9671, -1.0, -1.0, 0.04, 0.04, -1.0] # Found in getJointInfo
        move_joints = []
        ll = []
        ul = []
        joint_ranges = []

        for i in range(p.getNumJoints(pid)):
            joint = p.getJointInfo(pid, i)
            joint_type = joint[2]
            if joint_type != p.JOINT_FIXED:
                ll.append(joint[8])
                ul.append(joint[9])
                joint_ranges.append(joint[9] - joint[8])
                move_joints.append(i)

        
        target_orientation = p.getQuaternionFromEuler([np.deg2rad(0), np.deg2rad(90), np.deg2rad(0)])
        joint_angles = p.calculateInverseKinematics(pid, 11, point, lowerLimits=ll, upperLimits=ul, restPoses=REST_POSES, jointRanges=joint_ranges, targetOrientation=target_orientation, maxNumIterations=100, residualThreshold=1e-4)

        p.setJointMotorControlArray(
            bodyIndex = pid,
            jointIndices = move_joints,
            controlMode = p.POSITION_CONTROL,
            targetPositions = joint_angles,
            forces = [30.0] * len(move_joints)
        )

def go_home(panda, pandaId):
    c = 0
    cc= 0
    while True:
        panda.move_arm(pandaId, [HOME_X, HOME_Y, HOME_Z])
        p.stepSimulation()
        cart = panda.get_cart()
        e = (cart[0] - HOME_X)**2 + (cart[1] -HOME_Y)**2 + (cart[2] -HOME_Z)**2
        ee = math.sqrt(e)
        print(ee)
        if ee < 0.07:
            c += 1
            cc=0
            break
    
        cc +=1
        print(c)
        print(cc)
        time.sleep(1/240)

def reset_joints(pid): # Resets joints to the REST_POSE values, used to set initial position
    move_joints = []
    for i in range(p.getNumJoints(pid)):
        joint = p.getJointInfo(pid, i)
        joint_type = joint[2]
        if joint_type != p.JOINT_FIXED:
            move_joints.append(i)
    for pose_i, joint_id in enumerate(move_joints):
        p.resetJointState(pid, joint_id, REST_POSES[pose_i])
    return None

def draw_c(): # Creates the array of points in a C shape
    radius  = 0.2
    theta = np.linspace(np.pi/3, (5/3)*np.pi, 100)
    y = 0.4 - radius * np.cos(theta)
    z = 0.6 + radius * np.sin(theta)
    x = np.full_like(y, 0.47)
    points = np.column_stack((x, y))
    points = np.column_stack((points, z))
    return points

def draw_u(): # Creates the array of points in a U shape
    points2 = []
    for i in range(30):
        points2.append([0.47, 0.1, 0.8-(i*0.01)])
    radius = 0.1
    theta = np.linspace(np.pi, 2*np.pi, 30)
    y = 0 - radius * np.cos(theta)
    z = 0.55 + radius * np.sin(theta)
    x = np.full_like(y, 0.47)
    points = np.column_stack((x, y))
    points = np.column_stack((points, z))
    # points2.append(points)
    points3 = []
    for i in range(30):
        points3.append([0.47, -0.1, 0.55+(i*0.01)])
    points = np.concatenate((points2, points), axis=0)
    points = np.concatenate((points, points3), axis=0)
    return points

def draw_buff():
    return np.array([[0.47,0.264,0.320],[0.47,0.303,0.320],[0.47,0.347,0.329],[0.47,0.378,0.341],[0.47,0.394,0.349],[0.47,0.404,0.273],[0.47,0.414,0.230],
                     [0.47,0.436,0.198],[0.47,0.456,0.192],[0.47,0.479,0.162],[0.47,0.499,0.149],[0.47,0.523,0.151],[0.47,0.553,0.187],[0.47,0.580,0.236],
                     [0.47,0.590,0.260],[0.47,0.591,0.297],[0.47,0.581,0.322],[0.47,0.573,0.349],[0.47,0.552,0.377],[0.47,0.537,0.403],[0.47,0.537,0.435],
                     [0.47,0.527,0.465],[0.47,0.501,0.491],[0.47,0.478,0.507],[0.47,0.481,0.539],[0.47,0.478,0.570],[0.47,0.474,0.586],[0.47,0.495,0.597],
                     [0.47,0.511,0.614],[0.47,0.515,0.638],[0.47,0.503,0.654],[0.47,0.482,0.663],[0.47,0.458,0.664],[0.47,0.435,0.665,],[0.47,0.419,0.678,],
                     [0.47,0.400,0.691,],[0.47,0.367,0.708,],[0.47,0.325,0.721,],[0.47,0.302,0.724,],[0.47,0.273,0.767,],
                      [0.47,0.245,0.801,],[0.47,0.214,0.832,],[0.47,0.180,0.853,],[0.47,0.136,0.872,],[0.47,0.094,0.883,],[0.47,0.054,0.890,],[0.47,0.003,0.889,],
                      [0.47,-0.031,0.884],[0.47,-0.076,0.871],[0.47,-0.118,0.851],[0.47,-0.156,0.824],[0.47,-0.182,0.793],[0.47,-0.207,0.763],[0.47,-0.246,0.761],
                      [0.47,-0.273,0.754],[0.47,-0.307,0.742],[0.47,-0.353,0.714],[0.47,-0.390,0.668],[0.47,-0.413,0.622],[0.47,-0.420,0.572],[0.47,-0.420,0.518],
                      [0.47,-0.401,0.479],[0.47,-0.373,0.436],[0.47,-0.340,0.397],[0.47,-0.305,0.366],[0.47,-0.266,0.328],[0.47,-0.245,0.318],[0.47,-0.226,0.315],
                      [0.47,-0.200,0.293],[0.47,-0.173,0.293],[0.47,-0.159,0.308],[0.47,-0.148,0.316],[0.47,-0.121,0.291],[0.47,-0.096,0.271],[0.47,-0.073,0.258],
                      [0.47,-0.044,0.236],[0.47,-0.021,0.209],[0.47,-0.015,0.199],[0.47,0.047,0.198],[0.47,0.084,0.192],[0.47,0.115,0.196],[0.47,0.125,0.215],
                      [0.47,0.121,0.274],[0.47,0.109,0.318],[0.47,0.098,0.346],[0.47,0.091,0.353],[0.47,0.146,0.352],[0.47,0.175,0.338],[0.47,0.213,0.326]])



if __name__ == "__main__":
    pandaId = setup_pybullet()
    panda = Panda(pandaId)
    print("Done setting up pybullet")
    p.setRealTimeSimulation(0)

    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video.mp4")
    reset_joints(pandaId) # Set initial pose
    c_points = draw_c()
    dist_remaining = 0

    for point in c_points:
        while True:
            panda.move_arm(pandaId, point)
            p.stepSimulation()
            x_pos = panda.get_cart()[0]
            y_pos = panda.get_cart()[1]
            z_pos = panda.get_cart()[2]
            dist_remaining = np.sqrt((point[0] - x_pos)**2 + (point[1] - y_pos)**2 + (point[2]-z_pos)**2)
            print(f"This is the remaining distance: {dist_remaining}")
            if dist_remaining <= 0.01: # Adjust for how close the robot needs to get to the points, in meters
                panda.draw_on_blackboard()
                break

    go_home(panda, pandaId)
    reset_joints(pandaId)           
    u_points = draw_u()
    dist_remaining = 0

    for point in u_points:
        while True:
            panda.move_arm(pandaId, point)
            p.stepSimulation()
            x_pos = panda.get_cart()[0]
            y_pos = panda.get_cart()[1]
            z_pos = panda.get_cart()[2]
            dist_remaining = np.sqrt((point[0] - x_pos)**2 + (point[1] - y_pos)**2 + (point[2]-z_pos)**2)
            print(f"This is the remaining distance: {dist_remaining}")
            if dist_remaining <= 0.01: # Adjust for how close the robot needs to get to the points, in meters
                panda.draw_on_blackboard()
                break
    
    go_home(panda, pandaId)
    time.sleep(1)
    reset_joints(pandaId)
    buff_points = draw_buff()

    for point in buff_points:
        while True:
            panda.move_arm(pandaId, point)
            p.stepSimulation()
            x_pos = panda.get_cart()[0]
            y_pos = panda.get_cart()[1]
            z_pos = panda.get_cart()[2]
            dist_remaining = np.sqrt((point[0] - x_pos)**2 + (point[1] - y_pos)**2 + (point[2]-z_pos)**2)
            if dist_remaining <= 0.01: # Adjust for how close the robot needs to get to the points, in meters
                panda.draw_on_blackboard()
                break
    go_home(panda, pandaId)       
    p.stopStateLogging(log_id)
    while p.isConnected():
        time.sleep(1/2)

    p.disconnect()
