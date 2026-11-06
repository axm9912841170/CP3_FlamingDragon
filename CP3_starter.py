import pybullet as p
import pybullet_data
import numpy as np
import time, math


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
        upper_limits = [2.9671, 1.8326, 2.9671, 0.0, 2.9671, 3.8223, 2.9671, -1.0, -1.0, 0.04, 0.04, -1.0]

        point2 = [0,0,0]
        move_joints = []
        ll = []
        ul = []
        joint_ranges = []
        for i in range(p.getNumJoints(pid)):
            joint = p.getJointInfo(pid, i)
            joint_type = joint[2]
            # print(joint)
            if joint_type != p.JOINT_FIXED:
                ll.append(joint[8])
                ul.append(joint[9])
                move_joints.append(i)
        move_joints2 = move_joints[0:7]
        target_orientation = p.getQuaternionFromEuler([np.deg2rad(0), np.deg2rad(90), np.deg2rad(0)])
        joint_angles = p.calculateInverseKinematics(pid, 11, point)
        
        #activejoints = [0,1,2,3,4,5,6,7,8]
        # print("Joint angles")
        # print(len(joint_angles))
        # print(joint_angles)
        # print(joint_angles2)
        # print(move_joints)
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
        panda.move_arm(pandaId, [0, 0.5, 0.5])
        p.stepSimulation()
        cart = panda.get_cart()
        e = (cart[0] - 0)**2 + (cart[1] -0.5)**2 + (cart[2] - 0.5)**2
        ee = math.sqrt(e)
        print(ee)
        if cc > 10 and ee < 0.07:
            panda.draw_on_blackboard()
            c += 1
            cc=0
            break
    
        cc +=1
        print(c)
        print(cc)
        p.stopStateLogging(log_id)
        time.sleep(1/240)

def draw_c():
    radius  = 0.1
    theta = np.linspace(np.pi/3, (5/3)*np.pi, 100)
    y = 0.4 - radius * np.cos(theta)
    z = 0.4 + radius * np.sin(theta)
    x = np.full_like(y, 0.4)
    points = np.column_stack((x, y))
    points = np.column_stack((points, z))
    return points



if __name__ == "__main__":
    pandaId = setup_pybullet()
    panda = Panda(pandaId)
    print("Done setting up pybullet")

    

    # points2 = [[0.5, 0, 0.5], [0.5, 0.01, 0.5], [0.5, 0.02, 0.5], [0.5, 0.03, 0.5], [0.5, 0.04, 0.5],
    #            [0.5, 0.05, 0.5], [0.5, 0.06, 0.5], [0.5, 0.07, 0.5], [0.5, 0.08, 0.5], [0.5, 0.09, 0.5],
    #            [0.5, 0.1, 0.5], [0.5, 0.11, 0.5], [0.5, 0.13, 0.5], [0.5, 0.14, 0.5], [0.5, 0.15, 0.5],
    #            [0.5, 0.16, 0.5], [0.5, 0.17, 0.5], [0.5, 0.18, 0.5], [0.5, 0.19, 0.5], [0.5, 0.2, 0.5]]
    # points3 = [[0, 0.5, 0.5], [0.01, 0.5, 0.5], [0.02, 0.5, 0.5], [0.03, 0.5, 0.5], [0.04, 0.5, 0.5],
    #                [0.05, 0.5, 0.5], [0.06, 0.5, 0.5], [0.07, 0.5, 0.5], [0.08, 0.5, 0.5], [0.09, 0.5, 0.5],
    #                [0.1, 0.5, 0.5], [0.11, 0.5, 0.5], [0.12, 0.5, 0.5], [0.13, 0.5, 0.5], [0.14, 0.5, 0.5],
    #                [0.15, 0.5, 0.5], [0.16, 0.5, 0.5], [0.17, 0.5, 0.5], [0.18, 0.5, 0.5], [0.19, 0.05, 0.5],]
    # points4 = [[0.5,0,0.5],[0.55,0,0.5]]
    #print(points)

    p.setRealTimeSimulation(0)
    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video.mp4")

    #for point in points2:
    # c = 0
    # cc= 0

    # while True:    
    #     #print(point)
    #     if c >= len(draw_c()):
    #         break

    #     p.stepSimulation()
    #     if c == 0:
    #         panda.move_arm(pandaId, [0.5, 0.5, 0.5])
        
            
    #     print("Cartesian position")
    #     print(panda.get_cart())
    #     # print(f"Circle Points: {draw_c()}")

    #     cart = panda.get_cart()
    #     e = (cart[0] - points2[0])**2 + (cart[1] - points2[1])**2 + (cart[2] - points2[2])**2
    #     ee = math.sqrt(e)
    #     print(ee)
    #     if cc > 60 and ee < 0.75:
    #         panda.draw_on_blackboard()
    #         c += 1
    #         cc=0
    #         #break
    #     panda.move_arm(pandaId, draw_c()[c])
    #     print(f"Circle Coordinate: {draw_c()[c]}")
    #     cc +=1
    #     print(c)
    
    #     time.sleep(1/100)
    points = draw_c()
    dist_remaining = 0
    for point in points:
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
    p.stopStateLogging(log_id)
    while p.isConnected():
        time.sleep(1/2)
    #     panda.move_arm(pid=pandaId)
    #     # IMPORTANT - You need to run this command for every step in simulation
    #     p.stepSimulation() 
        
    #     # Command to stop recording whenx done
    #     p.stopStateLogging(log_id)
        #break

    p.disconnect()
