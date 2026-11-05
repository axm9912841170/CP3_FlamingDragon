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
        #sprint(tip_position)
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
        #point = [0.5,0.5,0.5]
        point2 = [0,0,0]
        move_joints = []
        for i in range(p.getNumJoints(pid)):
            joint = p.getJointInfo(pid, i)
            joint_type = joint[2]
            print(joint)
            if joint_type != p.JOINT_FIXED:
                move_joints.append(i)
        move_joints2 = move_joints[0:7]
        #target_orientation = p.getQuaternionFromEuler([math.radians(180),0, math.radians(90)])
        joint_angles = p.calculateInverseKinematics(pid, 11, point[0:3])
        joint_angles2 = p.calculateInverseKinematics(pid, 11, point2)
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
            forces = [50.0] * len(move_joints)
        )

        # for(i, jidx) in enumerate(move_joints):

        #     p.setJointMotorControl2(pid,jidx,p.POSITION_CONTROL,joint_angles[i])
        # print("hello")
        # print(p.getNumJoints(pid))
        # print(len(joint_angles))
        # for ji in range(p.getNumJoints):
        #     p.resetJointState(pid, ji, joint_angles[ji])
        #self.draw_on_blackboard()



def go_home(panda, pandaId):
    c = 0
    cc= 0
    while True:
        p.stepSimulation()
        panda.move_arm(pandaId, [0, 0.5, 0.5])
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


if __name__ == "__main__":
    pandaId = setup_pybullet()
    panda = Panda(pandaId)
    print("Done setting up pybullet")

    radius  = 0.5
    theta = np.linspace(0, 2 * np.pi, 100)
    x = 0.5 + radius * np.cos(theta)
    y = 0.5 + radius * np.sin(theta)
    z = np.full_like(x, 0.5)
    points = np.column_stack((x, y))
    points = np.column_stack((points, z))

    ##
    ## y = green axis, x = red axis, z = blue axis

    # points2 = [[0.5, 0, 0.5], [0.5, 0.01, 0.5], [0.5, 0.02, 0.5], [0.5, 0.03, 0.5], [0.5, 0.04, 0.5],
    #            [0.5, 0.05, 0.5], [0.5, 0.06, 0.5], [0.5, 0.07, 0.5], [0.5, 0.08, 0.5], [0.5, 0.09, 0.5],
    #            [0.5, 0.1, 0.5], [0.5, 0.11, 0.5], [0.5, 0.13, 0.5], [0.5, 0.14, 0.5], [0.5, 0.15, 0.5],
    #            [0.5, 0.16, 0.5], [0.5, 0.17, 0.5], [0.5, 0.18, 0.5], [0.5, 0.19, 0.5], [0.5, 0.2, 0.5]]
    # points3 = [[0, 0.5, 0.5], [0.01, 0.5, 0.5], [0.02, 0.5, 0.5], [0.03, 0.5, 0.5], [0.04, 0.5, 0.5],
    #                [0.05, 0.5, 0.5], [0.06, 0.5, 0.5], [0.07, 0.5, 0.5], [0.08, 0.5, 0.5], [0.09, 0.5, 0.5],
    #                [0.1, 0.5, 0.5], [0.11, 0.5, 0.5], [0.12, 0.5, 0.5], [0.13, 0.5, 0.5], [0.14, 0.5, 0.5],
    #                [0.15, 0.5, 0.5], [0.16, 0.5, 0.5], [0.17, 0.5, 0.5], [0.18, 0.5, 0.5], [0.19, 0.05, 0.5],]
    # points4 = [
    #            [0.75, 0, 0.5], [0.75, 0.01, 0.5], [0.75, 0.02, 0.5], [0.75, 0.03, 0.5], [0.75, 0.04, 0.5],
    #            [0.75, 0.05, 0.5], [0.75, 0.06, 0.5], [0.75, 0.07, 0.5], [0.75, 0.08, 0.5], [0.75, 0.09, 0.5],
    #            [0.75, 0.1, 0.5], [0.75, 0.11, 0.5], [0.75, 0.13, 0.5], [0.75, 0.14, 0.5], [0.75, 0.15, 0.5],
    #            [0.75, 0.16, 0.5], [0.75, 0.17, 0.5], [0.75, 0.18, 0.5], [0.75, 0.19, 0.5], [0.75, 0.2, 0.5]]
    # points5 = [[0.5,0,0.75]]

    pointsbuff = [[0.5,0.435,0.665, "yes"],[0.5,0.419,0.678, "yes"],[0.5,0.400,0.691, "yes"],[0.5,0.367,0.708, "yes"],[0.5,0.325,0.721, "yes"],[0.5,0.302,0.724, "yes"],[0.5,0.273,0.767, "yes"],
                  [0.5,0.245,0.801, "yes"],[0.5,0.214,0.832, "yes"],[0.5,0.180,0.853, "yes"],[0.5,0.136,0.872, "yes"],[0.5,0.094,0.883, "yes"],[0.5,0.054,0.890, "yes"],[0.5,0.003,0.889, "yes"],
                  [0.5,-0.031,0.884,"yes"],[0.5,-0.076,0.871,"yes"],[0.5,-0.118,0.851,"yes"],[0.5,-0.156,0.824,"yes"],[0.5,-0.182,0.793,"yes"],[0.5,-0.207,0.763,"yes"],[0.5,-0.246,0.761,"yes"],
                  [0.5,-0.273,0.754,"yes"],[0.5,-0.307,0.742,"yes"],[0.5,-0.353,0.714,"yes"],[0.5,-0.390,0.668,"yes"],[0.5,-0.413,0.622,"yes"],[0.5,-0.420,0.572,"yes"],[0.5,-0.420,0.518,"yes"],
                  [0.5,-0.401,0.479,"yes"],[0.5,-0.373,0.436,"yes"],[0.5,-0.340,0.397,"yes"],[0.5,-0.305,0.366,"yes"],[0.5,-0.266,0.328,"yes"],[0.5,-0.245,0.318,"yes"],[0.5,-0.226,0.315,"yes"],
                  [0.5,-0.200,0.293,"yes"],[0.5,-0.173,0.293,"yes"],[0.5,-0.159,0.308,"yes"],[0.5,-0.148,0.316,"yes"],[0.5,-0.121,0.291,"yes"],[0.5,-0.096,0.271,"yes"],[0.5,-0.073,0.258,"yes"],
                  [0.5,-0.044,0.236,"yes"],[0.5,-0.021,0.209,"yes"],[0.5,-0.015,0.199,"yes"],[0.5,0.047,0.198,"yes"],[0.5,0.084,0.192,"yes"],[0.5,0.115,0.196,"yes"],[0.5,0.125,0.215,"yes"],
                  [0.5,0.121,0.274,"yes"],[0.5,0.109,0.318,"yes"],[0.5,0.098,0.346,"yes"],[0.5,0.091,0.353,"yes"],[0.5,0.146,0.352,"yes"],[0.5,0.175,0.338,"yes"],[0.5,0.213,0.326,"yes"],
                  [0.5,0.264,0.320,"yes"],[0.5,0.303,0.320,"yes"],[0.5,0.347,0.329,"yes"],[0.5,0.378,0.341,"yes"],[0.5,0.394,0.349,"yes"],[0.5,0.404,0.273,"yes"],[0.5,0.414,0.230,"yes"],
                  [0.5,0.436,0.198,"yes"],[0.5,0.456,0.192,"yes"],[0.5,0.479,0.162,"yes"],[0.5,0.499,0.149,"yes"],[0.5,0.523,0.151,"yes"],[0.5,0.553,0.187,"yes"],[0.5,0.580,0.236,"yes"],
                  [0.5,0.590,0.260,"yes"],[0.5,0.591,0.297,"yes"],[0.5,0.581,0.322,"yes"],[0.5,0.573,0.349,"yes"],[0.5,0.552,0.377,"yes"],[0.5,0.537,0.403,"yes"],[0.5,0.537,0.435,"yes"],
                  [0.5,0.527,0.465,"yes"],[0.5,0.501,0.491,"yes"],[0.5,0.478,0.507,"yes"],[0.5,0.481,0.539,"yes"],[0.5,0.478,0.570,"yes"],[0.5,0.474,0.586,"yes"],[0.5,0.495,0.597,"yes"],
                  [0.5,0.511,0.614,"yes"],[0.5,0.515,0.638,"yes"],[0.5,0.503,0.654,"yes"],[0.5,0.482,0.663,"yes"],[0.5,0.458,0.664,"yes"]]

    points6 = []
    # points6.append([0.5, 0.1, 0.75])
    # points6.append([0.5, -0.1, 0.75])
    for i in range(30):
        points6.append([0.5, 0, 0.75-(i*0.01), "yes"])
    # for j in range(10):
    #     points6.append([0.5, 0-(j*0.01), 0.45-(j*0.01), "yes"])
    # for k in range(10):
    #     points6.append([0.5, -0.1-(k*0.01), 0.35+(k*0.01), "yes"])
    # for l in range(30):
    #     points6.append([0.5, -0.2, 0.45+(l*0.01), "yes"])
    nump = 30
    rad = 0.2
    theta2 = np.linspace(np.pi, 2 * np.pi, nump)
    z = 0.45 + rad * np.cos(theta2)
    y = rad * np.sin(theta2)
    smile = list(zip(np.round(y,3),np.round(z,3)))
    #points6.append(smile)
    print("Smile")
    print(smile)
    smile2 = []
    for i in range(len(smile)):
        smile2.append([0.5, float(smile[i][0]), float(smile[i][1]), "yes"])
    points6.append(smile2)
    for l in range(30):
        points6.append([0.5, -0.2, 0.45+(l*0.01), "yes"])

    # points6.append([0.4, -0.1, 0.35, "no"])
    # points6.append([0.4, -0.1, 0.45, "no"])
    # points6.append([0.4, -0.1, 0.55, "no"])
    # points6.append([0.4, -0.1, 0.65, "no"])
    # points6.append([0.4, -0.1, 0.75, "no"])
    # for k in range(40):
    #     points6.append([0.5, -0.2, 0.75-(k*0.01), "yes"])
    # for l in range(10):
    #     points6.append([0.5, -0.2+(l*0.01), 0.45-(l*0.01), "yes"])


    # points6.append([0.5, -0.05, 0.25])
    # points6.append([0.5, -0.1, 0.45])
    # for j in range(30):
    #     points6.append([0.5, -0.2, 0.75-(j*0.01)])
    #points4 = [[0.5,0,0.5],[0.55,0,0.5]]
    #print(points)

    p.setRealTimeSimulation(0)
    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video.mp4")

    #for point in points2:
    c = 0
    cc= 0
    ccc = 0
    while True:    
        #print(point)
        if c >= len(pointsbuff):
            # if ccc == 0:
            #     reset_arm(panda, pandaId)
            #     ccc += 1
            #     continue
            go_home(panda, pandaId)
            break

        
        p.stepSimulation()
        panda.move_arm(pandaId, pointsbuff[c])
        print("Cartesian position")
        print(panda.get_cart())
        

        cart = panda.get_cart()
        e = (cart[0] - pointsbuff[c][0])**2 + (cart[1] - pointsbuff[c][1])**2 + (cart[2] - pointsbuff[c][2])**2
        ee = math.sqrt(e)
        print(ee)
        if cc > 10 and ee < 0.08:
            if pointsbuff[c][3] == "yes":
                panda.draw_on_blackboard()
            c += 1
            cc=0
    
        cc +=1
        # print(c)
        # print(cc)
        print(smile2)
        p.stopStateLogging(log_id)
        time.sleep(1/200)
    
    while p.isConnected():
        time.sleep(1/2)
    #     panda.move_arm(pid=pandaId)
    #     # IMPORTANT - You need to run this command for every step in simulation
    #     p.stepSimulation() 
        
    #     # Command to stop recording whenx done
    #     p.stopStateLogging(log_id)
        #break

    p.disconnect()
