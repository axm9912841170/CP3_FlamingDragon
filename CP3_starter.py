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
        joint_angles = p.calculateInverseKinematics(pid, 11, point)
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
            forces = [30.0] * len(move_joints)
        )

        # for(i, jidx) in enumerate(move_joints):

        #     p.setJointMotorControl2(pid,jidx,p.POSITION_CONTROL,joint_angles[i])
        # print("hello")
        # print(p.getNumJoints(pid))
        # print(len(joint_angles))
        # for ji in range(p.getNumJoints):
        #     p.resetJointState(pid, ji, joint_angles[ji])
        #self.draw_on_blackboard()





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

    points2 = [[0.5, 0, 0.5], [0.5, 0.01, 0.5], [0.5, 0.02, 0.5], [0.5, 0.03, 0.5], [0.5, 0.04, 0.5],
               [0.5, 0.05, 0.5], [0.5, 0.06, 0.5], [0.5, 0.07, 0.5], [0.5, 0.08, 0.5], [0.5, 0.09, 0.5],
               [0.5, 0.1, 0.5], [0.5, 0.11, 0.5], [0.5, 0.13, 0.5], [0.5, 0.14, 0.5], [0.5, 0.15, 0.5],
               [0.5, 0.16, 0.5], [0.5, 0.17, 0.5], [0.5, 0.18, 0.5], [0.5, 0.19, 0.5], [0.5, 0.2, 0.5]]
    points3 = [[0, 0.5, 0.5], [0.01, 0.5, 0.5], [0.02, 0.5, 0.5], [0.03, 0.5, 0.5], [0.04, 0.5, 0.5],
                   [0.05, 0.5, 0.5], [0.06, 0.5, 0.5], [0.07, 0.5, 0.5], [0.08, 0.5, 0.5], [0.09, 0.5, 0.5],
                   [0.1, 0.5, 0.5], [0.11, 0.5, 0.5], [0.12, 0.5, 0.5], [0.13, 0.5, 0.5], [0.14, 0.5, 0.5],
                   [0.15, 0.5, 0.5], [0.16, 0.5, 0.5], [0.17, 0.5, 0.5], [0.18, 0.5, 0.5], [0.19, 0.05, 0.5],]
    points4 = [[0.5,0,0.5],[0.55,0,0.5]]
    #print(points)

    p.setRealTimeSimulation(0)
    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video.mp4")

    #for point in points2:
    c = 0
    cc= 0
    while True:    
        #print(point)
        if c >= len(points3):
            break

        
        p.stepSimulation()
        panda.move_arm(pandaId, [0.5,0,0.5])
        print("Cartesian position")
        print(panda.get_cart())
        

        cart = panda.get_cart()
        e = cart[0]**2 + cart[1]**2 + cart[2]**2
        ee = math.sqrt(e)
        print(ee)
        if cc > 30 and ee < 0.75:
            panda.draw_on_blackboard()
            c += 1
            cc=0
            #break
        cc +=1
        print(c)
        p.stopStateLogging(log_id)
        time.sleep(1/100)
    
    while p.isConnected():
        time.sleep(1/2)
    #     panda.move_arm(pid=pandaId)
    #     # IMPORTANT - You need to run this command for every step in simulation
    #     p.stepSimulation() 
        
    #     # Command to stop recording whenx done
    #     p.stopStateLogging(log_id)
        #break

    p.disconnect()
