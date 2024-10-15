import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7
maxV = 0.6

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4, 0.0, 0.0]
rp = jointPositions

def setCameraOnRobotWrist(p, robot_id, link_id, physicsClientId=0):
    distance = 1
    position, orientation = p.getLinkState(robot_id,link_id, physicsClientId)[4:6]
    position = np.array(position)
    orientation = list(orientation)
    R_mat = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3,3)
    z_direction = R_mat[:,2]
    y_direction = R_mat[:,1]
    x_direction = R_mat[:,0]

    camera_pose = position + 0.05*x_direction - z_direction*0.08
    tar_p = camera_pose+z_direction*distance
    # print(orientation)

    # p.removeAllUserDebugItems()
    # x_end_p = (np.array(camera_pose) + np.array(x_direction*2)).tolist()
    # x_line_id = p.addUserDebugLine(camera_pose,x_end_p,[1,0,0])# y 轴
    # y_end_p = (np.array(camera_pose) + np.array(y_direction*2)).tolist()
    # y_line_id = p.addUserDebugLine(camera_pose,y_end_p,[0,1,0])# z轴
    # z_end_p = (np.array(camera_pose) + np.array(z_direction*2)).tolist()
    # z_line_id = p.addUserDebugLine(camera_pose,z_end_p,[0,0,1])


    viewMatrix = p.computeViewMatrix(position, tar_p, -z_direction, physicsClientId=physicsClientId)
    projectionMatrix = p.computeProjectionMatrixFOV(
        fov=42.5,               # 摄像头的视线夹角
        # fov=80,               # 摄像头的视线夹角
        aspect=1,
        nearVal=0.01,            # 摄像头焦距下限
        farVal=10,               # 摄像头能看上限
        physicsClientId=physicsClientId
    )

    p.getCameraImage(
        width=320, height=200,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix,
        physicsClientId=physicsClientId
    )
    return

class PandaSim(object):
    def __init__(self, bullet_client, offset):
        self.bullet_client = bullet_client
        self.offset = np.array(offset)

        #print("offset=",offset)
        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        legos=[]
        self.tableUid=self.bullet_client.loadURDF("table/table.urdf", basePosition=[-0.5,0,-0.65])
        # self.trayUid=self.bullet_client.loadURDF("tray/traybox.urdf", basePosition=[-0.65,0,0], flags=flags)
        # self.bullet_client.loadURDF("tray/traybox.urdf", [0+offset[0], 0+offset[1], -0.6+offset[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)
        self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.65, 0.0, 0.34])+self.offset, flags=flags)
        time.sleep(timeStep)
        self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.7, 0.1, 0.3])+self.offset, flags=flags)
        time.sleep(timeStep)
        self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.6, 0.13, 0.5])+self.offset, flags=flags)

        # legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.5, 0.1, 0.5])+self.offset, flags=flags))
        # legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.1, 0.3, -0.7])+self.offset, flags=flags))
        self.sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [-0.60, 0., 0.3])+self.offset, flags=flags)
        self.bullet_client.loadURDF("sphere_small.urdf",np.array( [-0.6, -0.12, 0.6])+self.offset, flags=flags)
        self.bullet_client.loadURDF("sphere_small.urdf",np.array( [-0.63, -0.1, 0.1])+self.offset, flags=flags)
        # orn=[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
        orn=[0.0, 0.0, 1, 0.0]
        eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
        self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
        self.reset()
        self.t = 0.
        self.EE_length = 0.04
        self.bullet_client.enableJointForceTorqueSensor(self.panda,9)
        self.bullet_client.enableJointForceTorqueSensor(self.panda,10)

        # frame_start_postition, frame_posture = p.getLinkState(self.panda,11)[4:6]
        # R_Mat = np.array(p.getMatrixFromQuaternion(frame_posture)).reshape(3,3)
        # x_axis = R_Mat[:,0]
        # x_end_p = (np.array(frame_start_postition) + np.array(x_axis*5)).tolist()
        # x_line_id = p.addUserDebugLine(frame_start_postition,x_end_p,[1,0,0])# y 轴
        # y_axis = R_Mat[:,1]
        # y_end_p = (np.array(frame_start_postition) + np.array(y_axis*5)).tolist()
        # y_line_id = p.addUserDebugLine(frame_start_postition,y_end_p,[0,1,0])# z轴
        # z_axis = R_Mat[:,2]
        # z_end_p = (np.array(frame_start_postition) + np.array(z_axis*5)).tolist()
        # z_line_id = p.addUserDebugLine(frame_start_postition,z_end_p,[0,0,1])

    
    def reset(self):
        index = 0
        for j in range(self.bullet_client.getNumJoints(self.panda)):
            self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
            info = self.bullet_client.getJointInfo(self.panda, j)

            jointName = info[1]
            jointType = info[2]
            if (jointType == self.bullet_client.JOINT_PRISMATIC):
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
                index=index+1
            if (jointType == self.bullet_client.JOINT_REVOLUTE):
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
                index=index+1
        self.init_pose, self.init_orn = self.bullet_client.getLinkState(self.panda,11)[:2]

        

    def step(self):
        # self.bullet_client.removeAllUserDebugItems()
        setCameraOnRobotWrist(self.bullet_client, self.panda, 11)
        # setCameraPicAndGetPic(self.panda)
        # self.bullet_client.getCameraImage(320,200)
        t = self.t
        self.t += 1./60.
        pos = [-0.65-0.1* math.sin(1.5 * t), self.offset[1]+0.1* math.cos(1.5 * t), self.offset[2]+ 0.4]
        # orn = self.bullet_client.getQuaternionFromEuler([0.,0,0])  #math.pi/2.
        orn=[0.0, 1.0, 0.0, 0.0]
        jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul,
            jr, rp, maxNumIterations=10)
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
    
    def gripper_homing(self):
        self.bullet_client.setJointMotorControl2(self.panda, 9, self.bullet_client.POSITION_CONTROL, 0.04, force=5 * 240., maxVelocity=maxV)
        self.bullet_client.setJointMotorControl2(self.panda, 10, self.bullet_client.POSITION_CONTROL, 0.04, force=5 * 240., maxVelocity=maxV)
        # print(self.bullet_client.getJointState(self.panda,0)[1])
        if abs(self.bullet_client.getJointState(self.panda,9)[0] - 0.04) < 1e-5:
            return True
        return False

    def approaching(self):
        pos,orn = self.bullet_client.getBasePositionAndOrientation(self.sphereId)
        pos = list(pos)
        pos[-1] += self.EE_length
        # orn = self.bullet_client.getQuaternionFromEuler([0.,0,0])  #math.pi/2.
        orn=[0.0, 1.0, 0.0, 0.0]

        # approching
        jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul,
            jr, rp, maxNumIterations=10)
        success = True
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240., maxVelocity=maxV)
            # print(self.bullet_client.getJointState(self.panda,i)[1])
        if np.abs(np.array(self.bullet_client.getLinkState(self.panda,11)[4]) - np.array(pos)).sum() > 1e-3:
            # print(np.abs(np.array(self.bullet_client.getLinkState(self.panda,11)[4]) - np.array(pos)).sum())
            success = success & False
        return success

    
    def down(self):
        pos,orn = self.bullet_client.getBasePositionAndOrientation(self.sphereId)
        orn=[0.0, 1.0, 0.0, 0.0]
        # approching
        jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul,
            jr, rp, maxNumIterations=10)
        success = True
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240., maxVelocity=maxV)
        if np.abs(np.array(self.bullet_client.getLinkState(self.panda,11)[4]) - np.array(pos)).sum() > 1e-4:
            success = success & False
        return success
    
    def ready_pose(self):
        jointPoses = jointPositions
        success = True
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240., maxVelocity=maxV)
        if np.abs(np.array(self.bullet_client.getLinkState(self.panda,11)[4]) - np.array(self.init_pose)).sum() > 1e-3:
            success = success & False
        return success


    def grasp(self):
        self.bullet_client.setJointMotorControl2(self.panda, 9, self.bullet_client.POSITION_CONTROL, 0.0, force=5 * 240., maxVelocity=maxV)
        self.bullet_client.setJointMotorControl2(self.panda, 10, self.bullet_client.POSITION_CONTROL, 0.0, force=5 * 240., maxVelocity=maxV)
        if abs(self.bullet_client.getJointState(self.panda,9)[2][0]) > 0.5:
            # self.bullet_client.setJointMotorControl2(self.panda, 9, self.bullet_client.VELOCITY_CONTROL, targetVelocity=0.0, force=5 * 240., maxVelocity=maxV)
            # self.bullet_client.setJointMotorControl2(self.panda, 10, self.bullet_client.VELOCITY_CONTROL, targetVelocity=0.0, force=5 * 240., maxVelocity=maxV)
            return True
        return False

  
if __name__=='__main__':
    p.connect(p.GUI)
    # p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
    p.setAdditionalSearchPath(pd.getDataPath())

    p.resetDebugVisualizerCamera(cameraDistance=1,cameraYaw=0,\
                                cameraPitch=-40,cameraTargetPosition=[-0.5,-0.9,1.5])
    timeStep=1./240.
    # p.setTimeStep(timeStep)
    p.setRealTimeSimulation(1)

    p.setGravity(0,0,-9.8)
    
    panda = PandaSim(p,[0,0,0])
    # setCameraPicAndGetPic(panda.panda)
    while(1):
        setCameraOnRobotWrist(p, panda.panda, 11)
        success = panda.gripper_homing()
        p.stepSimulation()
        # time.sleep(1/30.)
        if success == True:
            break

    while(1):
        setCameraOnRobotWrist(p, panda.panda, 11)
        success = panda.approaching()
        p.stepSimulation()
        # time.sleep(1/30.)
        if success == True:
            break

    while(1):
        setCameraOnRobotWrist(p, panda.panda, 11)
        success = panda.down()
        p.stepSimulation()
        # time.sleep(1/30.)
        if success == True:
            break

    while(1):
        setCameraOnRobotWrist(p, panda.panda, 11)
        success = panda.grasp()
        p.stepSimulation()
        # time.sleep(1/30.)
        if success == True:
            break

    while(1):
        setCameraOnRobotWrist(p, panda.panda, 11)
        success = panda.ready_pose()
        p.stepSimulation()
        # time.sleep(1/30.)
        if success == True:
            break

    p.removeBody(panda.sphereId)
    while(1):
        pass