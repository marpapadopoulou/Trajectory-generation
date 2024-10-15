import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11
pandaNumDofs = 7
maxV = 0.6

ll = [-7]*pandaNumDofs
ul = [7]*pandaNumDofs
jr = [7]*pandaNumDofs
jointPositions = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4, 0.0, 0.0]
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
    tar_p = camera_pose + z_direction*distance

    viewMatrix = p.computeViewMatrix(position, tar_p, -z_direction, physicsClientId=physicsClientId)
    projectionMatrix = p.computeProjectionMatrixFOV(
        fov=42.5,
        aspect=1,
        nearVal=0.01,
        farVal=10,
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

        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.tableUid = self.bullet_client.loadURDF("table/table.urdf", basePosition=[-0.5, 0, -0.65])
        self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.65, 0.0, 0.34]) + self.offset, flags=flags)
        time.sleep(timeStep)
        self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.7, 0.1, 0.3]) + self.offset, flags=flags)
        time.sleep(timeStep)
        self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.6, 0.13, 0.5]) + self.offset, flags=flags)

        self.sphereId = self.bullet_client.loadURDF("sphere_small.urdf", np.array([-0.60, 0., 0.3]) + self.offset, flags=flags)
        self.bullet_client.loadURDF("sphere_small.urdf", np.array([-0.6, -0.12, 0.6]) + self.offset, flags=flags)
        self.bullet_client.loadURDF("sphere_small.urdf", np.array([-0.63, -0.1, 0.1]) + self.offset, flags=flags)

        orn = [0.0, 0.0, 1, 0.0]
        self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0, 0, 0]) + self.offset, orn, useFixedBase=True, flags=flags)
        self.reset()
        self.t = 0.
        self.EE_length = 0.04
        self.bullet_client.enableJointForceTorqueSensor(self.panda, 9)
        self.bullet_client.enableJointForceTorqueSensor(self.panda, 10)

    def reset(self):
        index = 0
        for j in range(self.bullet_client.getNumJoints(self.panda)):
            self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
            info = self.bullet_client.getJointInfo(self.panda, j)

            jointName = info[1]
            jointType = info[2]
            if (jointType == self.bullet_client.JOINT_PRISMATIC):
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index])
                index += 1
            if (jointType == self.bullet_client.JOINT_REVOLUTE):
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index])
                index += 1
        self.init_pose, self.init_orn = self.bullet_client.getLinkState(self.panda, 11)[:2]

    def step(self):
        setCameraOnRobotWrist(self.bullet_client, self.panda, 11)
        t = self.t
        self.t += 1./60.
        pos = [-0.65 - 0.1* math.sin(1.5 * t), self.offset[1] + 0.1* math.cos(1.5 * t), self.offset[2] + 0.4]
        orn = [0.0, 1.0, 0.0, 0.0]
        jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=10)
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240.)
    
    def gripper_homing(self):
        self.bullet_client.setJointMotorControl2(self.panda, 9, self.bullet_client.POSITION_CONTROL, 0.04, force=5 * 240., maxVelocity=maxV)
        self.bullet_client.setJointMotorControl2(self.panda, 10, self.bullet_client.POSITION_CONTROL, 0.04, force=5 * 240., maxVelocity=maxV)
        if abs(self.bullet_client.getJointState(self.panda, 9)[0] - 0.04) < 1e-5:
            return True
        return False

    def move_in_direction(self, direction):
        pos, orn = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex)[4:6]
        pos = np.array(pos)
        step_size = 0.01

        if direction == 'UP':
            pos[2] += step_size
        elif direction == 'DOWN':
            pos[2] -= step_size
        elif direction == 'LEFT':
            pos[1] += step_size
        elif direction == 'RIGHT':
            pos[1] -= step_size
        elif direction == 'FORWARD':
            pos[0] += step_size
        elif direction == 'BACKWARD':
            pos[0] -= step_size

        jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=10)
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240.)

    def down(self):
        pos, orn = self.bullet_client.getBasePositionAndOrientation(self.sphereId)
        orn = [0.0, 1.0, 0.0, 0.0]
        jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=10)
        success = True
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240., maxVelocity=maxV)
        if np.abs(np.array(self.bullet_client.getLinkState(self.panda, 11)[4]) - np.array(pos)).sum() > 1e-4:
            success = success & False
        return success
    
    def ready_pose(self):
        jointPoses = jointPositions
        success = True
        for i in range(pandaNumDofs):
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240., maxVelocity=maxV)
        if np.abs(np.array(self.bullet_client.getLinkState(self.panda, 11)[4]) - np.array(self.init_pose)).sum() > 1e-3:
            success = success & False
        return success

    def grasp(self):
        self.bullet_client.setJointMotorControl2(self.panda, 9, self.bullet_client.POSITION_CONTROL, 0.0, force=5 * 240., maxVelocity=maxV)
        self.bullet_client.setJointMotorControl2(self.panda, 10, self.bullet_client.POSITION_CONTROL, 0.0, force=5 * 240., maxVelocity=maxV)
        if abs(self.bullet_client.getJointState(self.panda, 9)[2][0]) > 0.5:
            return True
        return False

if __name__ == '__main__':
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())

    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                 cameraPitch=-40, cameraTargetPosition=[-0.5, -0.9, 1.5])
    timeStep = 1./240.  # Increase for faster simulation
    p.setTimeStep(timeStep)
    p.setRealTimeSimulation(1)

    p.setGravity(0,0,-9.8)
    
    panda = PandaSim(p,[0,0,0])
    print("Press 'H' for homing, 'A' to approach, 'D' to move down, 'G' to grasp, 'R' to return to ready pose, arrow keys to move, 'Q' to quit.")

    while True:
        keys = p.getKeyboardEvents()

        if ord('H') in keys and keys[ord('H')] & p.KEY_WAS_TRIGGERED:
            while not panda.gripper_homing():
                setCameraOnRobotWrist(p, panda.panda, 11)
                p.stepSimulation()

        if ord('A') in keys and keys[ord('A')] & p.KEY_WAS_TRIGGERED:
            while not panda.approaching():
                setCameraOnRobotWrist(p, panda.panda, 11)
                p.stepSimulation()

        if ord('D') in keys and keys[ord('D')] & p.KEY_WAS_TRIGGERED:
            while not panda.down():
                setCameraOnRobotWrist(p, panda.panda, 11)
                p.stepSimulation()

        if ord('G') in keys and keys[ord('G')] & p.KEY_WAS_TRIGGERED:
            while not panda.grasp():
                setCameraOnRobotWrist(p, panda.panda, 11)
                p.stepSimulation()

        if ord('R') in keys and keys[ord('R')] & p.KEY_WAS_TRIGGERED:
            while not panda.ready_pose():
                setCameraOnRobotWrist(p, panda.panda, 11)
                p.stepSimulation()

        if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
            panda.move_in_direction('UP')
            setCameraOnRobotWrist(p, panda.panda, 11)
            p.stepSimulation()

        if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
            panda.move_in_direction('DOWN')
            setCameraOnRobotWrist(p, panda.panda, 11)
            p.stepSimulation()

        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            panda.move_in_direction('LEFT')
            setCameraOnRobotWrist(p, panda.panda, 11)
            p.stepSimulation()

        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            panda.move_in_direction('RIGHT')
            setCameraOnRobotWrist(p, panda.panda, 11)
            p.stepSimulation()

        if ord('W') in keys and keys[ord('W')] & p.KEY_IS_DOWN:
            panda.move_in_direction('FORWARD')
            setCameraOnRobotWrist(p, panda.panda, 11)
            p.stepSimulation()

        if ord('S') in keys and keys[ord('S')] & p.KEY_IS_DOWN:
            panda.move_in_direction('BACKWARD')
            setCameraOnRobotWrist(p, panda.panda, 11)
            p.stepSimulation()

        if ord('Q') in keys and keys[ord('Q')] & p.KEY_WAS_TRIGGERED:
            break

        p.stepSimulation()
