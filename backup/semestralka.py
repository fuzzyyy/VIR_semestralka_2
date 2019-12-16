import pybullet as p
import pybullet_data
import time
import numpy as np

def print_joint_info(robot):
    print(f"joint info for robot {robot}")
    num = p.getNumJoints(robot)
    for i in range(num):
        print(p.getJointInfo(robot, i))

def print_joint_states(robot):
    print(f"joint states for robot {robot}")
    num = p.getNumJoints(robot)
    for i in range(num):
        print(p.getJointState(robot, i))
    # print(p.getJointStates(robot, range(num)))

def print_link_states(robot):
    print(f"link states for robot {robot}")
    num = p.getNumJoints(robot)
    for i in range(num):
        print(p.getLinkState(robot, i, computeForwardKinematics=True))

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 10, -10)
planeId = p.loadURDF("plane.urdf")

kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)

print_joint_info(kukaId)
print_joint_states(kukaId)
print_link_states(kukaId)
numjoints = p.getNumJoints(kukaId)
positions = [0] * numjoints
p.setJointMotorControlArray(kukaId, range(numjoints), p.POSITION_CONTROL, targetPositions=positions)

fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

ballId = p.loadURDF("sphere2red.urdf", [2, 2, 0])
time.sleep(5)

#for i in range(100):
#   joint = np.random.randint(numjoints)
#   newval = np.random.uniform(-3.14, 3.14)
#   positions[joint] = newval
#   print(f"setting {joint=} to {newval=}")
  p.setJointMotorControlArray(kukaId, range(numjoints), p.POSITION_CONTROL, targetPositions=positions)
  # print_joint_states(kukaId)
  # for _ in range(100):
  #   p.stepSimulation()
  # vykradeno z https://github.com/bulletphysics/bullet3/issues/1616
  pos, orn, _, _, _, _ = p.getLinkState(kukaId, numjoints - 1)
  # rot_matrix = p.getMatrixFromQuaternion(orn)
  # rot_matrix = np.array(rot_matrix).reshape(3, 3)
  # camvec = [0, 1, 0]  # kamera kouka smerem osy y
  # camera_vector = rot_matrix.dot(camvec)
  # zvec = [0, 0, 1] # upvector kamery je smerem osy z
  # camera_upvector = rot_matrix.dot(zvec)
  # view_matrix = p.computeViewMatrix(pos, pos + 0.1*camera_vector, camera_upvector)
  #
  # img = p.getCameraImage(128, 128, view_matrix, projection_matrix)
  #
  # print(f"------------- iteration {i} over")
  # time.sleep(2)



# pos, orn, _, _, _, _ = p.getLinkState(kukaId, numjoints-1, computeForwardKinematics=True)
# print(pos, orn)
# print("DONE")
# time.sleep(5)
# print_joint_states(kukaId)
