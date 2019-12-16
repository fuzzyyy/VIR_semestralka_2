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


def calc_projection_matrix():
    fov, aspect, nearplane, farplane = 60, 1.0, 0.03, 10
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
    return projection_matrix


def calc_view_matrix(position, orientation):
    rot_matrix = p.getMatrixFromQuaternion(orientation)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    init_camera_vector = (0, 0, 1)
    init_up_vector = (0, 1, 0)
    camvec = rot_matrix.dot(init_camera_vector)
    upvec = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(position, position + 0.1 * camvec, upvec)
    return view_matrix, camvec

def my_getLinkState(robot):
    position, orientation, _,_,_,_ = p.getLinkState(robot, p.getNumJoints(robot) - 1, computeForwardKinematics=True)
    return position, orientation


projection_matrix = calc_projection_matrix()


def img_wrapper(robot):
    pos, orn, _, _, _, _ = p.getLinkState(robot, p.getNumJoints(robot) - 1, computeForwardKinematics=True)
    view_matrix, camvec = calc_view_matrix(pos, orn)
    return p.getCameraImage(128, 128, view_matrix, projection_matrix), pos, camvec

def my_getCameraImage(view_matrix, projection_matrix):
    return p.getCameraImage(128, 128, view_matrix, projection_matrix)

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


def test_camera():
    # zajimavy pozice na testovani:
    # [1.1731611377222513, -2.290785729075651, -1.2009673967971037, 0, -1.509407331996121, 2.6989293575021738, 0]

    positions = [0.186, 0.306, 0.847, -0.745, -0.131, 0.418, 0.036]  # pohled zblizka s kulickou 0.5 0.5 1
    ballId = p.loadURDF("sphere2red.urdf", [0.5, 0.5, 1], globalScaling=0.1, useFixedBase=True)
    ballId2 = p.loadURDF("sphere2red.urdf", [0.6, 0.5, 1.1], globalScaling=0.1, useFixedBase=True)
    p.setJointMotorControlArray(kukaId, range(numjoints), p.POSITION_CONTROL, targetPositions=positions)
    for i in range(500):
        p.stepSimulation()
    # img, pos, camvec = img_wrapper(kukaId)
    pos, orn = my_getLinkState(kukaId)
    view_matrix, camvec = calc_view_matrix(pos, orn)
    img = my_getCameraImage(view_matrix, projection_matrix)
    time.sleep(2)
    newpos = pos + camvec
    print(f"newpos is {newpos}")
    p.resetBasePositionAndOrientation(ballId2, (1, 1, 0), p.getQuaternionFromEuler((0, 0, 0)))
    p.stepSimulation()
    # img, pos, camvec = img_wrapper(kukaId)
    pos, orn = my_getLinkState(kukaId)
    view_matrix, camvec = calc_view_matrix(pos, orn)
    img = my_getCameraImage(view_matrix, projection_matrix)

    time.sleep(173)


def test_camera_do_dalky():
    positions = [0.664, -0.064, -0.702, -0.64, 0.218, 2.093, -0.33]  # pohled zdalky na kulicku 0.5 0 0
    ballId = p.loadURDF("sphere2red.urdf", [0.5, 0, 0], globalScaling=0.1, useFixedBase=True)
    ballId2 = p.loadURDF("sphere2red.urdf", [1, 0, 0], globalScaling=0.1, useFixedBase=True)
    p.setJointMotorControlArray(kukaId, range(numjoints), p.POSITION_CONTROL, targetPositions=positions)
    for i in range(500):
        p.stepSimulation()
    # pos, orn, _, _, _, _ = p.getLinkState(kukaId, numjoints - 1)
    # view_matrix, camvec = calc_view_matrix(pos, orn)
    # img = p.getCameraImage(128, 128, view_matrix, projection_matrix)
    # img, pos, camvec = img_wrapper(kukaId)
    pos, orn = my_getLinkState(kukaId)
    view_matrix, camvec = calc_view_matrix(pos, orn)
    img = my_getCameraImage(view_matrix, projection_matrix)
    time.sleep(2)
    newpos = pos + camvec
    print(f"newpos is {newpos}")
    p.resetBasePositionAndOrientation(ballId2, (1, 1, 0), p.getQuaternionFromEuler((0, 0, 0)))
    p.stepSimulation()
    # img = img_wrapper(kukaId)
    pos, orn = my_getLinkState(kukaId)
    view_matrix, camvec = calc_view_matrix(pos, orn)
    img = my_getCameraImage(view_matrix, projection_matrix)
    time.sleep(173)


def freestyle():
    ballId = p.loadURDF("sphere2red.urdf", [0.5, 0, 0], globalScaling=0.1, useFixedBase=True)
    for i in range(100):
        joint = np.random.randint(numjoints - 1)
        newval = np.random.uniform(-3.14, 3.14)
        positions[joint] = newval
        print(f"setting {joint=} to {newval=}")
        p.setJointMotorControlArray(kukaId, range(numjoints), p.POSITION_CONTROL, targetPositions=positions)
        #   print_joint_states(kukaId)
        for _ in range(100):
            p.stepSimulation()
        # vykradeno z https://github.com/bulletphysics/bullet3/issues/1616
        # pos, orn, _, _, _, _ = p.getLinkState(kukaId, numjoints - 1)
        # view_matrix, camera_vector = calc_view_matrix(pos, orn)
        # img = p.getCameraImage(192, 128, view_matrix, projection_matrix)
        # img, pos, camera_vector = img_wrapper(kukaId)
        pos, orn = my_getLinkState(kukaId)
        view_matrix, camvec = calc_view_matrix(pos, orn)
        p.resetBasePositionAndOrientation(ballId, pos + 0.2 * camvec, p.getQuaternionFromEuler((0, 0, 0)))
        img = my_getCameraImage(view_matrix, projection_matrix)
        # img, pos, camera_vector = img_wrapper(kukaId)

        print(f"------------- iteration {i} over")
        print(f"positions are {positions}")
        time.sleep(10)
        # keys = p.getKeyboardEvents()


# test_camera()
# test_camera_do_dalky()
freestyle()
