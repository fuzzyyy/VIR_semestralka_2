import pybullet as p
import pybullet_data
import time
import numpy as np

import os, glob, random
import math

from pickle_testing import data_management as d
import generate_camera_coords as gnr

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
 #   position, orientation, _,_,_,_ = p.getLinkState(robot, p.getNumJoints(robot) - 1, computeForwardKinematics=True)
    _, _, _, _, position, orientation = p.getLinkState(robot, p.getNumJoints(robot) - 1, computeForwardKinematics=True)
    return position, orientation

def img_wrapper(robot):
    pos, orn, _, _, _, _ = p.getLinkState(robot, p.getNumJoints(robot) - 1, computeForwardKinematics=True)
    view_matrix, camvec = calc_view_matrix(pos, orn)
    return p.getCameraImage(128, 128, view_matrix, projection_matrix), pos, camvec

def my_getCameraImage(view_matrix, projection_matrix):
    return p.getCameraImage(128, 128, view_matrix, projection_matrix)

def move(joints):
    p.setJointMotorControlArray(kukaId, range(numjoints), p.POSITION_CONTROL, targetPositions=joints)
    for _ in range(100):
        p.stepSimulation()

def camera_coords_in_img_coords(): #TODO:
    return

def create_dataset():
    length_img = 0.15
    width_img = 0.10

    x_img = 0.58
    y_img = 0
    z_img = 0.25

    x_width = width_img/2   #0.01
    y_width = length_img/2    #0.01
    z_width = 0.05          #0.01

    iter_num = 10000

    img = p.loadURDF("img.urdf", [x_img, y_img, z_img], globalScaling=1, useFixedBase=True)
    textureId = p.loadTexture("bubbly.jpg")
    p.changeVisualShape(img, -1, textureUniqueId=textureId)

    joints = np.array([0, 0.5, 0, -1.5, 0, 1.15, -math.pi/2])
    move(joints)

    X, Y, Z = gnr.generate_position(x_img, x_width, y_img, y_width, z_img + 0.20, z_width, iter_num)

    for k in range(iter_num):
        pos = (X[k], Y[k], Z[k])
        orn = [0.70710607, -0.70709531, -0.0029381,   0.00293214]

        joints = p.calculateInverseKinematics(kukaId, 6, pos, orn, maxNumIterations=100)

        move(joints)

        view_matrix, camvec = calc_view_matrix(pos, orn)
        img, pos1, camera_vector = img_wrapper(kukaId)
    #    time.sleep(1)
        d.save_data(kukaId, k)

if __name__ == '__main__':
    projection_matrix = calc_projection_matrix()

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

    create_dataset()

