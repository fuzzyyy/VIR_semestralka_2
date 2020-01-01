import pybullet as p
import pybullet_data
import time
import numpy as np

import os, glob, random
import math

from pickle_testing import data_management as d
from camera_angle_generator import *
from generate_camera_coords import *

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
 #   _, _, _, _, position, orientation = p.getLinkState(robot, p.getNumJoints(robot) - 1, computeForwardKinematics=True)
    return position, orientation

def img_wrapper(robot):
    pos, orn, _, _, _, _ = p.getLinkState(robot, p.getNumJoints(robot) - 1, computeForwardKinematics=True)
    view_matrix, camvec = calc_view_matrix(pos, orn)
    return p.getCameraImage(128, 128, view_matrix, projection_matrix), pos, camvec

def my_getCameraImage(view_matrix, projection_matrix):
    return p.getCameraImage(128, 128, view_matrix, projection_matrix)

def gaussian(x, mu=0.0, sigma=1.0):
    x = float(x - mu) / sigma
    return math.exp(-x*x/2.0) / math.sqrt(2.0*math.pi) / sigma

def deg2rad(deg):
    return deg*math.pi/180

def rad2deg(rad):
    return rad/math.pi*180

def move(joints):
    p.setJointMotorControlArray(kukaId, range(numjoints), p.POSITION_CONTROL, targetPositions=joints)
    for _ in range(100):
        p.stepSimulation()

def add_const(joints):
     joints = np.array([joints[0] - 4.608852735626947e-07, joints[1] - 0.02198945545371711,
                        joints[2] - 3.27422148943968e-05,joints[3] + 0.01983139924852373,
                        joints[4] + 5.821055593944444e-07, joints[5] + 0.04170378438381217,
                                                        joints[6] - 4.4836419497817914e-05])
def rot_x(phi):
    R = np.matrix([[1,      0,              0,        0],
                   [0, math.cos(phi), -math.sin(phi), 0],
                   [0, math.sin(phi),  math.cos(phi), 0],
                   [0,      0,              0,        1]])
    return R

def random_texture(fig):
    texture_paths = glob.glob(os.path.join('dtd', '**', '*.jpg'), recursive=True)
    random_texture_path = texture_paths[random.randint(0, len(texture_paths) - 1)]
    textureId = p.loadTexture(random_texture_path)
    p.changeVisualShape(fig, -1, textureUniqueId=textureId)

def calc_angle(a, b):
    c = math.sqrt(pow(a,2)+pow(b,2))
    alpha = math.asin(a/c)
    return alpha

def camera_coords_in_img_coords(): #TODO:
    return

def create_dataset():
    length_img = 0.15
    width_img = 0.10

    x_img = 0.58
    y_img = 0
    z_img = 0.25

    # 20cm:
    domain = 0.1
    mean = 0
    step = 0.01

    iter_num = (int)(domain*2/step)

    #img = p.loadURDF("img.urdf", [x_img, y_img, z_img], globalScaling=1, useFixedBase=True)

 #   random_texture(img)

    joints = np.array([0, 0.5, 0, -1.5, 0, 1.15, -math.pi/2])

    move(joints)

    for i in range(10):
        print("----------Interation", i)

        pos, orn = my_getLinkState(kukaId)
        euler = p.getEulerFromQuaternion(orn)
        print('\n\n\n\n','euler: ', euler, '\n\n\n\n\n')
        pos = np.array(pos)
        orn = np.array(orn)
        [*args, camvec] = calc_view_matrix(pos, orn)
        print('cam vec: ', camvec)

        print('orn: ',orn)
        if i == 0:
            pos[0] = pos[0] + 0.015
            pos[2] = pos[2] - 0.03

        if i < 2:
            joints = p.calculateInverseKinematics(kukaId, 6, pos, orn, maxNumIterations=100)
  #          add_const(joints)

            view_matrix, camvec = calc_view_matrix(pos, orn)
      #      img = my_getCameraImage(view_matrix, projection_matrix)
            img, pos, camera_vector = img_wrapper(kukaId)

            move(joints)
            time.sleep(1)

     #       d.save_data(kukaId, i)

        if i >= 2:
   #         X,Y,Z = py_bivariate_normal_pdf(0.15, 0, .01)#30cm
            X, Y, Z = py_bivariate_normal_pdf(domain, mean, step)

            for k in range(iter_num+2):
                pos, orn = my_getLinkState(kukaId)
                pos = np.array(pos)
                vys = pos[2] - z_img
                ax = abs(pos[1] - y_img)
                x = calc_angle(ax, vys)

                for j in range(iter_num+2):
                    pos, orn = my_getLinkState(kukaId)
                    pos = np.array(pos)

                    pos[0] = X[k][j] + x_img
                    pos[1] = Y[k][j] + y_img
                    pos[2] = Z[k][j] + z_img + 0.07 - 0.5921   #-0.4968752
                #    orn = [0.70710607, -0.70709531, -0.0029381,   0.00293214]

                    vys = pos[2] - z_img

                    az = abs(pos[0] - x_img)
                    ax = abs(pos[1] - y_img)

                    z = calc_angle(az, vys)

                    if j == 0:
                        euler = p.getEulerFromQuaternion(orn)
                        print('\n\n\n\n', euler, '\n\n\n\n\n')
                        euler = np.array(euler)
                        if  k == 0:
                            euler0 = euler[0]
                            euler1 = euler[1]
                            x = calc_angle(ax, 0.07)
                        euler[0] = euler0 + z
                        if k <= iter_num/2:
                            euler[1] = euler1 + x
                        else:
                            euler[1] = euler1 - x
                        orn = p.getQuaternionFromEuler(euler)

                    if i < 2:
                        joints = p.calculateInverseKinematics(kukaId, 6, pos, orn, maxNumIterations=100)
                        #          add_const(joints)

                        view_matrix, camvec = calc_view_matrix(pos, orn)
                        #      img = my_getCameraImage(view_matrix, projection_matrix)
                        img, pos, camera_vector = img_wrapper(kukaId)

                        move(joints)
                        time.sleep(1)

                    if j > 0:
                        euler = p.getEulerFromQuaternion(orn)
                        euler = np.array(euler)
                        if j <= iter_num/2:
                            euler[0] = euler0 + z
                        else:
                            euler[0] = euler0 - z
                        orn = p.getQuaternionFromEuler(euler)

                    euler = (3.133269383433572, 1.54354483102075075, -2.0708120545837738)
                    euler = get_angle_dateset()
                    orn = p.getQuaternionFromEuler(euler)
                    print('sdfgsdg', orn)
                    print(p.getEulerFromQuaternion(orn))
                    joints = p.calculateInverseKinematics(kukaId, 6, pos, orn, maxNumIterations=100)
          #          add_const(joints)
                    move(joints)

                    view_matrix, camvec = calc_view_matrix(pos, orn)
           #         img = my_getCameraImage(view_matrix, projection_matrix)
                    img, pos1, camera_vector = img_wrapper(kukaId)
                    time.sleep(1)
                    d.save_data(kukaId, i)


def camera_test():
    length_img = 0.15
    width_img = 0.10

    x_img = 0.58
    y_img = 0
    #z_img = 0.25
    z_img = 0


    # 20cm:
    domain = 0.1
    mean = 0
    step = 0.01

    iter_num = (int)(domain*2/step)

    img = p.loadURDF("img.urdf", [x_img, y_img, z_img], globalScaling=1, useFixedBase=True)

 #   random_texture(img)

    joints = np.array([0, 0.5, 0, -1.5, 0, 1.15, -math.pi/2])

    move(joints)

    eulers = get_angle_dateset(sample_size=10)

    for i in range(10):
        print("----------Interation", i)

        #pos, orn = my_getLinkState(kukaId)
        #euler = p.getEulerFromQuaternion(orn)


        # --- set euler ---
        euler = eulers[i]
        print('\n', 'eulers[i]: ', eulers[i], '\n')

        # --- set pos ---
        pos = np.array([x_img, y_img, 0.2])

        orn = p.getQuaternionFromEuler(eulers[i])

        #[*args, camvec] = calc_view_matrix(pos, orn)
        #print('cam vec: ', camvec)

        joints = p.calculateInverseKinematics(kukaId, 6, pos, orn, maxNumIterations=200)
        add_const(joints)
        move(joints)

        view_matrix, camvec = calc_view_matrix(pos, orn)
        #img = my_getCameraImage(view_matrix, projection_matrix)
        img, pos1, camera_vector = img_wrapper(kukaId)
        pos, orn = my_getLinkState(kukaId)
        print('sdfgsdfg ', pos, orn)
        time.sleep(1)
        d.save_data(kukaId, i)





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

    camera_test()

