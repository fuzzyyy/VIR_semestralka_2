import numpy as np
import pybullet as p

import os, glob, random
import math


def rot_x_ax(phi):
    R = np.array([[1,      0,                     0],
                  [0, math.cos(phi), -math.sin(phi)],
                  [0, math.sin(phi),  math.cos(phi)]])
    return R


def rot_y_ax(phi):
    if phi > np.pi:
        phi = -2*np.pi+phi

    R = np.array([[math.cos(phi), 0, -math.sin(phi)],
                  [0,             1,              0],
                  [math.sin(phi), 0,  math.cos(phi)]])
    return R


def rot_z_ax(phi):
    R = np.array([[math.cos(phi), -math.sin(phi), 0],
                  [math.sin(phi),  math.cos(phi), 0],
                  [0,                    0,       1]])
    return R


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)

def euler_angles_from_rotation_matrix(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return psi, theta, phi


def get_angle_dateset(mu=0, sigma=0.1, sample_size=1000):
    x_angle_diff = np.random.normal(mu, sigma, sample_size)
    y_angle_diff = np.random.normal(mu, sigma, sample_size)
    z_angle_diff = np.random.normal(mu, sigma, sample_size)

    eulers = []

    for i in range(sample_size):
        R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        R = np.matmul(R, rot_z_ax(np.pi/2 + z_angle_diff[i]))
        R = np.matmul(R, rot_y_ax(np.pi + y_angle_diff[i]))
        R = np.matmul(R, rot_x_ax(x_angle_diff[i]))
        eulers.append(euler_angles_from_rotation_matrix(R))

    return np.array(eulers)


if __name__ == '__main__':
    print(get_angle_dateset()[0:5])