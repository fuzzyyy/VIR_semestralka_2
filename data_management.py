import pybullet as p
import pybullet_data
import numpy as np
import pickle

from dataset import *

jointCount = 7
default_dir = "data/"


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


def compare_joints(robot, file_name):
    data = load_pickle(file_name)
    joints = get_joints(robot)
    for i in range(jointCount):
        diff = joints[i] - data['joints'][i]
        print(f"joint {i}: pickle: {data['joints'][i]}     original: {joints[i]}     difference: {diff}")


def get_joints(robot):
    joints = p.getJointStates(robot, range(jointCount))
    states = [0 for i in range(jointCount)]
    for i in range(jointCount):
        states[i] = joints[i][0]
    return states


def get_camera_pos(robot):
    pos, orn = my_getLinkState(robot)
    pos_to_img = camera_coords_in_img_coords(pos)
    return pos_to_img


def get_camera_angle(robot):
    pos, orn = my_getLinkState(robot)
    return orn


def get_image(img_data):
    image_size = [img_data[0], img_data[1]]
    img_data = img_data[2]
    image_RGB = img_data[:, :, :3]

    return image_size, image_RGB


def save_pickle(data, file_name, dir=default_dir):
    #try:
    file = open(dir + file_name + ".p", "wb")
    pickle.dump(data, file)
    #except:
    #    print(f"saving file \"{file_name}\" failed")


def load_pickle(file_name, dir=default_dir):
    file = open(dir + file_name + ".p", "rb")
    data = pickle.load(file)
    return data
    #print(f"loading file \"{file_name}\" failed")
    #return None


def save_data(robot, num, img_data, dir=default_dir):
    data = {}
    data["joints"] = get_joints(robot)
    data["camera_pos"] = get_camera_pos(robot)
    data["camera_angle"] = get_camera_angle(robot)
    img_size, img = get_image(img_data)
    data["image_size"] = img_size
    data["image"] = img

    name = f'data{num}'
    save_pickle(data, name, dir)
    print(f'Dataset {name} saved successfully!')


def test_save():
    sample_size = 10

    joint_data = [[(0.1, 0.2, (0.3, 0.4, 0.5, 0.6, 0.7, 0.8), 0.9) for i in range(jointCount)] for j in
                  range(sample_size)]

    data1 = {}
    data1["joins"] = joint_data[0]

    save_pickle(data1, 'data1')

    data2 = load_pickle('data1')

    print(data2)


def test_load():
    data = load_pickle("data10")
    for item in data:
        print(f'{item}: {data[item]}')


if __name__ == '__main__':          # test funkcionality on example data
    #test_save()
    test_load()
