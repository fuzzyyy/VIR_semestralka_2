import numpy as np
import math

sample_size = 1000
#params =
# ...





#from https://towardsdatascience.com/a-python-tutorial-on-generating-and-plotting-a-3d-guassian-distribution-8c6ec6c41d03
def generate_position(x_mean, x_width, y_mean, y_width, z_mean, z_width, size = 10000):
    X = np.random.normal(x_mean, x_width, size)
    Y = np.random.normal(y_mean, y_width, size)
    Z = np.random.normal(z_mean, z_width, size)
    return  X, Y, Z

def generate_orientation():

    return
