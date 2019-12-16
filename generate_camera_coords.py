import numpy as np
import math

sample_size = 1000
#params =
# ...





#from https://towardsdatascience.com/a-python-tutorial-on-generating-and-plotting-a-3d-guassian-distribution-8c6ec6c41d03
def py_bivariate_normal_pdf(domain, mean, variance):
    #TODO: rework: https://docs.scipy.org/doc/numpy-1.15.0/reference/generated/numpy.random.normal.html
    X = [[-mean+x*variance for x in range(int((-domain+mean)//variance),
                                                   int((domain+mean)//variance)+1)]
                  for _ in range(int((-domain+mean)//variance),
                                 int((domain+mean)//variance)+1)]
    Y = [*map(list, zip(*X))]
    R = [[math.sqrt(a**2 + b**2) for a, b in zip(c, d)] for c, d in zip(X, Y)]
 #   Z = [[(1. / math.sqrt(0.5 * math.pi)) * math.exp(-11.25*r**2) for r in r_sub] for r_sub in R] #30cm
#    Z = [[(1. / math.sqrt(2 * math.pi)) * math.exp(-.5 * r ** 2) for r in r_sub] for r_sub in R]
    Z = [[(1. / math.sqrt(0.5 * math.pi)) * math.exp(-13.5 * r ** 2) for r in r_sub] for r_sub in R]#20cm
    X = [*map(lambda a: [b+mean for b in a], X)]
    Y = [*map(lambda a: [b+mean for b in a], Y)]
    return  np.array(X), np.array(Y), np.array(Z)

def generate_orientation():

    return