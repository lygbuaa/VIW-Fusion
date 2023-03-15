#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq

#sita in degree
sita_list = np.array([0.00, 0.76, 1.52, 2.28, 3.03, 3.79, 4.55, 5.31, 6.07, 6.83, 7.58, 8.34, 9.10, 9.86, 10.62, 11.38, 12.14, 12.89, 13.65, 14.41, 15.17, 15.93, 16.69, 17.44, 18.20, 18.96, 19.72, 20.48, 21.24, 21.99, 22.75, 23.51, 24.27, 25.03, 25.79, 26.55, 27.30, 28.06, 28.82, 29.58, 30.34, 31.10, 31.85, 32.61, 33.37, 34.13, 34.89, 35.65, 36.41, 37.16, 37.92, 38.68, 39.44, 40.20, 40.96, 41.71, 42.47, 43.23, 43.99, 44.75, 45.51, 46.27, 47.02, 47.78, 48.54, 49.30, 50.06, 50.82, 51.57, 52.33, 53.09, 53.85, 54.61, 55.37, 56.13, 56.88, 57.64, 58.40, 59.16, 59.92, 60.68, 61.43, 62.19, 62.95, 63.71, 64.47, 65.23, 65.98, 66.74, 67.50, 68.26, 69.02, 69.78, 70.54, 71.29, 72.05, 72.81, 73.57, 74.33, 75.09, 75.84])
# angle_list = [sita/57.3 for sita in sita_list]
angle_list = np.deg2rad(sita_list) 
#radius with distortion, unit: mm, focal lenght: 2.666mm
rd_list = np.array([0.000, 0.035, 0.070, 0.106, 0.141, 0.176, 0.211, 0.246, 0.281, 0.316, 0.352, 0.387, 0.422, 0.457, 0.492, 0.527, 0.561, 0.596, 0.631, 0.666, 0.701, 0.735, 0.770, 0.805, 0.839, 0.874, 0.908, 0.943, 0.977, 1.011, 1.045, 1.080, 1.114, 1.148, 1.181, 1.215, 1.249, 1.283, 1.316, 1.350, 1.383, 1.416, 1.450, 1.483, 1.516, 1.549, 1.581, 1.614, 1.646, 1.679, 1.711, 1.743, 1.775, 1.807, 1.839, 1.871, 1.902, 1.934, 1.965, 1.996, 2.027, 2.058, 2.088, 2.119, 2.149, 2.179, 2.209, 2.239, 2.268, 2.297, 2.327, 2.356, 2.384, 2.413, 2.441, 2.470, 2.497, 2.525, 2.553, 2.580, 2.607, 2.634, 2.660, 2.687, 2.713, 2.739, 2.764, 2.790, 2.815, 2.839, 2.864, 2.888, 2.912, 2.936, 2.959, 2.982, 3.005, 3.027, 3.050, 3.071, 3.093])

# order-2 Kannala-Brandt model
def poly_func(para, x):
    k1, k3 = para
    return k1*x + k3*x**3

def fit_error(para, x, y):
    return poly_func(para, x) - y

'''
scipy.optimize.leastsq(func, x0, args=(), Dfun=None, full_output=0, col_deriv=0, ftol=1.49012e-08, xtol=1.49012e-08, gtol=0.0, maxfev=0, epsfcn=None, factor=100, diag=None)[source]
'''
def run_demo():
    x = np.linspace(-10, 10, 100)
    p_value = [1.0, 0.1]
    noise = np.random.randn(len(x))
    y = poly_func(p_value, x) + noise*2
    para0 = [2.0, 2.0]

    para = leastsq(func=fit_error, x0=para0, args=(x, y), full_output=True)
    print("leastsq output: {}".format(para))
    y_fitted = poly_func(para[0], x)
 
    plt.figure
    plt.plot(x, y, 'r', label = 'Original curve')
    plt.plot(x, y_fitted, '-b', label ='Fitted curve')
    plt.legend()
    plt.show()
    print("fitted poly params: {}".format(para[0]))

def calc_kb2():
    para0 = [2.6, -0.2]
    para = leastsq(func=fit_error, x0=para0, args=(angle_list, rd_list), full_output=False)
    print("leastsq output: {}".format(para))
    y_fitted = poly_func(para[0], angle_list)
 
    plt.figure
    plt.plot(angle_list, rd_list, 'ro', label = 'original curve')
    plt.plot(angle_list, y_fitted, '-b', label ='fitted curve')
    plt.legend()
    plt.show()
    #fitted poly params: [ 2.66822668 -0.18649677]
    print("fitted poly params: {}".format(para[0]))

if __name__ == "__main__":
    print("sita_list: {}, rd_list: {}".format(len(sita_list), len(rd_list)))
    # run_demo()
    calc_kb2()