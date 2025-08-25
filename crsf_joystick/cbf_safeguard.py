import numpy as np
import cvxpy as cp
from cv2 import threshold


def cbf_filter(current_positions,alpha=1.0):
    x_min = -4.5
    x_max = 3.0
    y_min = 0.0
    y_max = 2.0
    z_min = -2.0
    z_max = 3.0
    thresholds= [0.3,0.2,0.3] #x,y,z
    u = cp.Variable(3)

    cbf_trigger= False
    constraints = []
    flags=[]
    h_front= x_max-current_positions[0]
    h_back = current_positions[0]-x_min
    if h_front <=thresholds[0]:
        constraints.append(u[0]+alpha*h_front>=0)
        h_front_flag=True
    else:
        h_front_flag=False
    if h_back <=thresholds[0]:
        constraints.append(u[0]+alpha*h_back>=0)
        h_back_flag = True
    else:
        h_back_flag = False

    h_up = y_max-current_positions[1]
    h_down = current_positions[1]-y_min
    if h_up <=thresholds[1]:
        constraints.append(u[1]+alpha*h_up>=0)
        h_up_flag = True
    else:
        h_up_flag = False
    if h_down <=thresholds[1]:
        constraints.append(u[1]+alpha*h_down>=0)
        h_down_flag = False
    else:
        h_down_flag = False

    h_right = z_max-current_positions[2]
    h_left = current_positions[2]-z_min
    if h_right <=thresholds[2]:
        constraints.append(u[2]+alpha*h_right>=0)
        h_right_flag = True
    else:
        h_right_flag = False
    if h_left <=thresholds[2]:
        constraints.append(u[2]+alpha*h_left>=0)
        h_left_flag = True
    else:
        h_left_flag = False



    flags = [h_front_flag, h_back_flag, h_up_flag, h_down_flag, h_right_flag, h_left_flag]
    constraints_exist = any(flags)

    if constraints_exist:
        cbf_trigger = True
    else:
        cbf_trigger = False
    return cbf_trigger