#!/usr/bin/env python
import sys

import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np

def bin_ndarray(ndarray, new_shape, operation='sum'):
    """
    From here: https://stackoverflow.com/questions/8090229/resize-with-averaging-or-rebin-a-numpy-2d-array

    Bins an ndarray in all axes based on the target shape, by summing or
        averaging.

    Number of output dimensions must match number of input dimensions and 
        new axes must divide old ones.

    Example
    -------
    >>> m = np.arange(0,100,1).reshape((10,10))
    >>> n = bin_ndarray(m, new_shape=(5,5), operation='sum')
    >>> print(n)

    [[ 22  30  38  46  54]
     [102 110 118 126 134]
     [182 190 198 206 214]
     [262 270 278 286 294]
     [342 350 358 366 374]]

    """
    operation = operation.lower()
    if not operation in ['sum', 'mean']:
        raise ValueError("Operation not supported.")
    if ndarray.ndim != len(new_shape):
        raise ValueError("Shape mismatch: {} -> {}".format(ndarray.shape,
                                                           new_shape))
    compression_pairs = [(d, c//d) for d,c in zip(new_shape,
                                                  ndarray.shape)]
    flattened = [l for p in compression_pairs for l in p]
    ndarray = ndarray.reshape(flattened)
    for i in range(len(new_shape)):
        op = getattr(ndarray, operation)
        ndarray = op(-1*(i+1))
    return ndarray

def simplify_occupancy_grid(map_data, dim=4):
    map_data_arr = np.array(map_data.data)

    # reshape map_data to 2D array
    reshaped_data = np.reshape(map_data_arr, (map_data.info.height, map_data.info.width))    

    # decrease size of 2D array by summing/averaging neighbor cells
    binned_data = bin_ndarray(reshaped_data, (dim, dim), operation='sum')

    return binned_data


def callback(map_data):
    
    print simplify_occupancy_grid(map_data)

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('map', OccupancyGrid, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()