import numpy as np

"""
This module includes functions for scaling values to a custom scale.

Based on this: https://stats.stackexchange.com/a/281164
"""

def scale_scalar(scalar, min_value, max_value, new_min_value, new_max_value):
    """
    Rescales a scalar value.
    
    Arguments:
        scalar -- The value.
        min_value -- The minimum value of the current scale.
        max_value -- The maximum value of the current scale.
        new_min_value -- The minimum value of the new scale.
        new_max_value -- The maximum value of the new scale.
    
    Returns:
        The value adusted to the new scale.
    """
    return (m - min_value) / (max_value - min_value) * (new_max_value - new_min_value) + new_min_value

def scale_arr(ndarr, min_value, max_value, new_min_value, new_max_value):
    new_ndarr = np.copy(ndarr)

    with np.nditer(map_data_arr, op_flags=['readwrite']) as it:
        for x in it:
            x[...] = scale_scalar(x, min_value, max_value, new_min_value, new_max_value)