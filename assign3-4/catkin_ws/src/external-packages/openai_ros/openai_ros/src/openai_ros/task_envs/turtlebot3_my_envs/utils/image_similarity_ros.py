import os
import cv2
import ntpath

import subprocess, threading
import image_similarity

import numpy as np
import roslaunch
import rospy

script_dir = os.path.dirname(os.path.abspath(__file__))
images_relative_dir = "images"
images_abs_dir = script_dir + os.path.sep + images_relative_dir

def path_leaf(path):
    head, tail = ntpath.split(path)
    return tail or ntpath.basename(head)

class Command(object):
    def __init__(self, cmd):
        self.cmd = cmd
        self.process = None

    def run(self, timeout):
        def target():
            print ('Thread started for command: "{}"').format(self.cmd)
            self.process = subprocess.Popen(self.cmd, shell=True)
            self.process.communicate()
            print ('Thread finished for command: "{}"').format(self.cmd)

        thread = threading.Thread(target=target)
        thread.start()

        thread.join(timeout)
        if thread.is_alive():
            print ('Terminating process for command: "{}"').format(self.cmd)
            self.process.terminate()
            thread.join()
        print ('Process "{}" terminated with code: {}'.format(self.cmd, self.process.returncode))

def fill(data, start_coords, fill_value, border_value, connectivity=8):
    """
    Flood fill algorithm

    Parameters
    ----------
    data : (M, N) ndarray of uint8 type
        Image with flood to be filled. Modified inplace.
    start_coords : tuple
        Length-2 tuple of ints defining (row, col) start coordinates.
    fill_value : int
        Value the flooded area will take after the fill.
    border_value: int
        Value of the color to paint the borders of the filled area with.
    connectivity: 4 or 8
        Connectivity which we use for the flood fill algorithm (4-way or 8-way).
        
    Returns
    -------
    filled_data: ndarray
        The data with the filled area.
    borders: ndarray
        The borders of the filled area painted with border_value color.
    """
    assert connectivity in [4,8]

    filled_data = data.copy()

    xsize, ysize = filled_data.shape
    orig_value = filled_data[start_coords[0], start_coords[1]]

    stack = set(((start_coords[0], start_coords[1]),))
    if fill_value == orig_value:
        raise ValueError("Filling region with same value already present is unsupported. Did you already fill this region?")

    border_points = []

    while stack:
        x, y = stack.pop()

        if filled_data[x, y] == orig_value:
            filled_data[x, y] = fill_value
            if x > 0:
                stack.add((x - 1, y))
            if x < (xsize - 1):
                stack.add((x + 1, y))
            if y > 0:
                stack.add((x, y - 1))
            if y < (ysize - 1):
                stack.add((x, y + 1))

            if connectivity == 8:
                if x > 0 and y > 0:
                    stack.add((x - 1, y - 1))
                if x > 0 and y < (ysize - 1):
                    stack.add((x - 1, y + 1))
                if x < (xsize - 1) and y > 0:
                    stack.add((x + 1, y - 1))
                if x < (xsize - 1) and y < (ysize - 1):
                    stack.add((x + 1, y + 1))
        else:
            if filled_data[x, y] != fill_value:
                border_points.append([x,y])
    
    # Fill all image with white
    borders = filled_data.copy()
    borders.fill(255)

    # Paint borders
    for x,y in border_points:
        borders[x, y] = border_value
    
    return filled_data, borders


def compare_current_map_to_actual_map(current_map_file_location, walkable_map_file):
    """
    Runs the map_server command to save the current image of the map runs comparison between the current
    robot's image and the actual map.

    Returns a float which represents the difference between the maps. A LOWER VALUE means the images are MORE SIMILAR.
    """

    walkable_map_file_location = images_abs_dir + os.path.sep + walkable_map_file

    # If map_saver could not save a map, return maximum difference
    if not os.path.exists(current_map_file_location + ".pgm"):
        return 1

    current_map_image = cv2.imread(current_map_file_location + ".pgm", cv2.IMREAD_GRAYSCALE)
    walkable_map_image = cv2.imread(walkable_map_file_location, cv2.IMREAD_GRAYSCALE)

    # Rotate 90 degrees to the left by rotating 3 times to the right
    current_map_image = np.rot90(current_map_image, k=3)

    # Convert current estimate to binary
    current_walkable_map = ((current_map_image==254) * 255).astype(np.uint8)

    # Cropping final images
    x1, y1, w1, h1 = cv2.boundingRect(current_walkable_map)
    current_walkable_map = current_walkable_map[y1:y1+h1, x1:x1+w1]

    x2, y2, w2, h2 = cv2.boundingRect(walkable_map_image)
    walkable_map_image = cv2.resize(walkable_map_image[y2:y2+h2, x2:x2+w2], (h1, w1))

    # cv2.imshow("Current", current_walkable_map)
    # cv2.imshow("Actual", walkable_map_image)
    # cv2.waitKey(0)

    # compare_images returns a value between 0 and 1, but may return 1000, so we choose the min of the two
    image_diff = min(1, image_similarity.my_compare_images(walkable_map_image, current_walkable_map)) 
    return image_diff

def generate_walkable_area_image(actual_map_file, floodfill_x, floodfill_y, map_width=384, map_height=384):
    actual_map_image = cv2.imread(actual_map_file, cv2.IMREAD_GRAYSCALE)
    
    # Get the borders of the area inside the map
    # Floodfill actual map area with value 128, i.e. mid-grey. 
    # Some pixels may have darker colors, so I went with 50.
    floodval = 50
    cv2.floodFill(actual_map_image, None, (floodfill_x, floodfill_y), floodval)

    # Extract filled area alone
    walkable_map = ((actual_map_image==floodval) * 255).astype(np.uint8)

    cv2.imshow("Walkable", cv2.resize(walkable_map, (500, 500), interpolation = cv2.INTER_NEAREST))

    x, y, w, h = cv2.boundingRect(walkable_map)

    cropped_walkable_map = cv2.resize(walkable_map[y:y+h, x:x+w], (map_height, map_width), interpolation = cv2.INTER_NEAREST)

    cv2.imshow("Flooded", cv2.resize(actual_map_image, (500, 500), interpolation = cv2.INTER_NEAREST))

    cv2.imshow("Cropped Walkable Map", cropped_walkable_map)
    cv2.waitKey(0)
    
    filename, file_extension = os.path.splitext(path_leaf(actual_map_file))
    cv2.imwrite(images_abs_dir + os.path.sep + filename + "_walkable.pgm", cropped_walkable_map)

def get_number_of_almost_white_pixels(image_file):
    file_location = images_abs_dir + os.path.sep + image_file

    image = cv2.imread(file_location, cv2.IMREAD_GRAYSCALE)
    return np.sum(image >= 250)

def get_number_of_almost_white_pixels_current_map(current_map_file_location):
    # If map_saver could not save a map, return maximum difference
    if not os.path.exists(current_map_file_location + ".pgm"):
        return 1

    current_map_image = cv2.imread(current_map_file_location + ".pgm", cv2.IMREAD_GRAYSCALE)
    h, w = current_map_image.shape

    # Rotate 90 degrees to the left by rotating 3 times to the right
    current_map_image = np.rot90(current_map_image, k=3)

    # Convert current estimate to binary
    current_walkable_map = ((current_map_image==254) * 255).astype(np.uint8)

    # Cropping final images
    x1, y1, w1, h1 = cv2.boundingRect(current_walkable_map)
    current_walkable_map = cv2.resize(current_walkable_map[y1:y1+h1, x1:x1+w1], (h, w))

    return np.sum(current_walkable_map >= 250)

if __name__ == "__main__":
    current_map_file_location = "/tmp/ros_merge_map"
    print("Image difference: {}".format(compare_current_map_to_actual_map(current_map_file_location, "turtlebot3_world_map_walkable.pgm")))
    # generate_walkable_area_image("/home/jazz/Projects/FEUP/ProDEI/feup-ri/assign3-4/catkin_ws/src/external-packages/openai_ros/openai_ros/src/openai_ros/task_envs/turtlebot3_my_envs/utils/images/house-1.pgm", 1450, 1700)
    # print(get_number_of_almost_white_pixels("turtlebot3_world_map_walkable.pgm"))