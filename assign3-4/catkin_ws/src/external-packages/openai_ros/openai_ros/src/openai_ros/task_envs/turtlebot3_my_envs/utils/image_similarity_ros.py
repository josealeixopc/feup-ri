import os
import cv2

import subprocess, threading
import image_similarity

import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
images_relative_dir = "image_examples"
images_abs_dir = script_dir + os.path.sep + images_relative_dir

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

def compare_current_map_to_actual_map(actual_map_file):
    """
    Runs the map_server command to save the current image of the map runs comparison between the current
    robot's image and the actual map.

    Returns a float which represents the difference between the maps. A LOWER VALUE means the images are MORE SIMILAR.
    """
    current_map_file_location = "/tmp/current_tb3_map"


    command=Command("rosrun map_server map_saver -f {}".format(current_map_file_location))
    # command.run(timeout=10)

    current_map_image = cv2.imread(current_map_file_location + ".pgm", cv2.IMREAD_GRAYSCALE)
    actual_map_image = cv2.imread(actual_map_file, cv2.IMREAD_GRAYSCALE)

    # Rotate 90 degrees to the left by rotating 3 times to the right
    current_map_image = np.rot90(current_map_image, k=3)
    
    # Resize for test purposes 
    actual_map_image = cv2.resize(actual_map_image, (500, 500))
    # actual_map_image = cv2.cvtColor(actual_map_image, cv2.COLOR_BGR2GRAY)

    h, w = actual_map_image.shape[:2]
    
    # Get the borders of the area inside the map
    # Floodfill actual map area with value 128, i.e. mid-grey. 
    # Some pixels may have darker colors, so I went with 50.
    floodval = 50
    cv2.floodFill(actual_map_image, None, (h/2 + 10, w/2 + 10), floodval)

    # Extract filled area alone
    walkable_map = ((actual_map_image==floodval) * 255).astype(np.uint8)

    # Find edges of flooded area (walkable_map)
    walkable_map_edges = cv2.Canny(walkable_map, 100, 200)

    # Find black lines (edges) of the current map, but don't build contour
    current_map_edges = current_map_image.copy()
    current_map_edges[current_map_edges > 0] = 255
    current_map_edges = cv2.bitwise_not(current_map_edges)

    # Cropping final images
    x, y, w, h = cv2.boundingRect(current_map_edges)
    current_map_edges = current_map_edges[y:y+h, x:x+w]

    x, y, w, h = cv2.boundingRect(walkable_map_edges)
    walkable_map_edges = walkable_map_edges[y:y+h, x:x+w]

    # cv2.imshow("Walkable map", walkable_map)
    # cv2.imshow("Current Map", current_map_image)
    # cv2.imshow("Walkable Map Edges", walkable_map_edges)
    # cv2.imshow("Current Map Edges", current_map_edges)
    # cv2.waitKey(0)

    image_diff = image_similarity.my_compare_images(walkable_map_edges, current_map_edges)
    return image_diff

if __name__ == "__main__":
    compare_current_map_to_actual_map(images_abs_dir + os.path.sep + "turtlebot3_house_map.pgm")