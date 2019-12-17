import cv2
import numpy as np
import os
import logging

"""Script to return the degree of similarity of two images.

Based on this: https://www.youtube.com/watch?v=9mQznoHk4mU
"""

### LOGGER BEGIN
# Create a custom logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# add formatter to ch
ch.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)
### LOGGER END

script_dir = os.path.dirname(os.path.abspath(__file__))
images_relative_dir = "image_examples"
images_abs_dir = script_dir + os.path.sep + images_relative_dir

def resize_proportionally(image, scale):
    return cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))

def compare_images(image1_file, image2_file):

    logger.debug("Loading both images for comparison.")

    image1 = cv2.imread(images_abs_dir + os.path.sep + image1_file)
    image2 = cv2.imread(images_abs_dir + os.path.sep + image2_file)

    # TEMPLATE MACTHING works only if the image has the same size or else really similiar.

    # FEATURE DETECTION/MATCHING allows comparing images with different sizes, colors, etc...
    # We will use the SIFT algorithm for feature detection

    logger.debug("Executing SIFT...")

    sift = cv2.xfeatures2d.SIFT_create()
    kp_1, desc_1 = sift.detectAndCompute(image1, None)
    kp_2, desc_2 = sift.detectAndCompute(image2, None)

    # kp (key point) is where the feature is detected
    # desc (descriptor) is the way OpenCV describes a features (mathematically)

    logger.debug("Finished SIFT.")

    logger.debug("Executing key points and descriptors comparison...")

    index_params = dict(algorithm=0, trees=5)
    search_params = dict()

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(desc_1, desc_2, k=2)
    # Some of these matches are not "good"

    good_points = []

    # ratio test (low ratio means fewer but more precise matches; high ratio means the opposite)
    ratio = 0.4

    for m, n in matches:
        if m.distance < ratio * n.distance:
            good_points.append(m)

    logger.debug("Finished key points and descriptors comparison.")


    # Calculate the percentage of good_points in relation to the lowest available number of key points
    # If the lowest number of key points in the images is X, then the maximum value for good points is X

    max_number_keypoints = max([len(kp_1), len(kp_2)])
    estimated_similarity_ratio = len(good_points) / float(max_number_keypoints) # cast to float so result is float too

    logger.info("Estimated similarity ratio: {}".format(estimated_similarity_ratio))

    window_size = (500, 900)

    result = cv2.drawMatches(image1, kp_1, image2, kp_2, good_points, None)
    cv2.imshow("Result", cv2.resize(result, window_size))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return estimated_similarity_ratio


if __name__ == "__main__":
    compare_images("turtlebot3_house_map_half.pgm", "turtlebot3_house_map.pgm")
