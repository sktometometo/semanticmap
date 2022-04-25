import rospy
import numpy as np


def calc_cell_semantics(pixel, semantics, allow_diff=3):
    for label, value in semantics.items():
        if np.linalg.norm(np.array(pixel) - np.array(value)) < allow_diff:
            return label
    rospy.logerr('pixel value {} is unknown'.format(pixel))
    return 'unknown'


def convert_image_to_semantic_map(raw_image, semantics):

    if len(raw_image.shape) != 3:
        print()

    height = raw_image.shape[0]
    width = raw_image.shape[1]

    semantic_map = [[calc_cell_semantics(raw_image[i][j], semantics)
                     for i in range(height)] for j in range(width)]

    return semantic_map


def convert_semantic_map_to_image(semantic_map, semantics):

    height = len(semantic_map[0])
    width = len(semantic_map)
    image = [[semantics[semantic_map[i][j]]
              for i in range(height)] for j in range(width)]
    return np.array(image, dtype='uint8')
