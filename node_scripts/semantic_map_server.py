#!/usr/bin/env python

import rospy
import cv2
import os
import yaml
import PyKDL

from semanticmap import convert_image_to_semantic_map
from semanticmap import convert_semantic_map_to_image

from semanticmap.msg import GridCell
from semanticmap.msg import SemanticMapGrid
from semanticmap.msg import SemanticMapMetaData


def main():

    rospy.init_node('semantic_map_server')
    map_info_path = rospy.get_param('~map_info')
    semantics = rospy.get_param('~semantics')
    semantics['unknown'] = [255, 255, 255]

    with open(map_info_path) as f:
        map_info = yaml.load(f, Loader=yaml.Loader)
    map_image_basename = map_info['image']
    resolution = map_info['resolution']
    origin = map_info['origin']
    map_id = map_info['map_id']

    map_info_dir = os.path.dirname(map_info_path)
    map_image_path = os.path.join(map_info_dir, map_image_basename)
    raw_image = cv2.cvtColor(
        cv2.imread(map_image_path),
        cv2.COLOR_BGR2RGB
    )

    semantic_map = convert_image_to_semantic_map(
        raw_image,
        semantics)

    msg_semantic_map_grid = SemanticMapGrid()
    msg_semantic_map_meta_data = SemanticMapMetaData()

    msg_semantic_map_meta_data.map_load_time = rospy.Time.now()
    msg_semantic_map_meta_data.resolution = resolution
    msg_semantic_map_meta_data.width = len(semantic_map[0])
    msg_semantic_map_meta_data.height = len(semantic_map)
    msg_semantic_map_meta_data.origin.position.x = origin[0]
    msg_semantic_map_meta_data.origin.position.y = origin[1]
    msg_semantic_map_meta_data.origin.orientation.x = PyKDL.Rotation.RotZ(
        origin[2]).GetQuaternion()[0]
    msg_semantic_map_meta_data.origin.orientation.y = PyKDL.Rotation.RotZ(
        origin[2]).GetQuaternion()[1]
    msg_semantic_map_meta_data.origin.orientation.z = PyKDL.Rotation.RotZ(
        origin[2]).GetQuaternion()[2]
    msg_semantic_map_meta_data.origin.orientation.w = PyKDL.Rotation.RotZ(
        origin[2]).GetQuaternion()[3]

    msg_semantic_map_grid.header.stamp = msg_semantic_map_meta_data.map_load_time
    msg_semantic_map_grid.header.frame_id = map_id
    msg_semantic_map_grid.info = msg_semantic_map_meta_data
    for index_height in range(msg_semantic_map_meta_data.height):
        for index_width in range(msg_semantic_map_meta_data.width):
            msg_semantic_map_grid.data.append(
                GridCell(
                    semantics_name=semantic_map[index_height][index_width]
                )
            )

    # converted_image = convert_semantic_map_to_image(
    #     semantic_map,
    #     semantics)
    # cv2.imshow('original', cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR))
    # cv2.imshow('converted', cv2.cvtColor(converted_image, cv2.COLOR_RGB2BGR))
    # cv2.waitKey(0)

    pub_semantic_map_grid = rospy.Publisher(
        '~semantic_map', SemanticMapGrid, queue_size=1, latch=True)
    pub_semantic_map_meta_data = rospy.Publisher(
        '~semantic_map_meta_data', SemanticMapMetaData, queue_size=1, latch=True)

    pub_semantic_map_grid.publish(msg_semantic_map_grid)
    pub_semantic_map_meta_data.publish(msg_semantic_map_meta_data)

    rospy.loginfo('initialized')

    rospy.spin()


if __name__ == '__main__':
    main()
