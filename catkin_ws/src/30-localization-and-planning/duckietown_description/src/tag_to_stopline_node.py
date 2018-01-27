#!/usr/bin/env python

import csv
import re
from collections import namedtuple

import rospkg
import rospy
from duckietown_msgs.srv import GetStopLineForTag

__author__ = 'Xomak'


class TagToStoplineNode(object):
    STOPLINE_COLUMN = 5
    TAGID_COLUMN = 0
    STOPLINE_TEMPLATE = "stopline_{x}_{y}_{index}"
    STOPLINE_REGEX = re.compile("\((?P<x>\d+) (?P<y>\d+) (?P<index>\d+)\)")

    StopLine = namedtuple('StopLine', ('x', 'y', 'index'))

    def __init__(self):
        self.node_name = rospy.get_name()
        self.package_path = rospkg.RosPack().get_path('duckietown_description')
        self.tag_map_csv = rospy.get_param('~tag_map_csv')
        rospy.Service("~tag_to_stopline", GetStopLineForTag, self.on_request)

        self.tags = {}
        rospy.loginfo("[%s] Initialized." % self.node_name)
        self.construct_from_csv()

    def construct_from_csv(self):
        csv_file = open(self.tag_map_csv)
        csv_reader = csv.reader(csv_file, delimiter=',')
        for index, row in enumerate(csv_reader):
            if index == 0:
                continue
            tag_id = int(row[self.TAGID_COLUMN])
            description = row[self.STOPLINE_COLUMN].strip()
            m = self.STOPLINE_REGEX.match(description)
            if m:
                self.tags[tag_id] = self.StopLine(int(m.group('x')), int(m.group('y')), int(m.group('index')))
                rospy.loginfo("[{}] Add tag: {} -> {}".format(self.node_name, tag_id, self.tags[tag_id]))
        csv_file.close()

    def on_request(self, req):
        if req.tag_id in self.tags:
            stopline = self.tags[req.tag_id]
            stopline_id = self.STOPLINE_TEMPLATE.format(x=stopline.x, y=stopline.y, index=stopline.index)
        else:
            stopline_id = ""
        return {'stopline': stopline_id}

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % self.node_name)


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('tag_to_stopline_node', anonymous=False)

    # Create the NodeName object
    node = TagToStoplineNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
