# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pathlib
import unittest
import re

from test_msgs.msg import BasicTypes

import rclpy

TEST_NODE = 'my_publisher_node'
TEST_NAMESPACE = '/my_publisher_ns'

# TODO Add more topic styles
TEST_TOPIC = "my_topic"


def remote_topic_protocol(topic_name):
    if not re.match('rostopic://', topic_name):
        return topic_name
    return topic_name.lstrip('rostopic://')


def full_topic(topic_name):
    if not topic_name:
        raise (Exception("Invalid topic name, empty!"))
    if topic_name[0] == "/":
        return topic_name

    return TEST_NAMESPACE + "/" + topic_name


class TestPublisher(unittest.TestCase):

    @classmethod
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            'publisher_test_node',
            # namespace=TEST_NAMESPACE,
            context=self.context,
            cli_args=[
                '--ros-args', '-r', 'publisher_test_topic:=publisher_test_new_topic',
                '--ros-args', '-r', '{}:={}'.format(TEST_TOPIC, "new_topic"),
            ],
        )
        self.node_with_ns = rclpy.create_node(
            'publisher_test_node_ns',
            context=self.context,
            cli_args=[
                '--ros-args', '-r', 'publisher_test_topic:=publisher_test_new_topic',
                '--ros-args', '-r', '{}:={}'.format(TEST_TOPIC, "new_topic"),
            ],
        )

    @classmethod
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    @classmethod
    def gen_topic_name_pairs(self):
        original_topics = ['my_topic', '/my_topic', '/my_ns/my_topic', 'rostopic://my_topic',
                           'rostopic:///my_ns/my_topic']
        topic_name_pairs = dict()
        for original_topic in original_topics:
            topic_name_pairs[original_topic] = full_topic(original_topic)

    def test_resolved_name(self):
        publisher = self.node.create_publisher(BasicTypes, TEST_TOPIC, 0)
        print(publisher.topic_name, 'vs', full_topic("new_topic"))
        assert publisher.topic_name == full_topic("new_topic")


if __name__ == '__main__':
    unittest.main()
