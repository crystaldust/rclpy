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

import unittest

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from test_msgs.msg import BasicTypes
from rclpy_components.component_manager import ComponentManager
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from threading import Thread


class TestComponentManager(unittest.TestCase):
    single_threaded_executor: SingleThreadedExecutor = None
    multi_threaded_executor = None
    component_manager = None
    helper_node = None
    context = None
    composition_clients = None

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.single_threaded_executor = SingleThreadedExecutor(context=cls.context)
        cls.multi_threaded_executor = MultiThreadedExecutor(context=cls.context)
        cls.component_manager = ComponentManager(cls.single_threaded_executor,
                                                 'TestComponentManager',
                                                 context=cls.context)

        cls.single_threaded_executor.add_node(cls.component_manager)

        cls.helper_node = rclpy.create_node('helper', context=cls.context)
        cls.action_service_map = {
            'load_node': LoadNode,
            'unload_node': UnloadNode,
            'list_nodes': ListNodes,
        }

        cls.composition_clients = {}
        for action, service in cls.action_service_map.items():
            cls.composition_clients[action] = cls.helper_node.create_client(
                service,
                '{}/_container/{}'.format(cls.component_manager.get_name(), action)
            )

    @classmethod
    def tearDownClass(cls):
        cls.single_threaded_executor.remove_node(cls.component_manager)
        cls.component_manager.destroy_node()

        rclpy.shutdown(context=cls.context)

    @classmethod
    def _get_res(cls, req, client_key):
        client = cls.composition_clients[client_key]
        future = client.call_async(req)
        print('_get_res, future:', future)
        return future.result()

    @classmethod
    def load_node(cls, req: LoadNode.Request, multi_threaded_container=False) -> LoadNode.Response:
        client_key = 'load_node_mt' if multi_threaded_container else 'load_node'
        return cls._get_res(req, client_key)

    @classmethod
    def unload_node(cls, req: UnloadNode.Request, multi_threaded_container=False) -> UnloadNode.Response:
        client_key = 'unload_node_mt' if multi_threaded_container else 'unload_node'
        return cls._get_res(req, client_key)

    @classmethod
    def list_nodes(cls, req: ListNodes.Request, multi_threaded_container=False) -> ListNodes.Response:
        client_key = 'list_nodes_mt' if multi_threaded_container else 'list_nodes'
        return cls._get_res(req, client_key)

    def test_fuck(self):
        TestComponentManager.single_threaded_executor.spin_once(3)
        print('after spin once')

        req = LoadNode.Request()
        req.package_name = 'py_composition'
        req.plugin_name = 'py_composition::Listener'


        rclpy.init()
        node = rclpy.create_node("fuck")
        cli = node.create_client(LoadNode, "/TestComponentManager/_container/load_node")
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(TestComponentManager.helper_node, future)
        print('future:', future)

    def test_load_node(self):
        return
        # t = Thread(target=TestComponentManager.single_threaded_executor.spin, daemon=True)
        t = Thread(target=lambda cls: cls.single_threaded_executor.spin(), args=(TestComponentManager,), daemon=True)
        t.start()


        req = LoadNode.Request()
        req.package_name = 'py_composition'
        req.plugin_name = 'py_composition::Listener'
        print(t.is_alive())

        from time import sleep
        # sleep(5)
        rclpy.init()
        node = rclpy.create_node("fuck")
        # node = rclpy.create_node("fuck", context=TestComponentManager.context)
        cli = node.create_client(LoadNode, '/TestComponentManager/_container/load_node')
        future = cli.call_async(req)
        print('dddd')
        tx = Thread(target=rclpy.spin_until_future_complete(node, future), args=(node, future), daemon=True)
        print('tx.start')
        tx.start()
        print(future.result())

        tx.join()
        print('after tx.join()')
        # rclpy.spin_until_future_complete(node, future)
        # print('future:', future)
        # print('result:', future.result())


        # cli = TestComponentManager.composition_clients['load_node']
        # future = cli.call_async(req)
        # print('future:', future)
        # print(future.result())
        TestComponentManager.single_threaded_executor.shutdown()
        t.join()


if __name__ == '__main__':
    unittest.main()
