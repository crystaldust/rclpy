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
from rclpy.node import Node
from rclpy.context import Context
from rclpy_components.component_manager import ComponentManager
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from threading import Thread
from multiprocessing import Process

class TestComponentManager(unittest.TestCase):
    single_threaded_executor: SingleThreadedExecutor = None
    multi_threaded_executor: MultiThreadedExecutor = None
    component_manager: ComponentManager = None
    helper_node: Node = None
    context: Context = None
    composition_clients = None

    background_process: Process = None
    foreground_process: Process = None

    @classmethod
    def _start_component_manager(cls):
        rclpy.init()
        single_threaded_executor = SingleThreadedExecutor()
        component_manager = ComponentManager(single_threaded_executor, 'TestComponentManager')
        single_threaded_executor.add_node(component_manager)
        try:
            single_threaded_executor.spin()
        except Exception as e:
            print(e)
            pass

        component_manager.destroy_node()
        rclpy.shutdown()

    # @classmethod
    # def _call_load_service(cls):
    #     rclpy.init()
    #     node = rclpy.create_node('load_component_node')
    #     cli = node.create_client(LoadNode, "/TestComponentManager/_container/load_node")
    #     req = LoadNode.Request()
    #     req.package_name


    @classmethod
    def setUpClass(cls):
        print('setupClass')
        def run_component_manager():
            rclpy.init()
            single_threaded_executor = SingleThreadedExecutor()
            component_manager = ComponentManager(single_threaded_executor, 'TestComponentManager')
            single_threaded_executor.add_node(component_manager)
            try:
                single_threaded_executor.spin()
            except Exception as e:
                print(e)
                pass

            component_manager.destroy_node()
            rclpy.shutdown()
        cls.background_process = Process(target=run_component_manager)
        cls.background_process.start()
        # cls.action_service_map = {
        #     'load_node': LoadNode,
        #     'unload_node': UnloadNode,
        #     'list_nodes': ListNodes,
        # }
        #
        # cls.composition_clients = {}
        # for action, service in cls.action_service_map.items():
        #     cls.composition_clients[action] = cls.helper_node.create_client(
        #         service,
        #         '{}/_container/{}'.format(cls.component_manager.get_name(), action)
        #     )
        #
        # cls.background_process = Process(target=cls.single_threaded_executor.spin)


    @classmethod
    def tearDownClass(cls):
        print('teardown class')
        cls.background_process.terminate()
        cls.background_process.kill()
        # cls.single_threaded_executor.remove_node(cls.component_manager)
        # cls.component_manager.destroy_node()
        #
        # rclpy.shutdown(context=cls.context)
        # cls.background_process.terminate()

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
        print('test_fuck')

        def foo():
            req = LoadNode.Request()
            req.package_name = 'py_composition'
            req.plugin_name = 'py_composition::Listener'
            c = rclpy.context.Context()
            rclpy.init()
            node = rclpy.create_node('xxxhelper')
            print('rclpy.create_node with cls.context')
            cli = node.create_client(LoadNode, "/TestComponentManager/_container/load_node")

            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('no load service found in /PyComponentManager')

            future = cli.call_async(req)

            # while rclpy.ok():
                # rclpy.spin_once()
            print('spin until')
            rclpy.spin_until_future_complete(node, future)
            print(future.result())
            res: LoadNode.Response = future.result()
            print(f'res.full_node_name: {res.full_node_name}')
            assert res.full_node_name == '/listener'

            node.destroy_node()
            rclpy.shutdown()

        foo()
        # p = Process(target=foo)
        # p.start()
        # p.join()
        # print('p finished')



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
