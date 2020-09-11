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
from multiprocessing import Process

import rclpy
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from rclpy.client import Client
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.node import Node
from rclpy_components.component_manager import ComponentManager

RCLPY_COMPOSITION = 'py_composition'
RCLPY_COMPOSITION_TALKER = 'py_composition::Talker'
RCLPY_COMPOSITION_LISTENER = 'py_composition::Listener'

class TestComponentManager(unittest.TestCase):
    single_threaded_executor: SingleThreadedExecutor = None
    multi_threaded_executor: MultiThreadedExecutor = None
    component_manager: ComponentManager = None
    helper_node: Node = None
    context: Context = None
    composition_clients = None

    container_process: Process = None

    container_name = 'TestComponentManager'
    load_node_svc_name = f'{container_name}/_container/load_node'
    unload_node_svc_name = f'{container_name}/_container/unload_node'
    list_node_svc_name = f'{container_name}/_container/list_nodes'
    load_cli: Client = None
    unload_cli: Client = None
    list_cli: Client = None

    @classmethod
    def setUpClass(cls):
        def run_component_manager(container_name):
            rclpy.init()
            single_threaded_executor = SingleThreadedExecutor()
            component_manager = ComponentManager(single_threaded_executor, container_name)
            single_threaded_executor.add_node(component_manager)
            try:
                single_threaded_executor.spin()
            except Exception as e:
                print(e)
                pass

            component_manager.destroy_node()
            rclpy.shutdown()

        # Start the test component manager in the background
        cls.container_process = Process(target=run_component_manager,
                                        args=(cls.container_name,))
        cls.container_process.start()

        # Setup the helper_node, which will help create client and test the services
        rclpy.init()
        cls.helper_node = rclpy.create_node('helper_node')
        cls.load_cli = cls.helper_node.create_client(LoadNode, cls.load_node_svc_name)
        cls.unload_cli = cls.helper_node.create_client(UnloadNode, cls.unload_node_svc_name)
        cls.list_cli = cls.helper_node.create_client(ListNodes, cls.list_node_svc_name)

    @classmethod
    def tearDownClass(cls):
        cls.container_process.terminate()
        cls.load_cli.destroy()
        cls.unload_cli.destroy()
        cls.list_cli.destroy()
        cls.helper_node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def load_node(cls, package_name, plugin_name, node_name="", node_namespace=""):
        if not cls.load_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'No load service found in /{cls.container_name}')

        load_req = LoadNode.Request()
        load_req.package_name = package_name
        load_req.plugin_name = plugin_name

        if node_name != "":
            load_req.node_name = node_name
        if node_namespace != "":
            load_req.node_namespace = node_namespace

        future = cls.load_cli.call_async(load_req)
        rclpy.spin_until_future_complete(cls.helper_node, future)
        return future.result()

    @classmethod
    def unload_node(cls, unique_id):
        if not cls.unload_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'No unload service found in /{cls.container_name}')

        unload_req = UnloadNode.Request()
        unload_req.unique_id = unique_id

        future = cls.unload_cli.call_async(unload_req)
        rclpy.spin_until_future_complete(cls.helper_node, future)
        return future.result()

    @classmethod
    def list_nodes(cls):
        if not cls.list_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'No list service found in {cls.container_name}')
        list_req = ListNodes.Request()
        future = cls.list_cli.call_async(list_req)
        rclpy.spin_until_future_complete(TestComponentManager.helper_node, future)
        return future.result()

    def test_load_node(self):
        load_res = TestComponentManager.load_node(RCLPY_COMPOSITION, RCLPY_COMPOSITION_LISTENER)
        assert load_res.success is True
        assert load_res.error_message == ""
        assert load_res.unique_id == 1
        assert load_res.full_node_name == '/listener'

        node_name = "renamed_listener"
        node_ns = 'testing_ns'
        remap_load_res = load_res = TestComponentManager.load_node(
            RCLPY_COMPOSITION, RCLPY_COMPOSITION_LISTENER, node_name=node_name,
            node_namespace=node_ns)
        assert remap_load_res.success is True
        assert remap_load_res.error_message == ""
        assert remap_load_res.unique_id == 2
        assert remap_load_res.full_node_name == f'/{node_ns}/{node_name}'

        list_res: ListNodes.Response = TestComponentManager.list_nodes()
        assert len(list_res.unique_ids) == len(list_res.full_node_names) == 2

    def test_unload_node(self):
        if not TestComponentManager.unload_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'no unload service found in {TestComponentManager.container_name}')

        unload_res: UnloadNode.Response = TestComponentManager.unload_node(1)
        assert unload_res.success is True
        assert unload_res.error_message == ""

        # Should be only 1 node left
        list_res: ListNodes.Response = TestComponentManager.list_nodes()
        assert len(list_res.full_node_names) == len(list_res.unique_ids) == 1

        # The index definitely won't exist
        unload_res: UnloadNode.Response = TestComponentManager.unload_node(1000)
        assert unload_res.error_message != ""
        assert unload_res.success is False
        list_res: ListNodes.Response = TestComponentManager.list_nodes()
        assert len(list_res.full_node_names) == len(list_res.unique_ids) == 1

        # Unload the last node
        unload_req = UnloadNode.Request()
        unload_req.unique_id = 2
        future = TestComponentManager.unload_cli.call_async(unload_req)
        rclpy.spin_until_future_complete(TestComponentManager.helper_node, future)
        unload_res: UnloadNode.Response = future.result()
        assert unload_res.success is True
        assert unload_res.error_message == ""

        # Won't be any node in the container
        list_res: ListNodes.Response = TestComponentManager.list_nodes()
        assert len(list_res.full_node_names) == len(list_res.unique_ids) == 0

    def test_list_nodes(self):
        print('list_nodes tested within test_load_node and test_unload_node')


if __name__ == '__main__':
    unittest.main()
