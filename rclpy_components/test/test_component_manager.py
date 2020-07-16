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

import pytest

import rclpy
from rclpy_components.component_manager import ComponentManager
from rclpy.executors import Executor, SingleThreadedExecutor, MultiThreadedExecutor
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from threading import Thread


class TestComponentManager:
    context = None
    executor: Executor = None

    helper_node = None
    helper_executor: SingleThreadedExecutor = None

    container: ComponentManager = None
    composition_clients = None
    container_thread = None

    @classmethod
    def init_executor(cls):
        cls.executor = SingleThreadedExecutor(context=cls.context)

    @classmethod
    def setup_class(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.init_executor()
        cls.container = ComponentManager(cls.executor,
                                         'TestComponentManager',
                                         context=cls.context)

        cls.executor.add_node(cls.container)

        cls.helper_node = rclpy.create_node('helper', context=cls.context)
        cls.helper_executor = SingleThreadedExecutor(context=cls.context)
        cls.action_service_map = {
            'load_node': LoadNode,
            'unload_node': UnloadNode,
            'list_nodes': ListNodes,
        }

        cls.composition_clients = {}
        for action, service in cls.action_service_map.items():
            cls.composition_clients[action] = cls.helper_node.create_client(
                service,
                '{}/_container/{}'.format(cls.container.get_name(), action)
            )

        cls.container_thread = Thread(target=cls.executor.spin, daemon=True)
        cls.container_thread.start()

    @classmethod
    def teardown_class(cls):
        cls.executor.remove_node(cls.container)
        cls.container.destroy_node()
        # Let the system release all resources, since the executor thread is still blocking
        # cls.container_thread.join()
        # rclpy.shutdown(context=cls.context)

    @classmethod
    def _get_res(cls, req, client_key):
        client = cls.composition_clients[client_key]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(cls.helper_node, future, cls.helper_executor)
        return future.result()

    @classmethod
    def load_node(cls, req: LoadNode.Request) -> LoadNode.Response:
        return cls._get_res(req, 'load_node')

    @classmethod
    def unload_node(cls, req: UnloadNode.Request) -> UnloadNode.Response:
        return cls._get_res(req, 'unload_node')

    @classmethod
    def list_nodes(cls, req: ListNodes.Request) -> ListNodes.Response:
        return cls._get_res(req, 'list_nodes')

    def test_load_node(self):
        node_names = []
        req = LoadNode.Request()
        req.package_name = 'py_composition'
        req.plugin_name = 'py_composition::Listener'

        res = TestComponentManager.load_node(req)
        assert res.success is True
        assert not res.error_message
        assert res.full_node_name == '/listener'
        node_names.append(res.full_node_name)

        req.node_namespace = 'test_ns'
        res = TestComponentManager.load_node(req)
        assert res.success is True
        assert not res.error_message
        assert res.full_node_name == '/test_ns/listener'
        node_names.append(res.full_node_name)

        res = TestComponentManager.list_nodes(ListNodes.Request())
        for node_name in node_names:
            assert node_name in res.full_node_names

    def test_load_node_remap_rules(self):
        req = LoadNode.Request()
        req.package_name = 'py_composition'
        req.plugin_name = 'py_composition::Listener'
        req.remap_rules = ['__ns:=another_ns', '__node:=another_node'] # Invalid ns remapping
        res = TestComponentManager.load_node(req)
        assert res.success is False

        req.remap_rules = ['__ns:=/another_ns', '__node:=another_node']
        res = TestComponentManager.load_node(req)
        assert res.success is True
        assert res.full_node_name == '/another_ns/another_node'

    def test_unload_node(self):
        temp_node_name = 'deleteme'
        req = LoadNode.Request()
        req.package_name = 'py_composition'
        req.plugin_name = 'py_composition::Listener'
        req.node_name = temp_node_name
        res = TestComponentManager.load_node(req)
        assert res.success is True

        unload_req = UnloadNode.Request()
        unload_req.unique_id = res.unique_id
        unload_res = TestComponentManager.unload_node(unload_req)
        assert unload_res.success is True

        list_res = TestComponentManager.list_nodes(ListNodes.Request())
        assert temp_node_name not in list_res.full_node_names
        assert res.unique_id not in list_res.unique_ids


class TestComponentManagerMT(TestComponentManager):
    @classmethod
    def init_executor(cls):
        cls.executor = MultiThreadedExecutor(context=cls.context)
