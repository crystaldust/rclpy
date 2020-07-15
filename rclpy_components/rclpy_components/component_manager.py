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

from rclpy.node import Node
from rclpy.executors import Executor
from rclpy.logging import get_logger
from rclpy.exceptions import InvalidNodeNameException, InvalidNamespaceException
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
try:
    from importlib.metadata import entry_points
except ImportError:
    from importlib_metadata import entry_points

RCLPY_COMPONENTS = 'rclpy_components'
logger = get_logger('ComponentManager')


class ComponentManager(Node):

    def __init__(self, executor: Executor, name="py_component_manager", **kwargs):
        # TODO Handle the py args equivalent to rclcpp 'NodeOptions'
        super().__init__(name, **kwargs)
        self.executor = executor
        # Implement the 3 services described in
        # http://design.ros2.org/articles/roslaunch.html#command-line-arguments
        self.list_node_srv_ = self.create_service(ListNodes, "~/_container/list_nodes", self.on_list_node)
        self.load_node_srv_ = self.create_service(LoadNode, "~/_container/load_node", self.on_load_node)
        self.unload_node_srv_ = self.create_service(UnloadNode, "~/_container/unload_node", self.on_unload_node)

        self.components = {}  # key: unique_id, value: full node name and component instance
        self.unique_id_index = 0

    def _parse_remap_rules(self, remap_rules):
        """
        Parse name and namespace from remap rules

        This is a temporary method, when node.get_fully_qualified_name is ready, it should be removed.
        """
        import re
        name, ns = None, None
        name_pattern, ns_pattern = re.compile('__node:=(.*)'), re.compile('__ns:=(.*)')
        for rule in remap_rules:
            name_result = name_pattern.search(rule)
            if name_result:
                name = name_result.group(1)
            ns_result = ns_pattern.search(rule)
            if ns_result:
                # There might be leading slash
                # Leave the ugly here, it will be removed.
                ns = ns_result.group(1).lstrip('/')

            if name and ns:
                break
        return name, ns

    def gen_unique_id(self):
        self.unique_id_index += 1
        return self.unique_id_index

    def on_list_node(self, req: ListNodes.Request, res: ListNodes.Response):
        res.unique_ids = [int(key) for key in self.components.keys()]
        res.full_node_names = [v[0] for v in self.components.values()]

        return res

    def on_load_node(self, req: LoadNode.Request, res: LoadNode.Response):
        component_entry_points = entry_points().get(RCLPY_COMPONENTS, None)
        if not component_entry_points:
            logger.error('No rclpy components registered')
            res.success = False
            return res

        component_entry_point = None
        for ep in component_entry_points:
            if ep.name == req.plugin_name:
                component_entry_point = ep
                break

        if not component_entry_point:
            logger.error('No rclpy component found by %s' % req.plugin_name)
            res.success = False
            return res

        component_class = component_entry_point.load()

        params_dict = {'use_global_arguments': False, 'context': self.context}
        if req.parameters:
            params_dict['parameter_overrides'] = req.parameters

        if req.node_namespace:
            params_dict['namespace'] = req.node_namespace

        if req.remap_rules:
            params_dict['cli_args'] = ['--ros-args']
            for rule in req.remap_rules:
                params_dict['cli_args'].extend(['-r', rule])
        try:
            # TODO Assign the full_node_name with node.get_fully_qualified_name, which will handle priority of ns, name
            remapped_name, remapped_ns = self._parse_remap_rules(req.remap_rules)
            node_name = remapped_name if remapped_name else req.node_name
            if not node_name:
                node_name = str.lower(str.split(component_entry_point.value, ':')[1])
            res.full_node_name = '/{}'.format(node_name)
            namespace = remapped_ns if remapped_ns else req.node_namespace
            if namespace:
                res.full_node_name = '/{}{}'.format(namespace, res.full_node_name)

            logger.info('Instantiating {} with {}, {}'.format(component_entry_point.value, node_name, params_dict))
            component = component_class(node_name, **params_dict)
            res.unique_id = self.gen_unique_id()
            res.success = True
            self.components[str(res.unique_id)] = (res.full_node_name, component)
            self.executor.add_node(component)
            return res
        except (InvalidNodeNameException, InvalidNamespaceException, TypeError) as e:
            error_message = str(e)
            logger.error('Failed to load node: %s' % error_message)
            res.success = False
            res.error_message = error_message
            return res
        except Exception as e:
            logger.error('Failed to load node: %s' % str(e))
            res.success = False
            res.error_message = 'Unexpected error, please check the component container log.'
            return res

    def on_unload_node(self, req: UnloadNode.Request, res: UnloadNode.Response):
        uid = str(req.unique_id)
        if uid not in self.components:
            res._error_message = 'No node found with unique_id: %s' % uid
            res.success = False
            return res

        _, component_instance = self.components.pop(uid)
        self.executor.remove_node(component_instance)
        res.success = True
        return res
