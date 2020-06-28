from rclpy.node import Node
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from rclpy.executors import Executor
from importlib_metadata import entry_points

RCLPY_COMPONENTS = 'rclpy_components'


def _get_entrypoint_from_component(package_name, component_content):
    """
    component_content is the file content, a single string with '\n'
    """
    for line in str.splitlines(component_content):
        parts = str.split(line, ';')
        if parts[0].strip() == package_name:
            return parts[1].strip()
    raise Exception("%s not found in the components" % package_name)


class ComponentManager(Node):

    def __init__(self, executor: Executor, name="py_component_manager", *args, **kwargs):
        # TODO Handle the py args equivalent to rclcpp 'NodeOptions'
        super().__init__(name, *args, **kwargs)
        self.executor = executor
        # Implement the 3 services described in
        # http://design.ros2.org/articles/roslaunch.html#command-line-arguments
        self.list_node_srv_ = self.create_service(ListNodes, "~/_container/list_nodes", self.on_list_node)
        self.load_node_srv_ = self.create_service(LoadNode, "~/_container/load_node", self.on_load_node)
        self.unload_node_srv_ = self.create_service(UnloadNode, "~/_container/unload_node", self.on_unload_node)

        self.components = {}  # key: unique_id, value: full node name and component instance
        self.unique_id_index = 0

        self.executor.spin()

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
            self.get_logger().error('No rclpy components registered')
            res.success = False
            return res

        component_entry_point = None
        for ep in component_entry_points:
            if ep.name == req.plugin_name:
                component_entry_point = ep
                break

        if not component_entry_point:
            self.get_logger().error('No rclpy component found by %s' % req.plugin_name)
            res.success = False
            return res

        component_class = component_entry_point.load()
        node_name = req.node_name if req.node_name else str.split(component_entry_point.value, ':')[1]
        component = component_class(node_name)

        # TODO Handle the node_name, node_namespace, and remapping rules.

        res.unique_id = self.gen_unique_id()
        res.full_node_name = '/' + str.lower(node_name)
        self.components[str(res.unique_id)] = (res.full_node_name, component)
        self.executor.add_node(component)
        res.success = True
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
