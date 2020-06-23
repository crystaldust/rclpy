from rclpy.node import Node
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from ament_index_python import get_resource
from importlib import import_module
from rclpy.executors import Executor

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
        try:
            # content example: composition::Talker;my_pkg.my_components:Talker
            content, base_path = get_resource(RCLPY_COMPONENTS, req.package_name)
            entrypoint_path = _get_entrypoint_from_component(req.plugin_name, content)

            module_path, class_name = [part.strip() for part in str.split(entrypoint_path, ':')]
            component_module = import_module(module_path)
            component_cls = getattr(component_module, class_name)  # TODO Is there a better way to get the class?
            component_instance = component_cls(class_name)
            res.unique_id = self.gen_unique_id()
            res.full_node_name = '/' + str.lower(class_name)

            self.components[str(res.unique_id)] = (res.full_node_name, component_instance)
            self.executor.add_node(component_instance)
            res.success = True
            return res
        except Exception as e:
            self.get_logger().error('Failed to load node %s' % e)
            res.success = False
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
