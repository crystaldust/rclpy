from rclpy.node import Node
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode


class ComponentManager(Node):

    def __init__(self, executor):
        super().__init__("component_manager")  # TODO Handle the py args equivalent to rclcpp 'NodeOptions'
        self.executor = executor
        # Implement the 3 services described in
        # http://design.ros2.org/articles/roslaunch.html#command-line-arguments
        self.list_node_srv_ = self.create_service(ListNodes, "~/_container/list_node", self.on_list_node)
        self.load_node_srv_ = self.create_service(LoadNode, "~/_container/load_node", self.on_load_node)
        self.unload_node_srv_ = self.create_service(UnloadNode, "~/_container/unload_node", self.on_unload_node)

    def on_list_node(self, req, res):
        # TODO The ros2cli component verb might be referred to
        pass

    def on_load_node(self, req, res):
        pass

    def on_unload_node(self, req, res):
        pass
