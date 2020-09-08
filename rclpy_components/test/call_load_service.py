from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from test_msgs.msg import BasicTypes
from rclpy_components.component_manager import ComponentManager
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from threading import Thread

# context = rclpy.context.Context()
# rclpy.init(context=context)
# node = rclpy.create_node('helper', context=context)
# cli = node.create_client(LoadNode, '/TestComponentManager/_container/load_node')
#
# req = LoadNode.Request()
# req.package_name = "py_composition"
# req.plugin_name = "py_composition::Listener"
#
# future = cli.call_async(req)
# rclpy.spin_until_future_complete(node, future=future)
# print(future.result())

rclpy.init()
node = rclpy.create_node('helper')
cli = node.create_client(LoadNode, '/TestComponentManager/_container/load_node')

req = LoadNode.Request()
req.package_name = "py_composition"
req.plugin_name = "py_composition::Listener"

future = cli.call_async(req)
rclpy.spin_until_future_complete(node, future=future)
print(future.result())

node.destroy_node()
rclpy.shutdown()