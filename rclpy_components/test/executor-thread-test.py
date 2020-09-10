from multiprocessing import Process
import rclpy
from rclpy_components.component_manager import ComponentManager
from rclpy.executors import SingleThreadedExecutor
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes

import multiprocessing

def foo():

    global executor

    print('before executor spin')
    # executor.spin_once(1)
    executor.spin()
    print('after executor spin')

rclpy.init()
executor = SingleThreadedExecutor()
component_manager = ComponentManager(executor, "my_manager")
print(component_manager, 'inited')

t = Process(target=foo, daemon=True)
t.start()
t.join()

print('continue')

node = rclpy.create_node("helper")
cli = node.create_client(LoadNode, "/my_manager/_container/Load_node")

req = LoadNode.Request()
req.package_name = "py_composition"
req.plugin_name = "py_composition::Listener"

# future = cli.call_async(req)
# print(future.result())
# print('spin for future')
# rclpy.spin_until_future_complete(node, future)
# print('after future')
# print(future, future.result())

# executor.remove_node(component_manager)
# component_manager.destroy_node()

# executor.shutdown()
# rclpy.shutdown()

t.join()
print('finished')