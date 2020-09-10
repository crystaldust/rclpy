import rclpy
from rclpy_components.component_manager import ComponentManager
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

single_threaded_executor = None
multi_threaded_executor = None
component_manager = None
component_manager_mt = None

from threading import Thread

def main():
    rclpy.init()
    single_threaded_executor = SingleThreadedExecutor()
    multi_threaded_executor = MultiThreadedExecutor()
    component_manager = ComponentManager(single_threaded_executor, "TestComponentManager")
    component_manager_mt = ComponentManager(multi_threaded_executor, "TestComponentManagerMT")

    Thread(target=single_threaded_executor.spin, daemon=True).start()
    Thread(target=multi_threaded_executor.spin, daemon=True).start()







if __name__ == '__main__':
    main()