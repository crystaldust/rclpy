import os

import ament_index_python as ament_index

from .component_manager import ComponentManager


RCLPY_COMPONENTS = 'rclpy_components'
AMENT_INDEX_RESOURCE_PATH = 'share/ament_index/resource_index/rclpy_components/'


def register_component_in_setuppy(package_name, rclpy_components):
    """
    Register component in a setup.py file


    :param rclpy_components: A list of setuptool entrypoint strings,
        like ['composition::Talker=rclpy_components_examples:Talker']
    :return: None
    """

    # Create ament resources
    # Order: The current folder(where setup.py is located)
    # If current folder is not in the search paths, use the system ROS folder(/opt/ros/<distro>/)
    resource_path = ""
    for p in ament_index.get_search_paths():
        if package_name in p:
            resource_path = p

    # Parse the entry points
    resources = {}  # Key: resource_name, value: list of resource strings
    for rclpy_component_str in rclpy_components:
        parts = str.split(rclpy_component_str, '=')
        resource_str = parts[0].strip()  # In the pattern 'RES_NAME::RES_CLASS'
        resource_name = str.split(resource_str, '::')[0].strip()
        entry_point_path = parts[1].strip()

        content = '{};{}'.format(resource_str, entry_point_path)
        if resource_name in resources:
            resources[resource_name].append(content)
        else:
            resources[resource_name] = [content]

    res_folder = os.path.join(resource_path, AMENT_INDEX_RESOURCE_PATH)
    if not os.path.exists(res_folder):
        os.mkdir(res_folder)

    for resource_name, contents in resources.items():
        res_path = os.path.join(res_folder, resource_name)
        with open(res_path, 'w') as f:
            f.writelines(contents)
