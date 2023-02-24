from setuptools import setup

package_name = 'image_lidar_roi_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_sandbox',
    maintainer_email='ros_sandbox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_lidar_roi_publisher_node = image_lidar_roi_publisher.image_lidar_roi_publisher_node:main'
        ],
    },
)
