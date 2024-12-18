# my_python_node/setup.py
from setuptools import setup

package_name = 'my_python_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple Python node for ROS 2',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = my_python_node.simple_node:main',
            'TSM_node = my_python_node.TSM_node:main',
            'BRAM_node = my_python_node.Read_BRAM:main',
        ],
    },
)
