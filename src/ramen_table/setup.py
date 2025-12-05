from setuptools import find_packages, setup

package_name = 'ramen_table'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Table nodes for ordering and feedback',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_node = ramen_table.order_node:main',
            'customer_node = ramen_table.customer_node:main',
            'counter_node =  ramen_table.counter_node:main',
        ],
    },
)