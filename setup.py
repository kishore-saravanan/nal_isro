from setuptools import find_packages, setup

package_name = 'nal_isro'

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
    maintainer='Kishore',
    maintainer_email='kishoresaravanan2002@gmail.com',
    description='Package with master and slave nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slave1_node = nal_isro.slave1_node:main',
            'slave2_node = nal_isro.slave2_node:main',
            'slave3_node = nal_isro.slave3_node:main',
            'slave5_node = nal_isro.slave5_node:main',
            'master_node = nal_isro.master_node:main',
            'record_cmd_vel = nal_isro.cmd_vel_recorder:main',
            'publish_cmd_vel_from_csv = nal_isro.publish_cmd_vel_from_csv:main',
            'depth_processor = nal_isro.depth_processor:main',
        ],
    },
)
