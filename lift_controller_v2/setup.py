from setuptools import setup

package_name = 'lift_controller_v2'

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
    maintainer='roy',
    maintainer_email='roy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"lift_control_node = lift_controller_v2.lift_control:main",
        	"robot_control_node = lift_controller_v2.robot_control:main",
        	"button_control_node = lift_controller_v2.button_control:main",
        	"qr_decoder = lift_controller_v2.qr_decoder:main"
        ],
    },
)
