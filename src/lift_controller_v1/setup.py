from setuptools import setup

package_name = 'lift_controller_v1'

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
        	"test_node = lift_controller_v1.test_node:main",
        	"test_pub = lift_controller_v1.test_pub:main",
        	"test_sub = lift_controller_v1.test_sub:main",
        	"lift_pub_test = lift_controller_v1.lift_pub_test1:main",
        	"lift_sub_test = lift_controller_v1.lift_sub_test1:main"
        ],
    },
)
