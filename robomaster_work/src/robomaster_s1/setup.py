from setuptools import find_packages, setup

package_name = 'robomaster_s1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # py_modules=[
    #     'ros2_ps5controller',
    #     'ps5controller'
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Van Hooser',
    maintainer_email='jvanhooser2021@my.fit.edu',
    description='Package developed for working with Robomaster s1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_ps5controller = robomaster_s1.ros2_ps5controller:main',
            'ros2_robot = robomaster_s1.ros2_robot:main'
        ],
    },
)
