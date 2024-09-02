from setuptools import find_packages, setup

package_name = 'dynamixel_keyboard_controller'

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
    maintainer='sli',
    maintainer_email='connieyunji@sookmyung.ac.kr',
    description='Control dynamixel whith keyboard input',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
        'dynamixel_keyboard_controller = dynamixel_keyboard_controller.controller:main'
        ],
    },
)
