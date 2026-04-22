from setuptools import setup, find_packages

package_name = 'ackermann_core'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages() trova in automatico anche "hardware" e "tools"
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pietro',
    maintainer_email='gelmini.pietro@gmail.com',
    description='Ackermann Autonomous Robotaxi Core',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_node = ackermann_core.base_node:main'
        ],
    },
)