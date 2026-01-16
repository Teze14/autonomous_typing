from setuptools import find_packages, setup

package_name = 'autonomous_typing'

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
    maintainer='quantum_main',
    maintainer_email='quantumrobotics.itesm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = autonomous_typing.vision_node:main',
            'controller_node = autonomous_typing.controller_node:main',
        ],
    },
)
