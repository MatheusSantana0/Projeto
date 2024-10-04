import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'go2_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matheus',
    maintainer_email='matheusjere10@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "broadcast_tf = go2_main.broadcast_tf:main",
            "valores_juntas = go2_main.valores_juntas:main",
            "pointcloud_bridge = go2_main.pointcloud_bridge:main" 
        ],
    },
)
