from setuptools import find_packages, setup

package_name = 'winterhack_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'launch/pc.launch.py',
            'launch/pi.launch.py'
        ]),
        ('share/' + package_name, [
            'config/nav2_params.yaml'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atomic',
    maintainer_email='313443118@qq.com',
    description='The bringup package of winterhack project',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
