from setuptools import find_packages, setup

package_name = 'pascal_full'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_launch.py']),
        ('share/' + package_name + '/launch', ['launch/apple_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pascal',
    maintainer_email='pascal@todo.todo',
    description='Full launch wrapper for arm control, depth proc, odom Z-estimate, and tree templating',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voxelize = pascal_full.voxelize:main',
            'apple_pred = pascal_full.apple_pred:main',
            'visualize_apple_pred = pascal_full.visualize_apple_pred:main',
            'trajectory_cacher = pascal_full.trajectory_cacher:main',
            'trajectory_execute = pascal_full.trajectory_execute:main',
            'all_points_cacher = pascal_full.all_points_cacher:main',
            'all_points_execute = pascal_full.all_points_execute:main',
            'hybrid_exec = pascal_full.hybrid_exec:main',
            'traj_csv_logger = pascal_full.traj_csv_logger:main'
        ],
    },
)