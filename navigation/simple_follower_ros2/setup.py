from setuptools import setup

package_name = 'simple_follower_ros2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/line_follower.launch.py']))
data_files.append(('share/' + package_name, ['launch/laser_follower.launch.py']))
data_files.append(('share/' + package_name, ['launch/visual_follower.launch.py']))
data_files.append(('share/' + package_name, ['launch/visual_follower.launch.py']))
data_files.append(('share/' + package_name, ['launch/adjust_hsv.launch.py']))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='powrbv@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follow = simple_follower_ros2.line_follow:main',
            'laserfollower = simple_follower_ros2.laserfollower:main',
            'visualtracker = simple_follower_ros2.visualTracker:main',
            'visualfollow = simple_follower_ros2.visualFollower:main',
            'adjust_hsv = simple_follower_ros2.adjust_hsv:main',
            'lasertracker ='
            ' simple_follower_ros2.laserTracker:main',
    
        ],
    },
)
