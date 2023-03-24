from setuptools import setup

package_name = 'goal_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mehce',
    maintainer_email='mehceu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pub = goal_publisher.goal_publisher:main'
        ],
    },
)
