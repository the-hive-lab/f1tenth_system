from setuptools import setup

package_name = 'throttle_interpolator'

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
    maintainer='reyespinr',
    maintainer_email='reyespinr@vcu.edu',
    description='ROS2 Node to interpolate and smooth steering and throttle to avoid high peaks.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_interpolator = throttle_interpolator.throttle_interpolator:main'
        ],
    },
)