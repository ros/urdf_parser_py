from glob import glob
from setuptools import setup

package_name = 'urdfdom_py'

setup(
    name=package_name,
    version='1.2.1',
    package_dir={'': 'src'},
    packages=['urdf_parser_py', 'urdf_parser_py.xml_reflection'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    description='Python implementation of the URDF parser.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display_urdf = urdf_parser_py.display_urdf:main',
        ],
    },
)
