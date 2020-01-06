from glob import glob
from setuptools import setup

package_name = 'urdfdom_py'

setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=['urdf_parser_py', 'urdf_parser_py.xml_reflection'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', glob('scripts/display_urdf')),
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
    description='Python implementation of the URDF parser.',
    license='BSD',
    tests_require=['pytest'],
)
