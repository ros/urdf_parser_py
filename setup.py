from setuptools import setup

package_name = 'urdfdom_py'

setup(
    name=package_name,
    version='0.3.3',
    package_dir={'': 'src'},
    packages=['urdf_parser_py', 'urdf_parser_py.xml_reflection'],
    data_files=[
        ('scripts/display_urdf',
        ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Victor Mayoral Vilches',
    author_email='vmayoral@acutronicrobotics.com',
    maintainer='Victor Mayoral Vilches',
    maintainer_email='vmayoral@acutronicrobotics.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python implementation of the URDF parser.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
