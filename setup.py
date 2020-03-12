from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['urdf_parser_py', 'urdf_parser_py.xml_reflection'],
    package_dir={'': 'src'}
)

setup(**d)
