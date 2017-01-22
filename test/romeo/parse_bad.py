#!/usr/bin/env python

import sys
import argparse

from urdf_parser_py.urdf import Robot

robot = Robot.from_xml_file('./romeo_bad.urdf')

print(robot)
