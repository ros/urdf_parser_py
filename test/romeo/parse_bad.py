#!/usr/bin/env python

import sys
import argparse

from urdf_parser_py import urdf

messages = []
def add_message(message):
    messages.append(message)
urdf.xmlr.core.on_error = add_message

xml_string = """
<robot name="test">
  <transmission name_BAD="bad_trans"/>
</robot>
"""
messages = []
try:
    urdf.Robot.from_xml_string(xml_string)
except urdf.xmlr.core.ParseError, e:
    print e
    print e.path
    
messages = []
xml_string = """
<link name="b">
    <unknown_element/>
</link>
"""
urdf.Link.from_xml_string(xml_string)
print messages

xml_string = '''<?xml version="1.0"?>
<link name="b" unknown_tag="something"/>'''
messages = []
urdf.Link.from_xml_string(xml_string)
print messages

xml_string = """
<robot name="test">
    <link name="good"/>
    <link na_me="bad">
        <unknown_element/>
    </link>
</robot>
"""
try:
    urdf.Robot.from_xml_string(xml_string)
except urdf.xmlr.core.ParseError, e:
    print e
    print e.path

xml_string = """
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor"/>
  </transmission>
  <transmission name="simple_trans_bad">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="foo_motor"/>
    <joint name_BAD="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
</robot>
"""
messages = []
try:
    urdf.Robot.from_xml_string(xml_string)
except urdf.xmlr.core.ParseError, e:
    print e
    print e.path
