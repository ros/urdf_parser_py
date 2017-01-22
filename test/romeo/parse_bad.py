#!/usr/bin/env python

import sys
import argparse

from urdf_parser_py import urdf


xml_string = """
<link name="b">
    <unknown_element/>
</link>
"""
# urdf.Link.from_xml_string(xml_string)

xml_string = """
<robot name="test">
    <link na_me="b">
        <unknown_element/>
    </link>
</robot>
"""
# urdf.Robot.from_xml_string(xml_string)

xml_string = """
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor"/>
  </transmission>
  <transmission name="simple_trans_2">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="foo_motor"/>
    <joint na_me="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
</robot>
"""
urdf.Robot.from_xml_string(xml_string)
