from __future__ import print_function

from os.path import abspath, dirname, join
import unittest
import sys
import warnings
from xml.dom import minidom  # noqa

import mock

# Add path to import xml_matching
# TODO(eacousineau): Can CTest somehow provide this?
TEST_DIR = dirname(abspath(__file__))
sys.path.append(TEST_DIR)
sys.path.append(join(dirname(TEST_DIR), 'src'))

from urdf_parser_py import urdf  # noqa
import urdf_parser_py._xml_reflection as _xmlr
from xml_matching import xml_matches  # noqa
from test_base import TestBase


class ParseException(_xmlr.ParseError):
    def __init__(self, e = "", path = ""):
        super(ParseException, self).__init__(e, path)


class TestURDFParser(TestBase):
    @mock.patch('urdf_parser_py._xml_reflection.on_error',
                mock.Mock(side_effect=ParseException))
    def parse(self, xml):
        return urdf.Robot.from_xml_string(xml)

    def parse_and_compare(self, orig):
        xml = minidom.parseString(orig)
        robot = urdf.Robot.from_xml_string(orig)
        rewritten = minidom.parseString(robot.to_xml_string())
        self.assertTrue(xml_matches(xml, rewritten))

    def test_new_transmission(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor">
      <mechanicalReduction>50.0</mechanicalReduction>
    </actuator>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_new_transmission_multiple_joints(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <joint name="bar_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor">
      <mechanicalReduction>50.0</mechanicalReduction>
    </actuator>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_new_transmission_multiple_actuators(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_motor">
      <mechanicalReduction>50.0</mechanicalReduction>
    </actuator>
    <actuator name="bar_motor"/>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_new_transmission_missing_joint(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
  </transmission>
</robot>'''
        self.assertRaises(Exception, self.parse, xml)

    def test_new_transmission_missing_actuator(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
</robot>'''
        self.assertRaises(Exception, self.parse, xml)

    def test_old_transmission(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name="PR2_trans" type="SimpleTransmission">
    <joint name="foo_joint"/>
    <actuator name="foo_motor"/>
    <mechanicalReduction>1.0</mechanicalReduction>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_link_material_missing_color_and_texture(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <link name="link">
    <visual>
      <geometry>
        <cylinder length="1" radius="1"/>
      </geometry>
      <material name="mat"/>
    </visual>
  </link>
</robot>'''
        self.parse_and_compare(xml)

    def test_robot_material(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <material name="mat">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
</robot>'''
        self.parse_and_compare(xml)

    def test_robot_material_missing_color_and_texture(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <material name="mat"/>
</robot>'''
        self.assertRaises(ParseException, self.parse, xml)

    def test_link_multiple_visual(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <link name="link">
    <visual>
      <geometry>
        <cylinder length="1" radius="1"/>
      </geometry>
      <material name="mat"/>
    </visual>
    <visual>
      <geometry>
        <cylinder length="4" radius="0.5"/>
      </geometry>
      <material name="mat2"/>
    </visual>
  </link>
</robot>'''
        self.parse_and_compare(xml)

    def test_link_multiple_collision(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <link name="link">
    <collision>
      <geometry>
        <cylinder length="1" radius="1"/>
      </geometry>
    </collision>
    <collision>
      <geometry>
        <cylinder length="4" radius="0.5"/>
      </geometry>
    </collision>
  </link>
</robot>'''
        self.parse_and_compare(xml)


class LinkOriginTestCase(TestBase):
    @mock.patch('urdf_parser_py._xml_reflection.on_error',
                mock.Mock(side_effect=ParseException))
    def parse(self, xml):
        return urdf.Robot.from_xml_string(xml)

    def test_robot_link_defaults(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <link name="test_link">
    <inertial>
      <mass value="10.0"/>
      <origin/>
    </inertial>
  </link>
</robot>'''
        robot = self.parse(xml)
        origin = robot.links[0].inertial.origin
        self.assertEquals(origin.xyz, [0, 0, 0])
        self.assertEquals(origin.rpy, [0, 0, 0])

    def test_robot_link_defaults_xyz_set(self):
        xml = '''<?xml version="1.0"?>
<robot name="test">
  <link name="test_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="1 2 3"/>
    </inertial>
  </link>
</robot>'''
        robot = self.parse(xml)
        origin = robot.links[0].inertial.origin
        self.assertEquals(origin.xyz, [1, 2, 3])
        self.assertEquals(origin.rpy, [0, 0, 0])


class LinkMultiVisualsAndCollisionsTest(TestBase):

    xml = '''<?xml version="1.0"?>
<robot name="test">
  <link name="link">
    <visual>
      <geometry>
        <cylinder length="1" radius="1"/>
      </geometry>
      <material name="mat"/>
    </visual>
    <visual>
      <geometry>
        <cylinder length="4" radius="0.5"/>
      </geometry>
      <material name="mat2"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="1" radius="1"/>
      </geometry>
    </collision>
    <collision>
      <geometry>
        <cylinder length="4" radius="0.5"/>
      </geometry>
    </collision>
  </link>
  <link name="link2"/>
</robot>'''

    def test_multi_visual_access(self):
        robot = urdf.Robot.from_xml_string(self.xml)
        self.assertEquals(2, len(robot.links[0].visuals))
        self.assertEqual(
            id(robot.links[0].visuals[0]), id(robot.links[0].visual))

        self.assertEquals(None, robot.links[1].visual)

        dummyObject = set()
        robot.links[0].visual = dummyObject
        self.assertEquals(id(dummyObject), id(robot.links[0].visuals[0]))

    def test_multi_collision_access(self):
        robot = urdf.Robot.from_xml_string(self.xml)
        self.assertEquals(2, len(robot.links[0].collisions))
        self.assertEqual(
            id(robot.links[0].collisions[0]), id(robot.links[0].collision))

        self.assertEquals(None, robot.links[1].collision)

        dummyObject = set()
        robot.links[0].collision = dummyObject
        self.assertEquals(id(dummyObject), id(robot.links[0].collisions[0]))


class TestDeprecation(TestBase):
    """Tests deprecated interfaces."""
    def test_deprecated_properties(self):
        with self.catch_warnings() as w:
            urdf.Robot.XML_REFL
            urdf.Pose().check_valid()
            self.assertEqual(len(w), 2)
            self.assertIn("'XML_REFL'", str(w[0].message))
            self.assertIn("'check_valid'", str(w[1].message))


class TestExampleRobots(TestBase):
    """Tests that some samples files can be parsed without error."""
    @unittest.skip("Badly formatted transmissions")
    def test_calvin_urdf(self):
        urdf.Robot.from_xml_file(join(TEST_DIR, 'calvin/calvin.urdf'))

    def test_romeo_urdf(self):
        urdf.Robot.from_xml_file(join(TEST_DIR, 'romeo/romeo.urdf'))


if __name__ == '__main__':
    unittest.main()
