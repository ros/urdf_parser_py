
import unittest
from unittest import mock
import os
import sys

# Add path to import xml_matching
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),
                                             '../src')))

from xml.dom import minidom  # noqa
from xml_matching import xml_matches  # noqa
from urdf_parser_py import urdf  # noqa
import urdf_parser_py.xml_reflection as xmlr

class ParseException(xmlr.core.ParseError):
    def __init__(self, e = "", path = ""):
        super(ParseException, self).__init__(e, path)


class TestURDFParser(unittest.TestCase):
    @mock.patch('urdf_parser_py.xml_reflection.on_error',
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
<robot name="test" version="1.0">
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
<robot name="test" version="1.0">
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
<robot name="test" version="1.0">
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
<robot name="test" version="1.0">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
  </transmission>
</robot>'''
        self.assertRaises(xmlr.core.ParseError, self.parse, xml)

    def test_new_transmission_missing_actuator(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <transmission name="simple_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="foo_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
  </transmission>
</robot>'''
        self.assertRaises(xmlr.core.ParseError, self.parse, xml)

    def test_old_transmission(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <transmission name="PR2_trans" type="SimpleTransmission">
    <joint name="foo_joint"/>
    <actuator name="foo_motor"/>
    <mechanicalReduction>1.0</mechanicalReduction>
  </transmission>
</robot>'''
        self.parse_and_compare(xml)

    def test_link_material_missing_color_and_texture(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
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
<robot name="test" version="1.0">
  <material name="mat">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
</robot>'''
        self.parse_and_compare(xml)

    def test_robot_material_missing_color_and_texture(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <material name="mat"/>
</robot>'''
        self.assertRaises(xmlr.core.ParseError, self.parse, xml)

    def test_link_multiple_visual(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
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
<robot name="test" version="1.0">
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

    def test_version_attribute_not_enough_dots(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_too_many_dots(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0.0">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_not_enough_numbers(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_no_major_number(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version=".0">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_negative_major_number(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="-1.0">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_negative_minor_number(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.-0">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_dots_no_numbers(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="a.c">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_dots_one_number(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.c">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_trailing_junk(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0~pre6">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_correct(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
</robot>'''
        self.parse_and_compare(xml)

    def test_version_attribute_invalid(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="foo">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

    def test_version_attribute_invalid_version(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="2.0">
</robot>'''
        self.assertRaises(ValueError, self.parse, xml)

class LinkOriginTestCase(unittest.TestCase):
    @mock.patch('urdf_parser_py.xml_reflection.on_error',
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
        self.assertEqual(origin.xyz, [0, 0, 0])
        self.assertEqual(origin.rpy, [0, 0, 0])

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
        self.assertEqual(origin.xyz, [1, 2, 3])
        self.assertEqual(origin.rpy, [0, 0, 0])


class LinkMultiVisualsAndCollisionsTest(unittest.TestCase):

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
        self.assertEqual(2, len(robot.links[0].visuals))
        self.assertEqual(
            id(robot.links[0].visuals[0]), id(robot.links[0].visual))

        self.assertEqual(None, robot.links[1].visual)

        dummyObject = set()
        robot.links[0].visual = dummyObject
        self.assertEqual(id(dummyObject), id(robot.links[0].visuals[0]))

    def test_multi_collision_access(self):
        robot = urdf.Robot.from_xml_string(self.xml)
        self.assertEqual(2, len(robot.links[0].collisions))
        self.assertEqual(
            id(robot.links[0].collisions[0]), id(robot.links[0].collision))

        self.assertEqual(None, robot.links[1].collision)

        dummyObject = set()
        robot.links[0].collision = dummyObject
        self.assertEqual(id(dummyObject), id(robot.links[0].collisions[0]))


class TestCreateNew(unittest.TestCase):
    def test_new_urdf(self):
        testcase = urdf.URDF('robot_name').to_xml()
        self.assertTrue('name' in testcase.keys())
        self.assertTrue('version' in testcase.keys())
        self.assertEqual(testcase.get('name'), 'robot_name')
        self.assertEqual(testcase.get('version'), '1.0')

    def test_new_urdf_with_version(self):
        testcase = urdf.URDF('robot_name', '1.0').to_xml()
        self.assertTrue('name' in testcase.keys())
        self.assertTrue('version' in testcase.keys())
        self.assertEqual(testcase.get('name'), 'robot_name')
        self.assertEqual(testcase.get('version'), '1.0')


if __name__ == '__main__':
    unittest.main()
