
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

    def test_version_attribute_1_1_valid(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.1">
</robot>'''
        self.parse_and_compare(xml)

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

    def test_xml_with_UTF8_encoding(self):
        xml = b'''<?xml version="1.0" encoding="UTF-8"?>
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


class TestCapsuleGeometry(unittest.TestCase):
    @mock.patch('urdf_parser_py.xml_reflection.on_error',
                mock.Mock(side_effect=ParseException))
    def parse(self, xml):
        return urdf.Robot.from_xml_string(xml)

    def parse_and_compare(self, orig):
        xml = minidom.parseString(orig)
        robot = urdf.Robot.from_xml_string(orig)
        rewritten = minidom.parseString(robot.to_xml_string())
        self.assertTrue(xml_matches(xml, rewritten))

    def test_capsule_not_supported_in_version_1_0(self):
        """Capsule geometry should fail in URDF version 1.0."""
        xml = '''<?xml version="1.0"?>
<robot name="capsule_test" version="1.0">
  <link name="link1">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
</robot>'''
        self.assertRaises(xmlr.core.ParseError, self.parse, xml)

    def test_capsule_visual_geometry(self):
        xml = '''<?xml version="1.0"?>
<robot name="capsule_test" version="1.1">
  <link name="link1">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
</robot>'''
        robot = self.parse(xml)
        self.assertEqual("capsule_test", robot.name)
        self.assertEqual(1, len(robot.links))
        link = robot.links[0]
        self.assertIsNotNone(link.visual)
        self.assertIsInstance(link.visual.geometry, urdf.Capsule)
        self.assertEqual(0.05, link.visual.geometry.radius)
        self.assertEqual(0.5, link.visual.geometry.length)

    def test_capsule_collision_geometry(self):
        xml = '''<?xml version="1.0"?>
<robot name="capsule_collision_test" version="1.1">
  <link name="link1">
    <collision>
      <geometry>
        <capsule radius="0.1" length="1.0"/>
      </geometry>
    </collision>
  </link>
</robot>'''
        robot = self.parse(xml)
        self.assertEqual("capsule_collision_test", robot.name)
        link = robot.links[0]
        self.assertIsNotNone(link.collision)
        self.assertIsInstance(link.collision.geometry, urdf.Capsule)
        self.assertEqual(0.1, link.collision.geometry.radius)
        self.assertEqual(1.0, link.collision.geometry.length)

    def test_capsule_visual_and_collision_geometry(self):
        xml = '''<?xml version="1.0"?>
<robot name="capsule_both_test" version="1.1">
  <link name="link1">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <capsule radius="0.1" length="1.0"/>
      </geometry>
    </collision>
  </link>
</robot>'''
        robot = self.parse(xml)
        link = robot.links[0]

        # Check visual geometry
        self.assertIsInstance(link.visual.geometry, urdf.Capsule)
        self.assertEqual(0.05, link.visual.geometry.radius)
        self.assertEqual(0.5, link.visual.geometry.length)

        # Check collision geometry
        self.assertIsInstance(link.collision.geometry, urdf.Capsule)
        self.assertEqual(0.1, link.collision.geometry.radius)
        self.assertEqual(1.0, link.collision.geometry.length)

    def test_capsule_zero_values(self):
        xml = '''<?xml version="1.0"?>
<robot name="capsule_zero_test" version="1.1">
  <link name="link1">
    <visual>
      <geometry>
        <capsule radius="0.0" length="0.0"/>
      </geometry>
    </visual>
  </link>
</robot>'''
        robot = self.parse(xml)
        link = robot.links[0]
        capsule = link.visual.geometry
        self.assertIsInstance(capsule, urdf.Capsule)
        self.assertEqual(0.0, capsule.radius)
        self.assertEqual(0.0, capsule.length)

    def test_capsule_parse_and_compare(self):
        xml = '''<?xml version="1.0"?>
<robot name="capsule_roundtrip_test" version="1.1">
  <link name="link1">
    <visual>
      <geometry>
        <capsule length="0.5" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <capsule length="1.0" radius="0.1"/>
      </geometry>
    </collision>
  </link>
</robot>'''
        self.parse_and_compare(xml)

    def test_capsule_multiple_visuals(self):
        xml = '''<?xml version="1.0"?>
<robot name="capsule_multi_visual_test" version="1.1">
  <link name="link1">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.5"/>
      </geometry>
    </visual>
    <visual>
      <geometry>
        <capsule radius="0.1" length="1.0"/>
      </geometry>
    </visual>
  </link>
</robot>'''
        robot = self.parse(xml)
        link = robot.links[0]
        self.assertEqual(2, len(link.visuals))
        self.assertIsInstance(link.visuals[0].geometry, urdf.Capsule)
        self.assertEqual(0.05, link.visuals[0].geometry.radius)
        self.assertEqual(0.5, link.visuals[0].geometry.length)
        self.assertIsInstance(link.visuals[1].geometry, urdf.Capsule)
        self.assertEqual(0.1, link.visuals[1].geometry.radius)
        self.assertEqual(1.0, link.visuals[1].geometry.length)

    def test_capsule_multiple_visuals_and_collisions(self):
        """Test multiple capsule geometries in both visual and collision."""
        xml = '''<?xml version="1.0"?>
<robot name="multi_capsule_test" version="1.1">
  <link name="link1">
    <visual>
      <geometry>
        <capsule radius="0.05" length="0.5"/>
      </geometry>
    </visual>
    <visual>
      <geometry>
        <capsule radius="0.1" length="1.0"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <capsule radius="0.15" length="1.5"/>
      </geometry>
    </collision>
    <collision>
      <geometry>
        <capsule radius="0.2" length="2.0"/>
      </geometry>
    </collision>
  </link>
</robot>'''
        robot = self.parse(xml)
        link = robot.links[0]

        # Check multiple visual geometries
        self.assertEqual(2, len(link.visuals))
        self.assertIsInstance(link.visuals[0].geometry, urdf.Capsule)
        self.assertEqual(0.05, link.visuals[0].geometry.radius)
        self.assertEqual(0.5, link.visuals[0].geometry.length)
        self.assertIsInstance(link.visuals[1].geometry, urdf.Capsule)
        self.assertEqual(0.1, link.visuals[1].geometry.radius)
        self.assertEqual(1.0, link.visuals[1].geometry.length)

        # Check multiple collision geometries
        self.assertEqual(2, len(link.collisions))
        self.assertIsInstance(link.collisions[0].geometry, urdf.Capsule)
        self.assertEqual(0.15, link.collisions[0].geometry.radius)
        self.assertEqual(1.5, link.collisions[0].geometry.length)
        self.assertIsInstance(link.collisions[1].geometry, urdf.Capsule)
        self.assertEqual(0.2, link.collisions[1].geometry.radius)
        self.assertEqual(2.0, link.collisions[1].geometry.length)

    def test_capsule_programmatic_creation(self):
        """Test creating a Capsule programmatically."""
        capsule = urdf.Capsule(radius=0.05, length=0.5)
        self.assertEqual(0.05, capsule.radius)
        self.assertEqual(0.5, capsule.length)

    def test_capsule_default_values(self):
        """Test Capsule default values."""
        capsule = urdf.Capsule()
        self.assertEqual(0.0, capsule.radius)
        self.assertEqual(0.0, capsule.length)


if __name__ == '__main__':
    unittest.main()
