import unittest
import mock
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

    def xml_and_compare(self, robot, xml):
        robot_xml_string = robot.to_xml_string()
        robot_xml = minidom.parseString(robot_xml_string)
        orig_xml = minidom.parseString(xml)
        self.assertTrue(xml_matches(robot_xml, orig_xml))

    def test_sensor_unknown(self):
        # TODO currently  <unknown/> not added when unknown sensors
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
   <sensor name="unknown sensor" group="" update_rate="20.0">
    <parent link="link1"/>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <imu />
  </sensor>
</robot>'''

        self.parse_and_compare(xml)
        robot = self.parse(xml)
        sensor = robot.sensors[0]
        self.assertEqual(sensor.name, 'unknown')

    def test_sensor_ray(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <sensor name="ray1" group="head" update_rate="20.0">
    <parent link="link1"/>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <ray>
      <horizontal samples="100" resolution="1" min_angle="-1.5708" max_angle="1.5708"/>
      <vertical samples="1" resolution="1" min_angle="0.0" max_angle="0.0"/>
    </ray>
  </sensor>
</robot>'''

        self.parse_and_compare(xml)

        robot = urdf.Robot(name='test', version='1.0')
        rayh = urdf.RayElement(samples=100, resolution=1, min_angle=-1.5708, max_angle=1.5708)
        rayv = urdf.RayElement(samples=1, resolution=1, min_angle=0.0, max_angle=0.0)
        ray = urdf.Ray()
        ray.horizontal = rayh
        ray.vertical = rayv
        sensor = urdf.SensorRay(name='ray1', parent="link1", group="head", update_rate=20.0, ray=ray)
        sensor.origin = urdf.Pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        robot.add_aggregate('sensor', sensor)
        self.xml_and_compare(robot, xml)

    def test_sensor_camera(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <sensor name="camera1" group="head" update_rate="20.0">
    <parent link="link1"/>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <camera>
      <image width="640.0" height="480.0" hfov="1.5708" format="RGB8" near="0.01" far="50.0"/>
    </camera>
  </sensor>
</robot>'''

        self.parse_and_compare(xml)

        robot = urdf.Robot(name='test', version='1.0')
        image = urdf.CameraImage(width=640.0, height=480.0, hfov=1.5708, format="RGB8", near=0.01, far=50.0)
        camera = urdf.Camera(image=image)
        sensor = urdf.SensorCamera(name='camera1', parent="link1", group="head", update_rate=20.0, camera=camera)
        sensor.origin = urdf.Pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        robot.add_aggregate('sensor', sensor)
        self.xml_and_compare(robot, xml)

    def test_sensor_tactile_taxel(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <sensor name="my_tactile_sensor" group="my_group" update_rate="100">
   <parent link="my_tactile_mount"/>
   <origin xyz="1.0 2.0 3.0" rpy="0.4 0.5 0.6"/>
   <tactile channel="my_channel">
     <taxel idx="0" xyz="0.02 0.03 0.04" rpy="0.5 0.6 0.7">
       <geometry>
         <mesh filename="package://sensor_description/model/my_tactiles/tax_lower.stl" scale="0.00101 0.00101 0.00101"/>
       </geometry>
     </taxel>
     <taxel idx="1" xyz="1.02 1.03 1.04" rpy="1.5 1.6 1.7">
       <geometry>
         <box size="12.0 13.0 14.0" />
       </geometry>
     </taxel>
      </tactile>
  </sensor>
</robot>'''
        self.parse_and_compare(xml)

        robot = urdf.Robot(name='test', version='1.0')
        meshgeometry = urdf.Mesh(filename="package://sensor_description/model/my_tactiles/tax_lower.stl",
                                 scale=[0.00101, 0.00101, 0.00101])
        taxelelement = urdf.TactileTaxelElement(0, [0.02, 0.03, 0.04], [0.5, 0.6, 0.7], meshgeometry)
        taxel = urdf.TactileTaxels(channel='my_channel')
        taxel.add_aggregate('taxel', taxelelement)
        boxgeometry = urdf.Box(size=[12.0, 13.0, 14.0])
        taxelelement2 = urdf.TactileTaxelElement(1, [1.02, 1.03, 1.04], [1.5, 1.6, 1.7], boxgeometry)
        taxel.add_aggregate('taxel', taxelelement2)
        sensor = urdf.SensorTactile(name='my_tactile_sensor', parent="my_tactile_mount", tactile=taxel)
        sensor.origin = urdf.Pose([1.0, 2.0, 3.0], [0.4, 0.5, 0.6])
        sensor.group = 'my_group'
        sensor.update_rate = 100
        robot.add_aggregate('sensor', sensor)
        self.xml_and_compare(robot, xml)


    def test_sensor_tactile_array(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <sensor name="my_tactile_sensor" group="my_group" update_rate="100">
    <parent link="my_tactile_mount"/>
    <origin xyz="1.0 2.0 3.0" rpy="0.4 0.5 0.6"/>
    <tactile channel="my_channel">
       <array rows="8" cols="16" order="row-major" 
              size="0.07 0.09" spacing="0.1 0.11" offset="0.12 0.13"/>
    </tactile>
  </sensor>
</robot>'''
        self.parse_and_compare(xml)

        robot = urdf.Robot(name='test', version='1.0')
        arrayelement = urdf.TactileArrayElement(8, 16, "row-major",
            [0.07, 0.09], [0.1,  0.11], [0.12, 0.13])
        array = urdf.TactileArray(channel='my_channel', array=arrayelement)
        sensor = urdf.SensorTactile(name='my_tactile_sensor', parent="my_tactile_mount", tactile=array)
        sensor.origin = urdf.Pose([1.0, 2.0, 3.0], [0.4, 0.5, 0.6])
        sensor.group = 'my_group'
        sensor.update_rate = 100
        robot.add_aggregate('sensor', sensor)
        self.xml_and_compare(robot, xml)


    def test_sensor_tactile_array_missing_offset(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <sensor name="my_tactile_sensor" group="my_group" update_rate="100">
    <parent link="my_tactile_mount"/>
    <origin xyz="1.0 2.0 3.0" rpy="0.4 0.5 0.6"/>
    <tactile channel="my_channel">
       <array rows="8" cols="16" order="row-major" 
              size="0.07 0.09" spacing="0.1 0.11"/>
    </tactile>
  </sensor>
</robot>'''
        robot = self.parse(xml)
        tactile = robot.sensors[0].tactile
        self.assertEqual(tactile.array.offset, [0.0, 0.0])  


    def test_sensor_tactile_array_missing_spacing(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <sensor name="my_tactile_sensor" group="my_group" update_rate="100">
    <parent link="my_tactile_mount2"/>
    <origin xyz="1.0 2.0 3.0" rpy="0.4 0.5 0.6"/>
    <tactile channel="my_channel">
       <array rows="8" cols="16" order="row-major" 
              size="0.07 2.09" offset="0.12 0.13"/>
    </tactile>
  </sensor>
</robot>'''
        robot = self.parse(xml)
        tactile = robot.sensors[0].tactile
        self.assertEqual(tactile.array.spacing , tactile.array.size)  
  
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

        robot = urdf.Robot(name='test', version='1.0')
        trans = urdf.Transmission(name='simple_trans')
        trans.type = 'transmission_interface/SimpleTransmission'
        joint = urdf.TransmissionJoint(name='foo_joint')
        joint.add_aggregate('hardwareInterface', 'EffortJointInterface')
        trans.add_aggregate('joint', joint)
        actuator = urdf.Actuator(name='foo_motor')
        actuator.mechanicalReduction = 50.0
        trans.add_aggregate('actuator', actuator)
        robot.add_aggregate('transmission', trans)
        self.xml_and_compare(robot, xml)

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

        robot = urdf.Robot(name='test', version='1.0')
        trans = urdf.Transmission(name='simple_trans')
        trans.type = 'transmission_interface/SimpleTransmission'
        joint = urdf.TransmissionJoint(name='foo_joint')
        joint.add_aggregate('hardwareInterface', 'EffortJointInterface')
        trans.add_aggregate('joint', joint)
        joint = urdf.TransmissionJoint(name='bar_joint')
        joint.add_aggregate('hardwareInterface', 'EffortJointInterface')
        joint.add_aggregate('hardwareInterface', 'EffortJointInterface')
        trans.add_aggregate('joint', joint)
        actuator = urdf.Actuator(name='foo_motor')
        actuator.mechanicalReduction = 50.0
        trans.add_aggregate('actuator', actuator)
        robot.add_aggregate('transmission', trans)
        self.xml_and_compare(robot, xml)

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

        robot = urdf.Robot(name='test', version='1.0')
        trans = urdf.Transmission(name='simple_trans')
        trans.type = 'transmission_interface/SimpleTransmission'
        joint = urdf.TransmissionJoint(name='foo_joint')
        joint.add_aggregate('hardwareInterface', 'EffortJointInterface')
        trans.add_aggregate('joint', joint)
        actuator = urdf.Actuator(name='foo_motor')
        actuator.mechanicalReduction = 50.0
        trans.add_aggregate('actuator', actuator)
        actuator = urdf.Actuator(name='bar_motor')
        trans.add_aggregate('actuator', actuator)
        robot.add_aggregate('transmission', trans)
        self.xml_and_compare(robot, xml)

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

        robot = urdf.Robot(name='test', version='1.0')
        trans = urdf.PR2Transmission(name='PR2_trans', joint='foo_joint', actuator='foo_motor', type='SimpleTransmission', mechanicalReduction=1.0)
        robot.add_aggregate('transmission', trans)
        self.xml_and_compare(robot, xml)

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

        robot = urdf.Robot(name='test', version='1.0')
        link = urdf.Link(name='link',
                         visual=urdf.Visual(geometry=urdf.Cylinder(length=1, radius=1),
                                              material=urdf.Material(name='mat')))
        robot.add_link(link)
        self.xml_and_compare(robot, xml)

        robot = urdf.Robot(name='test', version='1.0')
        link = urdf.Link(name='link')
        link.visual = urdf.Visual(geometry=urdf.Cylinder(length=1, radius=1),
                                  material=urdf.Material(name='mat'))
        robot.add_link(link)
        self.xml_and_compare(robot, xml)

    def test_robot_material(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <material name="mat">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
</robot>'''
        self.parse_and_compare(xml)

        robot = urdf.Robot(name='test', version='1.0')
        material = urdf.Material(name='mat', color=urdf.Color([0.0, 0.0, 0.0, 1.0]))
        robot.add_aggregate('material', material)
        self.xml_and_compare(robot, xml)

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

        robot = urdf.Robot(name='test', version='1.0')
        link = urdf.Link(name='link')
        link.visual = urdf.Visual(geometry=urdf.Cylinder(length=1, radius=1),
                                  material=urdf.Material(name='mat'))
        link.visual = urdf.Visual(geometry=urdf.Cylinder(length=4, radius=0.5),
                                  material=urdf.Material(name='mat2'))
        robot.add_link(link)
        self.xml_and_compare(robot, xml)

    def test_visual_with_name(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <link name="link">
    <visual name="alice">
      <geometry>
        <cylinder length="1" radius="1"/>
      </geometry>
      <material name="mat"/>
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

        robot = urdf.Robot(name='test', version='1.0')
        link = urdf.Link(name='link')
        link.collision = urdf.Collision(geometry=urdf.Cylinder(length=1, radius=1))
        link.collision = urdf.Collision(geometry=urdf.Cylinder(length=4, radius=0.5))
        robot.add_link(link)
        self.xml_and_compare(robot, xml)

    def test_collision_with_name(self):
        xml = '''<?xml version="1.0"?>
<robot name="test" version="1.0">
  <link name="link">
    <collision name="alice">
      <geometry>
        <cylinder length="1" radius="1"/>
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

    def test_xml_and_urdfdom_robot_compatible_with_kinetic(self):
        robot = urdf.Robot(name='test', version='1.0')
        link = urdf.Link(name='link')
        link.visual = urdf.Visual(geometry=urdf.Cylinder(length=1, radius=1),
                                  material=urdf.Material(name='mat'))
        link.visual = urdf.Visual(geometry=urdf.Cylinder(length=4, radius=0.5),
                                  material=urdf.Material(name='mat2'))
        link.collision = urdf.Collision(geometry=urdf.Cylinder(length=1, radius=1))
        link.collision = urdf.Collision(geometry=urdf.Cylinder(length=4, radius=0.5))
        robot.add_link(link)
        link = urdf.Link(name='link2')
        robot.add_link(link)
        #
        robot_xml_string = robot.to_xml_string()
        robot_xml = minidom.parseString(robot_xml_string)
        orig_xml = minidom.parseString(self.xml)
        self.assertTrue(xml_matches(robot_xml, orig_xml))

    def test_xml_and_urdfdom_robot_only_supported_since_melodic(self):
        robot = urdf.Robot(name='test', version='1.0')
        link = urdf.Link(name='link')
        link.add_aggregate('visual', urdf.Visual(geometry=urdf.Cylinder(length=1, radius=1),
                                                 material=urdf.Material(name='mat')))
        link.add_aggregate('visual', urdf.Visual(geometry=urdf.Cylinder(length=4, radius=0.5),
                                                 material=urdf.Material(name='mat2')))
        link.add_aggregate('collision', urdf.Collision(geometry=urdf.Cylinder(length=1, radius=1)))
        link.add_aggregate('collision', urdf.Collision(geometry=urdf.Cylinder(length=4, radius=0.5)))
        robot.add_link(link)
        link = urdf.Link(name='link2')
        robot.add_link(link)
        #
        robot_xml_string = robot.to_xml_string()
        robot_xml = minidom.parseString(robot_xml_string)
        orig_xml = minidom.parseString(self.xml)
        self.assertTrue(xml_matches(robot_xml, orig_xml))


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
