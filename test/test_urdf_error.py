from __future__ import print_function

import unittest
from urdf_parser_py import urdf
import urdf_parser_py.xml_reflection as xmlr

ParseError = xmlr.core.ParseError

class TestURDFParserError(unittest.TestCase):
    def setUp(self):
        # Manually patch "on_error" to capture errors
        self.errors = []
        def add_error(message):
            self.errors.append(message)
        xmlr.core.on_error = add_error
    
    def tearDown(self):
        xmlr.core.on_error = xmlr.core.on_error_stderr
    
    def assertLoggedErrors(self, errors, func, *args, **kwds):
        func(*args, **kwds)
        self.assertEqual(self.errors, errors)
    
    def assertParseErrorPath(self, path, func, *args, **kwds):
        with self.assertRaises(ParseError) as cm:
            func(*args, **kwds)
        e = cm.exception
        self.assertEqual(str(e.path), str(path))
        
    def test_unknown_tag(self):
        xml_string = '''<?xml version="1.0"?>
<link name="b">
    <unknown_element/>
</link>'''
        func = lambda: urdf.Link.from_xml_string(xml_string)
        errors_expected = ['Unknown tag "unknown_element" in /link[@name=\'b\']']
        self.assertLoggedErrors(errors_expected, func)
    
    def test_unknown_attribute(self):
        xml_string = '''<?xml version="1.0"?>
<link name="b" unknown_attribute="nothing useful"/>'''
        func = lambda: urdf.Link.from_xml_string(xml_string)
        errors_expected = ['Unknown attribute "unknown_attribute" in /link[@name=\'b\']']
        self.assertLoggedErrors(errors_expected, func)
    
    def test_unset_required_name_link(self):
        xml_string = '''<?xml version="1.0"?>
<link/>'''
        func = lambda: urdf.Link.from_xml_string(xml_string)
        self.assertParseErrorPath('/link', func)
    
    def test_unset_required_name_aggregate_in_robot(self):
        """ Show that an aggregate with an unset name still has its index specified """
        xml_string = '''<?xml version="1.0"?>
<robot name="test">
    <link name="good"/>
    <link name_BAD="bad"/>
</robot>'''
        func = lambda: urdf.Robot.from_xml_string(xml_string)
        self.assertParseErrorPath("/robot[@name='test']/link[2]", func)

    def test_unset_required_name_aggregate_ducktype(self):
        """ If an aggregate duck-typed element does not have a required attribute, ensure it is reported with the index """
        xml_string = '''<?xml version="1.0"?>
<robot name="test">
  <transmission name_BAD="bad_trans"/>
</robot>'''
        func = lambda: urdf.Robot.from_xml_string(xml_string)
        self.assertParseErrorPath("/robot[@name='test']/transmission[1]", func)

    def test_bad_inertial_origin_xyz(self):
        xml_string = '''<?xml version="1.0"?>
<robot name="test">
    <link name="b">
        <inertial>
            <origin xyz="0.1 0.2 BAD_NUMBER"/>
        </inertial>
    </link>
</robot>'''
        func = lambda: urdf.Robot.from_xml_string(xml_string)
        self.assertParseErrorPath("/robot[@name='test']/link[@name='b']/inertial/origin[@xyz]", func)

    def test_bad_ducktype(self):
        xml_string = '''<?xml version="1.0"?>
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
</robot>'''
        func = lambda: urdf.Robot.from_xml_string(xml_string)
        self.assertParseErrorPath("/robot[@name='test']/transmission[@name='simple_trans_bad']", func)

if __name__ == '__main__':
    unittest.main()
