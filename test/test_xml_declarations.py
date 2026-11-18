import unittest

from urdf_parser_py.urdf import Robot


class XMLDeclarationTest(unittest.TestCase):
    def check_name(self, xml):
        self.assertEqual(Robot.from_xml_string(xml).name, 'caf\u00e9')
        self.assertEqual(Robot().parse(xml).name, 'caf\u00e9')

    def test_unicode_with_utf8_declaration(self):
        self.check_name('<?xml version="1.0" encoding="UTF-8"?>'
                        '<robot name="caf\u00e9"/>')

    def test_already_decoded_unicode_with_latin1_declaration(self):
        self.check_name('<?xml version="1.0" encoding="ISO-8859-1"?>'
                        '<robot name="caf\u00e9"/>')

    def test_utf8_bytes_keep_declared_encoding(self):
        self.check_name(('<?xml version="1.0" encoding="UTF-8"?>'
                         '<robot name="caf\u00e9"/>').encode('utf-8'))

    def test_latin1_bytes_keep_declared_encoding(self):
        self.check_name(('<?xml version="1.0" encoding="ISO-8859-1"?>'
                         '<robot name="caf\u00e9"/>').encode('iso-8859-1'))

    def test_unicode_without_encoding_declaration(self):
        self.check_name('<robot name="caf\u00e9"/>')
