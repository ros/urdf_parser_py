import copy
import sys

# Backwards compatibility.
# TODO(eacousineau): Deprecate public access.
from urdf_parser_py import _now_private_property, _renamed_property
from urdf_parser_py.xml_reflection.basics import *

__all__ = [
    "reflect",
    "on_error",
    "skip_default",
    "value_types",
    "value_type_prefix",
    "start_namespace",
    "end_namespace",
    "add_type",
    "get_type",
    "make_type",
    "Path",
    "ParseError",
    "ValueType",
    "BasicType",
    "ListType",
    "VectorType",
    "RawType",
    "SimpleElementType",
    "ObjectType",
    "FactoryType",
    "DuckTypedFactory",
    "Param",
    "Attribute",
    "Element",
    "AggregateElement",
    "Info",
    "Reflection",
    "Object",
]

# Unless otherwise stated, all functions and classes are not intedned to be
# user-visible. Once deprecation is complete and public access is removed, then
# these implementation details should not need to worry about backwards
# compatibility.


def reflect(cls, *args, **kwargs):
    """
    Simple wrapper to add XML reflection to an xml_reflection.Object class
    """
    cls._XML_REFL = Reflection(*args, **kwargs)


def on_error_stderr(message):
    sys.stderr.write(message + '\n')

# What to do on an error. This can be changed to raise an exception.
on_error = on_error_stderr


skip_default = False
# defaultIfMatching = True # Not implemeneted yet

# Registering Types
value_types = {}
value_type_prefix = ''


def start_namespace(namespace):
    """
    Basic mechanism to prevent conflicts for string types for URDF and SDF
    @note Does not handle nesting!
    """
    global value_type_prefix
    value_type_prefix = namespace + '.'


def end_namespace():
    global value_type_prefix
    value_type_prefix = ''


def add_type(key, value):
    """Adds a type to the regsitry."""
    if isinstance(key, str):
        key = value_type_prefix + key
    assert key not in value_types
    value_types[key] = value


def get_type(cur_type):
    """Retrieves type from registry.
    If this is not registered, it will be implicitly registered."""
    # TODO(eacousineau): Remove confusing implicit behavior.
    if value_type_prefix and isinstance(cur_type, str):
        # See if it exists in current 'namespace'
        curKey = value_type_prefix + cur_type
        value_type = value_types.get(curKey)
    else:
        value_type = None
    if value_type is None:
        # Try again, in 'global' scope
        value_type = value_types.get(cur_type)
    if value_type is None:
        value_type = make_type(cur_type)
        add_type(cur_type, value_type)
    return value_type


def make_type(cur_type):
    """Creates a wrapping `ValueType` instance for `cur_type`."""
    # TODO(eacousineau): Remove this, and use direct instances.
    if isinstance(cur_type, ValueType):
        return cur_type
    elif isinstance(cur_type, str):
        if cur_type.startswith('vector'):
            extra = cur_type[6:]
            if extra:
                count = float(extra)
            else:
                count = None
            return VectorType(count)
        else:
            raise Exception("Invalid value type: {}".format(cur_type))
    elif cur_type == list:
        return ListType()
    elif issubclass(cur_type, Object):
        return ObjectType(cur_type)
    elif cur_type in [str, float]:
        return BasicType(cur_type)
    else:
        raise Exception("Invalid type: {}".format(cur_type))


class Path(object):
    """Records path information for producing XPath-like references in errors.
    """
    def __init__(self, tag, parent=None, suffix="", tree=None):
        self.parent = parent
        self.tag = tag
        self.suffix = suffix
        self.tree = tree # For validating general path (getting true XML path)

    def __str__(self):
        if self.parent is not None:
            return "{}/{}{}".format(self.parent, self.tag, self.suffix)
        else:
            if self.tag is not None and len(self.tag) > 0:
                return "/{}{}".format(self.tag, self.suffix)
            else:
                return self.suffix


class ParseError(Exception):
    """Indicates a parser error at a given path."""
    def __init__(self, e, path):
        self.e = e
        self.path = path
        message = "ParseError in {}:\n{}".format(self.path, self.e)
        super(ParseError, self).__init__(message)


class ValueType(object):
    """Primitive value type. Default semantics based on string parsing."""
    # TODO(eacousineau): Delegate string semantics to child class, so that this
    # can be a pure ABC.
    def read_xml_value(self, node, path):
        """Reads value from a node and returns the value.
        Can be overridden in child classes."""
        return self.from_string(node.text)

    def write_xml_value(self, node, value):
        """Writes value to a node (that must already exist).
        Can be overridden in child classes.
        """
        node.text = self.to_string(value)

    from_xml = _renamed_property('from_xml', 'read_xml_value')
    write_xml = _renamed_property('write_xml', 'write_xml_value')
    equals = _now_private_property('_equals')

    def _equals(self, a, b):
        # TODO(eacousineau): Remove this.
        return a == b


class BasicType(ValueType):
    def __init__(self, cur_type):
        self.type = cur_type

    def to_string(self, value):
        return str(value)

    def from_string(self, value):
        return self.type(value)


class ListType(ValueType):
    def to_string(self, values):
        return ' '.join(values)

    def from_string(self, text):
        return text.split()

    def _equals(self, aValues, bValues):
        return (len(aValues) == len(bValues) and
                all(a == b for (a, b) in zip(aValues, bValues)))


class VectorType(ListType):
    def __init__(self, count=None):
        self.count = count

    def check(self, values):
        if self.count is not None:
            assert len(values) == self.count, "Invalid vector length"

    def to_string(self, values):
        self.check(values)
        raw = list(map(str, values))
        return ListType.to_string(self, raw)

    def from_string(self, text):
        raw = ListType.from_string(self, text)
        self.check(raw)
        return list(map(float, raw))


class RawType(ValueType):
    """
    Simple, raw XML value. Need to bugfix putting this back into a document
    """

    def read_xml_value(self, node, path):
        return node

    def write_xml_value(self, node, value):
        # @todo Trying to insert an element at root level seems to screw up
        # pretty printing
        children = xml_children(value)
        list(map(node.append, children))
        # Copy attributes
        for (attrib_key, attrib_value) in value.attrib.iteritems():
            node.set(attrib_key, attrib_value)



class SimpleElementType(ValueType):
    """
    Extractor that retrieves data from an element, given a
    specified attribute, casted to value_type.
    """

    def __init__(self, attribute, value_type):
        self.attribute = attribute
        self.value_type = get_type(value_type)

    def read_xml_value(self, node, path):
        text = node.get(self.attribute)
        return self.value_type.from_string(text)

    def write_xml_value(self, node, value):
        text = self.value_type.to_string(value)
        node.set(self.attribute, text)


class ObjectType(ValueType):
    # Wraps an `Object`
    def __init__(self, cur_type):
        assert issubclass(cur_type, Object)
        self.type = cur_type

    def read_xml_value(self, node, path):
        obj = self.type()
        obj._read_xml(node, path)
        return obj

    def write_xml_value(self, node, obj):
        obj._write_xml(node)


class FactoryType(ValueType):
    def __init__(self, name, typeMap):
        self.name = name
        self.typeMap = typeMap
        self.nameMap = {}
        for (key, value) in typeMap.items():
            # Reverse lookup
            self.nameMap[value] = key

    def read_xml_value(self, node, path):
        cur_type = self.typeMap.get(node.tag)
        if cur_type is None:
            raise Exception("Invalid {} tag: {}".format(self.name, node.tag))
        value_type = get_type(cur_type)
        return value_type.read_xml_value(node, path)

    def get_name(self, obj):
        cur_type = type(obj)
        name = self.nameMap.get(cur_type)
        if name is None:
            raise Exception("Invalid {} type: {}".format(self.name, cur_type))
        return name

    def write_xml_value(self, node, obj):
        obj._write_xml(node)


class DuckTypedFactory(ValueType):
    def __init__(self, name, typeOrder):
        self.name = name
        assert len(typeOrder) > 0
        self.type_order = [get_type(x) for x in typeOrder]

    def read_xml_value(self, node, path):
        error_set = []
        for value_type in self.type_order:
            try:
                return value_type.read_xml_value(node, path)
            except Exception as e:
                error_set.append((value_type, e))
        # Should have returned, we encountered errors
        out = "Could not perform duck-typed parsing."
        for (value_type, e) in error_set:
            out += "\nValue Type: {}\nException: {}\n".format(value_type, e)
            raise ParseError(Exception(out), path)

    def write_xml_value(self, node, obj):
        assert isinstance(obj, Object)
        obj._write_xml(node)


class Param(object):
    """XML reflected parameter; serves as base class for Attribute and Element.

    @param xml_var: Xml name
            @todo If the value_type is an object with a tag defined in it's
                  reflection, allow it to act as the default tag name?
    @param var: Python class variable name. By default it's the same as the
                XML name
    """

    def __init__(self, xml_var, value_type, required=True, default=None,
                 var=None):
        self.value_type = get_type(value_type)
        assert isinstance(self.value_type, ValueType), self.value_type
        self.xml_var = xml_var
        if var is None:
            self.var = xml_var
        else:
            self.var = var
        self.type = None
        self.default = default
        if required:
            assert default is None, "Default does not make sense for a required field"  # noqa
        self.required = required
        self.is_aggregate = False

    def set_default(self, obj):
        if self.required:
            raise Exception("Required {} not set in XML: {}".format(self.type, self.xml_var))  # noqa
        elif not skip_default:
            setattr(obj, self.var, self.default)


class Attribute(Param):
    """Value stored in an XML attribute."""
    def __init__(self, xml_var, value_type, required=True, default=None,
                 var=None):
        Param.__init__(self, xml_var, value_type, required, default, var)
        self.type = 'attribute'

    def set_from_string(self, obj, value):
        """ Node is the parent node in this case """
        # Duplicate attributes cannot occur at this point
        setattr(obj, self.var, self.value_type.from_string(value))

    def get_value(self, obj):
        return getattr(obj, self.var)

    def add_to_xml(self, obj, node):
        value = getattr(obj, self.var)
        # Do not set with default value if value is None
        if value is None:
            if self.required:
                raise Exception("Required attribute not set in object: {}".format(self.var))  # noqa
            elif not skip_default:
                value = self.default
        # Allow value type to handle None?
        if value is not None:
            node.set(self.xml_var, self.value_type.to_string(value))

# Add option if this requires a header?
# Like <joints> <joint/> .... </joints> ???
# Not really... This would be a specific list type, not really aggregate


class Element(Param):
    """Value stored in an XML element."""
    def __init__(self, xml_var, value_type, required=True, default=None,
                 var=None, is_raw=False):
        Param.__init__(self, xml_var, value_type, required, default, var)
        self.type = 'element'
        self.is_raw = is_raw

    def set_from_xml(self, obj, node, path):
        value = self.value_type.read_xml_value(node, path)
        setattr(obj, self.var, value)

    def add_to_xml(self, obj, parent):
        value = getattr(obj, self.xml_var)
        if value is None:
            if self.required:
                raise Exception("Required element not defined in object: {}".format(self.var))  # noqa
            elif not skip_default:
                value = self.default
        if value is not None:
            self.add_scalar_to_xml(parent, value)

    def add_scalar_to_xml(self, parent, value):
        if self.is_raw:
            node = parent
        else:
            node = node_add(parent, self.xml_var)
        self.value_type.write_xml_value(node, value)


class AggregateElement(Element):
    """Indicates an element is an aggregate."""
    def __init__(self, xml_var, value_type, var=None, is_raw=False):
        if var is None:
            var = xml_var + 's'
        Element.__init__(self, xml_var, value_type, required=False, var=var,
                         is_raw=is_raw)
        self.is_aggregate = True

    def add_from_xml(self, obj, node, path):
        value = self.value_type.read_xml_value(node, path)
        obj._add_aggregate(self.xml_var, value)

    def set_default(self, obj):
        pass


class Info:
    """Small container for keeping track of what's been consumed."""
    # TODO(eacousineau): Rename to `Memo`.
    def __init__(self, node):
        self.attributes = list(node.attrib.keys())
        self.children = xml_children(node)


class Reflection(object):
    """Stores reflection information for an `Object` derived class."""
    def __init__(self, params=[], parent_cls=None, tag=None):
        """ Construct a XML reflection thing
        @param parent_cls: Parent class, to use it's reflection as well.
        @param tag: Only necessary if you intend to use Object.write_xml_doc()
                This does not override the name supplied in the reflection
                definition thing.
        """
        if parent_cls is not None:
            self.parent = parent_cls._XML_REFL
        else:
            self.parent = None
        self.tag = tag

        # Laziness for now
        attributes = []
        elements = []
        for param in params:
            if isinstance(param, Element):
                elements.append(param)
            else:
                attributes.append(param)

        self.vars = []
        self.paramMap = {}

        self.attributes = attributes
        self.attribute_map = {}
        self.required_attribute_names = []
        for attribute in attributes:
            self.attribute_map[attribute.xml_var] = attribute
            self.paramMap[attribute.xml_var] = attribute
            self.vars.append(attribute.var)
            if attribute.required:
                self.required_attribute_names.append(attribute.xml_var)

        self.elements = []
        self.element_map = {}
        self.required_element_names = []
        self.aggregates = []
        self.scalars = []
        self.scalarNames = []
        for element in elements:
            self.element_map[element.xml_var] = element
            self.paramMap[element.xml_var] = element
            self.vars.append(element.var)
            if element.required:
                self.required_element_names.append(element.xml_var)
            if element.is_aggregate:
                self.aggregates.append(element)
            else:
                self.scalars.append(element)
                self.scalarNames.append(element.xml_var)

    def set_from_xml(self, obj, node, path, info=None):
        is_final = False
        if info is None:
            is_final = True
            info = Info(node)

        if self.parent:
            path = self.parent.set_from_xml(obj, node, path, info)

        # Make this a map instead? Faster access? {name: isSet} ?
        unset_attributes = list(self.attribute_map.keys())
        unset_scalars = copy.copy(self.scalarNames)

        def get_attr_path(attribute):
            attr_path = copy.copy(path)
            attr_path.suffix += '[@{}]'.format(attribute.xml_var)
            return attr_path

        def get_element_path(element):
            element_path = Path(element.xml_var, parent = path)
            # Add an index (allow this to be overriden)
            if element.is_aggregate:
                values = obj._get_aggregate_list(element.xml_var)
                index = 1 + len(values) # 1-based indexing for W3C XPath
                element_path.suffix = "[{}]".format(index)
            return element_path

        id_var = "name"
        # Better method? Queues?
        for xml_var in copy.copy(info.attributes):
            attribute = self.attribute_map.get(xml_var)
            if attribute is not None:
                value = node.attrib[xml_var]
                attr_path = get_attr_path(attribute)
                try:
                    attribute.set_from_string(obj, value)
                    if attribute.xml_var == id_var:
                        # Add id_var suffix to current path (do not copy so it propagates)
                        path.suffix = "[@{}='{}']".format(id_var, attribute.get_value(obj))
                except ParseError, e:
                    raise
                except Exception, e:
                    raise ParseError(e, attr_path)
                unset_attributes.remove(xml_var)
                info.attributes.remove(xml_var)

        # Parse unconsumed nodes
        for child in copy.copy(info.children):
            tag = child.tag
            element = self.element_map.get(tag)
            if element is not None:
                # Name will have been set
                element_path = get_element_path(element)
                if element.is_aggregate:
                    element.add_from_xml(obj, child, element_path)
                else:
                    if tag in unset_scalars:
                        element.set_from_xml(obj, child, element_path)
                        unset_scalars.remove(tag)
                    else:
                        on_error("Scalar element defined multiple times: {}".format(tag))  # noqa
                info.children.remove(child)

        # For unset attributes and scalar elements, we should not pass the
        # attribute or element path, as those paths will implicitly not exist.
        # If we do supply it, then the user would need to manually prune the
        # XPath to try and find where the problematic parent element.
        for attribute in map(self.attribute_map.get, unset_attributes):
            try:
                attribute.set_default(obj)
            except ParseError:
                raise
            except Exception, e:
                raise ParseError(e, path) # get_attr_path(attribute.xml_var)

        for element in map(self.element_map.get, unset_scalars):
            try:
                element.set_default(obj)
            except ParseError:
                raise
            except Exception, e:
                raise ParseError(e, path) # get_element_path(element)

        if is_final:
            for xml_var in info.attributes:
                on_error('Unknown attribute "{}" in {}'.format(xml_var, path))
            for node in info.children:
                on_error('Unknown tag "{}" in {}'.format(node.tag, path))
        # Allow children parsers to adopt this current path (if modified with id_var)
        return path

    def add_to_xml(self, obj, node):
        if self.parent:
            self.parent.add_to_xml(obj, node)
        for attribute in self.attributes:
            attribute.add_to_xml(obj, node)
        for element in self.scalars:
            element.add_to_xml(obj, node)
        # Now add in aggregates
        if self.aggregates:
            obj._add_aggregates_to_xml(node)


class Object(YamlReflection):
    """Base for user-visible classes which leverage XML reflection."""
    # TODO(eacousineau): Remove most of the reflection-specific code to a
    # separate instance, if possible.
    _XML_REFL = None
    XML_REFL = _now_private_property('_XML_REFL')

    def _get_refl_vars(self):
        return self._XML_REFL.vars

    def _check_valid(self):
        pass

    check_valid = _now_private_property('_check_valid')
    pre_write_xml = _now_private_property('_pre_write_xml')
    post_read_xml = _now_private_property('_post_read_xml')
    write_xml = _now_private_property('_write_xml')
    read_xml = _now_private_property('_read_xml')
    from_xml = _now_private_property('_from_xml')

    def _pre_write_xml(self):
        """ If anything needs to be converted prior to dumping to xml
        i.e., getting the names of objects and such """
        pass

    def _write_xml(self, node):
        """ Adds contents directly to XML node """
        self._check_valid()
        self._pre_write_xml()
        self._XML_REFL.add_to_xml(self, node)

    def to_xml(self):
        """ Creates an overarching tag and adds its contents to the node """
        tag = self._XML_REFL.tag
        assert tag is not None, "Must define 'tag' in reflection to use this function"  # noqa
        doc = etree.Element(tag)
        self._write_xml(doc)
        return doc

    def to_xml_string(self, addHeader=True):
        return xml_string(self.to_xml(), addHeader)

    def _post_read_xml(self):
        pass

    def _read_xml(self, node, path):
        self._XML_REFL.set_from_xml(self, node, path)
        self._post_read_xml()
        try:
            self._check_valid()
        except ParseError:
            raise
        except Exception, e:
            raise ParseError(e, path)

    @classmethod
    def _from_xml(cls, node, path):
        cur_type = get_type(cls)
        return cur_type.read_xml_value(node, path)

    @classmethod
    def from_xml_string(cls, xml_string):
        node = etree.fromstring(xml_string)
        path = Path(cls._XML_REFL.tag, tree=etree.ElementTree(node))
        return cls._from_xml(node, path)

    @classmethod
    def from_xml_file(cls, file_path):
        xml_string = open(file_path, 'r').read()
        return cls.from_xml_string(xml_string)

    get_aggregate_list = _now_private_property('_get_aggregate_list')
    aggregate_init = _now_private_property('_aggregate_init')
    add_aggregate = _now_private_property('_add_aggregate')
    add_aggregates_to_xml = _now_private_property('_add_aggregates_to_xml')
    remove_aggregate = _now_private_property('_remove_aggregate')
    lump_aggregates = _now_private_property('_lump_aggregates')

    def _get_aggregate_list(self, xml_var):
        var = self._XML_REFL.paramMap[xml_var].var
        values = getattr(self, var)
        assert isinstance(values, list)
        return values

    def _aggregate_init(self):
        """ Must be called in constructor! """
        self._aggregate_order = []
        # Store this info in the loaded object??? Nah
        self._aggregate_type = {}

    def _add_aggregate(self, xml_var, obj):
        """ NOTE: One must keep careful track of aggregate types for this
        system.
        Can use '_lump_aggregates()' before writing if you don't care."""
        self._get_aggregate_list(xml_var).append(obj)
        self._aggregate_order.append(obj)
        self._aggregate_type[obj] = xml_var

    def _add_aggregates_to_xml(self, node):
        for value in self._aggregate_order:
            typeName = self._aggregate_type[value]
            element = self._XML_REFL.element_map[typeName]
            element.add_scalar_to_xml(node, value)

    def _remove_aggregate(self, obj):
        self._aggregate_order.remove(obj)
        xml_var = self._aggregate_type[obj]
        del self._aggregate_type[obj]
        self._get_aggregate_list(xml_var).remove(obj)

    def _lump_aggregates(self):
        """ Put all aggregate types together, just because """
        self._aggregate_init()
        for param in self._XML_REFL.aggregates:
            for obj in self._get_aggregate_list(param.xml_var):
                self._add_aggregate(param.var, obj)

    def parse(self, xml_string):
        """ Backwards compatibility """
        node = etree.fromstring(xml_string)
        path = Path(self._XML_REFL.tag, tree=etree.ElementTree(node))
        self._read_xml(node, path)
        return self


# Really common types
# TODO(eacousineau): Make this objects, not string names with weird implicit
# rules.
add_type('element_name', SimpleElementType('name', str))
add_type('element_value', SimpleElementType('value', float))

# Add in common vector types so they aren't absorbed into the namespaces
get_type('vector3')
get_type('vector4')
get_type('vector6')
