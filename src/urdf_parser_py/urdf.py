from urdf_parser_py import _now_private_property
import urdf_parser_py._xml_reflection as _xmlr

_xmlr.start_namespace('urdf')

_xmlr.add_type('element_link', _xmlr.SimpleElementType('link', str))
_xmlr.add_type('element_xyz', _xmlr.SimpleElementType('xyz', 'vector3'))


class Pose(_xmlr.Object):
    def __init__(self, xyz=None, rpy=None):
        self.xyz = xyz
        self.rpy = rpy

    def _check_valid(self):
        assert (self.xyz is None or len(self.xyz) == 3 and
                self.rpy is None or len(self.rpy) == 3)

    # Aliases for backwards compatibility
    @property
    def rotation(self): return self.rpy

    @rotation.setter
    def rotation(self, value): self.rpy = value

    @property
    def position(self): return self.xyz

    @position.setter
    def position(self, value): self.xyz = value


_xmlr.reflect(Pose, tag='origin', params=[
    _xmlr.Attribute('xyz', 'vector3', required=False, default=[0, 0, 0]),
    _xmlr.Attribute('rpy', 'vector3', required=False, default=[0, 0, 0])
])


# Common stuff
_name_attribute = _xmlr.Attribute('name', str)
_origin_element = _xmlr.Element('origin', Pose, required=False)


class Color(_xmlr.Object):
    def __init__(self, *args):
        # What about named colors?
        count = len(args)
        if count == 4 or count == 3:
            self.rgba = args
        elif count == 1:
            self.rgba = args[0]
        elif count == 0:
            self.rgba = None
        if self.rgba is not None:
            if len(self.rgba) == 3:
                self.rgba += [1.]
            if len(self.rgba) != 4:
                raise Exception('Invalid color argument count')


_xmlr.reflect(Color, tag='color', params=[
    _xmlr.Attribute('rgba', 'vector4')
])


class JointDynamics(_xmlr.Object):
    def __init__(self, damping=None, friction=None):
        self.damping = damping
        self.friction = friction


_xmlr.reflect(JointDynamics, tag='dynamics', params=[
    _xmlr.Attribute('damping', float, required=False),
    _xmlr.Attribute('friction', float, required=False)
])


class Box(_xmlr.Object):
    def __init__(self, size=None):
        self.size = size


_xmlr.reflect(Box, tag='box', params=[
    _xmlr.Attribute('size', 'vector3')
])


class Cylinder(_xmlr.Object):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length


_xmlr.reflect(Cylinder, tag='cylinder', params=[
    _xmlr.Attribute('radius', float),
    _xmlr.Attribute('length', float)
])


class Sphere(_xmlr.Object):
    def __init__(self, radius=0.0):
        self.radius = radius


_xmlr.reflect(Sphere, tag='sphere', params=[
    _xmlr.Attribute('radius', float)
])


class Mesh(_xmlr.Object):
    def __init__(self, filename=None, scale=None):
        self.filename = filename
        self.scale = scale


_xmlr.reflect(Mesh, tag='mesh', params=[
    _xmlr.Attribute('filename', str),
    _xmlr.Attribute('scale', 'vector3', required=False)
])


class _GeometricType(_xmlr.ValueType):
    def __init__(self):
        self.factory = _xmlr.FactoryType('geometric', {
            'box': Box,
            'cylinder': Cylinder,
            'sphere': Sphere,
            'mesh': Mesh
        })

    def read_xml_value(self, node, path):
        children = _xmlr.xml_children(node)
        assert len(children) == 1, 'One element only for geometric'
        return self.factory.read_xml_value(children[0], path=path)

    def write_xml_value(self, node, obj):
        name = self.factory.get_name(obj)
        child = _xmlr.node_add(node, name)
        obj._write_xml(child)


# TODO(eacousineau): Deprecate public access.
GeometricType = _GeometricType


_xmlr.add_type('geometric', _GeometricType())


class Collision(_xmlr.Object):
    def __init__(self, geometry=None, origin=None):
        self.geometry = geometry
        self.origin = origin


_xmlr.reflect(Collision, tag='collision', params=[
    _origin_element,
    _xmlr.Element('geometry', 'geometric')
])


class Texture(_xmlr.Object):
    def __init__(self, filename=None):
        self.filename = filename


_xmlr.reflect(Texture, tag='texture', params=[
    _xmlr.Attribute('filename', str)
])


class Material(_xmlr.Object):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    def _check_valid(self):
        if self.color is None and self.texture is None:
            _xmlr.on_error("Material has neither a color nor texture.")


_xmlr.reflect(Material, tag='material', params=[
    _name_attribute,
    _xmlr.Element('color', Color, False),
    _xmlr.Element('texture', Texture, False)
])


class LinkMaterial(Material):
    def _check_valid(self):
        pass


class Visual(_xmlr.Object):
    def __init__(self, geometry=None, material=None, origin=None):
        self.geometry = geometry
        self.material = material
        self.origin = origin


_xmlr.reflect(Visual, tag='visual', params=[
    _origin_element,
    _xmlr.Element('geometry', 'geometric'),
    _xmlr.Element('material', LinkMaterial, False)
])


class Inertia(_xmlr.Object):
    KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz

    def to_matrix(self):
        return [
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz]]


_xmlr.reflect(Inertia, tag='inertia',
              params=[_xmlr.Attribute(key, float) for key in Inertia.KEYS])


class Inertial(_xmlr.Object):
    def __init__(self, mass=0.0, inertia=None, origin=None):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin


_xmlr.reflect(Inertial, tag='inertial', params=[
    _origin_element,
    _xmlr.Element('mass', 'element_value'),
    _xmlr.Element('inertia', Inertia, required=False)
])


# FIXME: we are missing the reference position here.
class JointCalibration(_xmlr.Object):
    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling


_xmlr.reflect(JointCalibration, tag='calibration', params=[
    _xmlr.Attribute('rising', float, required=False, default=0),
    _xmlr.Attribute('falling', float, required=False, default=0)
])


class JointLimit(_xmlr.Object):
    def __init__(self, effort=None, velocity=None, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper


_xmlr.reflect(JointLimit, tag='limit', params=[
    _xmlr.Attribute('effort', float),
    _xmlr.Attribute('lower', float, required=False, default=0),
    _xmlr.Attribute('upper', float, required=False, default=0),
    _xmlr.Attribute('velocity', float)
])

# FIXME: we are missing __str__ here.


class JointMimic(_xmlr.Object):
    def __init__(self, joint_name=None, multiplier=None, offset=None):
        self.joint = joint_name
        self.multiplier = multiplier
        self.offset = offset


_xmlr.reflect(JointMimic, tag='mimic', params=[
    _xmlr.Attribute('joint', str),
    _xmlr.Attribute('multiplier', float, required=False),
    _xmlr.Attribute('offset', float, required=False)
])


class SafetyController(_xmlr.Object):
    def __init__(self, velocity=None, position=None, lower=None, upper=None):
        self.k_velocity = velocity
        self.k_position = position
        self.soft_lower_limit = lower
        self.soft_upper_limit = upper


_xmlr.reflect(SafetyController, tag='safety_controller', params=[
    _xmlr.Attribute('k_velocity', float),
    _xmlr.Attribute('k_position', float, required=False, default=0),
    _xmlr.Attribute('soft_lower_limit', float, required=False, default=0),
    _xmlr.Attribute('soft_upper_limit', float, required=False, default=0)
])


class Joint(_xmlr.Object):
    TYPES = ['unknown', 'revolute', 'continuous', 'prismatic',
             'floating', 'planar', 'fixed']

    def __init__(self, name=None, parent=None, child=None, joint_type=None,
                 axis=None, origin=None,
                 limit=None, dynamics=None, safety_controller=None,
                 calibration=None, mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = joint_type
        self.axis = axis
        self.origin = origin
        self.limit = limit
        self.dynamics = dynamics
        self.safety_controller = safety_controller
        self.calibration = calibration
        self.mimic = mimic

    def _check_valid(self):
        assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def joint_type(self): return self.type

    @joint_type.setter
    def joint_type(self, value): self.type = value

_xmlr.reflect(Joint, tag='joint', params=[
    _name_attribute,
    _xmlr.Attribute('type', str),
    _origin_element,
    _xmlr.Element('axis', 'element_xyz', required=False),
    _xmlr.Element('parent', 'element_link'),
    _xmlr.Element('child', 'element_link'),
    _xmlr.Element('limit', JointLimit, required=False),
    _xmlr.Element('dynamics', JointDynamics, required=False),
    _xmlr.Element('safety_controller', SafetyController, required=False),
    _xmlr.Element('calibration', JointCalibration, required=False),
    _xmlr.Element('mimic', JointMimic, required=False),
])


class Link(_xmlr.Object):
    def __init__(self, name=None, visual=None, inertial=None, collision=None,
                 origin=None):
        self._aggregate_init()
        self.name = name
        self.visuals = []
        self.inertial = inertial
        self.collisions = []
        self.origin = origin

    def __get_visual(self):
        """Return the first visual or None."""
        if self.visuals:
            return self.visuals[0]

    def __set_visual(self, visual):
        """Set the first visual."""
        if self.visuals:
            self.visuals[0] = visual
        else:
            self.visuals.append(visual)

    def __get_collision(self):
        """Return the first collision or None."""
        if self.collisions:
            return self.collisions[0]

    def __set_collision(self, collision):
        """Set the first collision."""
        if self.collisions:
            self.collisions[0] = collision
        else:
            self.collisions.append(collision)

    # Properties for backwards compatibility
    visual = property(__get_visual, __set_visual)
    collision = property(__get_collision, __set_collision)


_xmlr.reflect(Link, tag='link', params=[
    _name_attribute,
    _origin_element,
    _xmlr.AggregateElement('visual', Visual),
    _xmlr.AggregateElement('collision', Collision),
    _xmlr.Element('inertial', Inertial, required=False),
])


class PR2Transmission(_xmlr.Object):
    def __init__(self, name=None, joint=None, actuator=None, type=None,
                 mechanicalReduction=1):
        self.name = name
        self.type = type
        self.joint = joint
        self.actuator = actuator
        self.mechanicalReduction = mechanicalReduction


_xmlr.reflect(PR2Transmission, tag='pr2_transmission', params=[
    _name_attribute,
    _xmlr.Attribute('type', str),
    _xmlr.Element('joint', 'element_name'),
    _xmlr.Element('actuator', 'element_name'),
    _xmlr.Element('mechanicalReduction', float)
])


class Actuator(_xmlr.Object):
    def __init__(self, name=None, mechanicalReduction=1):
        self.name = name
        self.mechanicalReduction = None


_xmlr.reflect(Actuator, tag='actuator', params=[
    _name_attribute,
    _xmlr.Element('mechanicalReduction', float, required=False)
])


class TransmissionJoint(_xmlr.Object):
    def __init__(self, name=None):
        self._aggregate_init()
        self.name = name
        self.hardwareInterfaces = []

    def _check_valid(self):
        assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"


_xmlr.reflect(TransmissionJoint, tag='joint', params=[
    _name_attribute,
    _xmlr.AggregateElement('hardwareInterface', str),
])


class Transmission(_xmlr.Object):
    """ New format: http://wiki.ros.org/urdf/XML/Transmission """

    def __init__(self, name=None):
        self._aggregate_init()
        self.name = name
        self.joints = []
        self.actuators = []

    def _check_valid(self):
        assert len(self.joints) > 0, "no joint defined"
        assert len(self.actuators) > 0, "no actuator defined"


_xmlr.reflect(Transmission, tag='new_transmission', params=[
    _name_attribute,
    _xmlr.Element('type', str),
    _xmlr.AggregateElement('joint', TransmissionJoint),
    _xmlr.AggregateElement('actuator', Actuator)
])

_xmlr.add_type('transmission',
              _xmlr.DuckTypedFactory('transmission',
                                    [Transmission, PR2Transmission]))


class Robot(_xmlr.Object):
    def __init__(self, name=None):
        self._aggregate_init()

        self.name = name
        self.joints = []
        self.links = []
        self.materials = []
        self.gazebos = []
        self.transmissions = []

        self.joint_map = {}
        self.link_map = {}

        self.parent_map = {}
        self.child_map = {}

    def _add_aggregate(self, typeName, elem):
        _xmlr.Object._add_aggregate(self, typeName, elem)

        if typeName == 'joint':
            joint = elem
            self.joint_map[joint.name] = joint
            self.parent_map[joint.child] = (joint.name, joint.parent)
            if joint.parent in self.child_map:
                self.child_map[joint.parent].append((joint.name, joint.child))
            else:
                self.child_map[joint.parent] = [(joint.name, joint.child)]
        elif typeName == 'link':
            link = elem
            self.link_map[link.name] = link

    def add_link(self, link):
        self._add_aggregate('link', link)

    def add_joint(self, joint):
        self._add_aggregate('joint', joint)

    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            (joint, parent) = self.parent_map[link]
            if joints:
                if fixed or self.joint_map[joint].joint_type != 'fixed':
                    chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    def get_root(self):
        root = None
        for link in self.link_map:
            if link not in self.parent_map:
                assert root is None, "Multiple roots detected, invalid URDF."
                root = link
        assert root is not None, "No roots detected, invalid URDF."
        return root

    @classmethod
    def from_parameter_server(cls, key='robot_description'):
        """
        Retrieve the robot model on the parameter server
        and parse it to create a URDF robot structure.

        Warning: this requires roscore to be running.
        """
        # Could move this into xml_reflection
        import rospy
        return cls.from_xml_string(rospy.get_param(key))


_xmlr.reflect(Robot, tag='robot', params=[
    _xmlr.Attribute('name', str, required=False),  # Is 'name' a required attribute?
    _xmlr.AggregateElement('link', Link),
    _xmlr.AggregateElement('joint', Joint),
    _xmlr.AggregateElement('gazebo', _xmlr.RawType()),
    _xmlr.AggregateElement('transmission', 'transmission'),
    _xmlr.AggregateElement('material', Material)
])

# Make an alias
URDF = Robot

_xmlr.end_namespace()
