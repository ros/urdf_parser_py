from urdf_parser_py.xml_reflection.basics import *
from urdf_parser_py.xml_reflection.core import *
import urdf_parser_py.xml_reflection as xmlr

# Add a 'namespace' for names to avoid a conflict between URDF and SDF?
# A type registry? How to scope that? Just make a 'global' type pointer?
# Or just qualify names? urdf.geometric, sdf.geometric

xmlr.start_namespace('urdf')

xmlr.add_type('element_link', xmlr.SimpleElementType('link', str))
xmlr.add_type('element_xyz', xmlr.SimpleElementType('xyz', 'vector3'))

verbose = True


class Pose(xmlr.Object):
    def __init__(self, xyz=None, rpy=None):
        self.xyz = xyz
        self.rpy = rpy

    def check_valid(self):
        assert (self.xyz is None or len(self.xyz) == 3) and \
            (self.rpy is None or len(self.rpy) == 3)

    # Aliases for backwards compatibility
    @property
    def rotation(self): return self.rpy

    @rotation.setter
    def rotation(self, value): self.rpy = value

    @property
    def position(self): return self.xyz

    @position.setter
    def position(self, value): self.xyz = value


xmlr.reflect(Pose, tag='origin', params=[
    xmlr.Attribute('xyz', 'vector3', False, default=[0, 0, 0]),
    xmlr.Attribute('rpy', 'vector3', False, default=[0, 0, 0])
])


# Common stuff
name_attribute = xmlr.Attribute('name', str)
origin_element = xmlr.Element('origin', Pose, False)


class Color(xmlr.Object):
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


xmlr.reflect(Color, tag='color', params=[
    xmlr.Attribute('rgba', 'vector4')
])


class JointDynamics(xmlr.Object):
    def __init__(self, damping=None, friction=None):
        self.damping = damping
        self.friction = friction


xmlr.reflect(JointDynamics, tag='dynamics', params=[
    xmlr.Attribute('damping', float, False),
    xmlr.Attribute('friction', float, False)
])


class Box(xmlr.Object):
    def __init__(self, size=None):
        self.size = size


xmlr.reflect(Box, tag='box', params=[
    xmlr.Attribute('size', 'vector3')
])


class Cylinder(xmlr.Object):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length


xmlr.reflect(Cylinder, tag='cylinder', params=[
    xmlr.Attribute('radius', float),
    xmlr.Attribute('length', float)
])


class Sphere(xmlr.Object):
    def __init__(self, radius=0.0):
        self.radius = radius


xmlr.reflect(Sphere, tag='sphere', params=[
    xmlr.Attribute('radius', float)
])


class Mesh(xmlr.Object):
    def __init__(self, filename=None, scale=None):
        self.filename = filename
        self.scale = scale


xmlr.reflect(Mesh, tag='mesh', params=[
    xmlr.Attribute('filename', str),
    xmlr.Attribute('scale', 'vector3', required=False)
])


class GeometricType(xmlr.ValueType):
    def __init__(self):
        self.factory = xmlr.FactoryType('geometric', {
            'box': Box,
            'cylinder': Cylinder,
            'sphere': Sphere,
            'mesh': Mesh
        })

    def from_xml(self, node, path):
        children = xml_children(node)
        assert len(children) == 1, 'One element only for geometric'
        return self.factory.from_xml(children[0], path=path)

    def write_xml(self, node, obj):
        name = self.factory.get_name(obj)
        child = node_add(node, name)
        obj.write_xml(child)


xmlr.add_type('geometric', GeometricType())


class Collision(xmlr.Object):
    def __init__(self, geometry=None, origin=None, name=None):
        self.geometry = geometry
        self.name = name
        self.origin = origin


xmlr.reflect(Collision, tag='collision', params=[
    xmlr.Attribute('name', str, False),
    origin_element,
    xmlr.Element('geometry', 'geometric')
])


class Texture(xmlr.Object):
    def __init__(self, filename=None):
        self.filename = filename


xmlr.reflect(Texture, tag='texture', params=[
    xmlr.Attribute('filename', str)
])


class Material(xmlr.Object):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    def check_valid(self):
        if self.color is None and self.texture is None:
            xmlr.on_error("Material has neither a color nor texture.")


xmlr.reflect(Material, tag='material', params=[
    name_attribute,
    xmlr.Element('color', Color, False),
    xmlr.Element('texture', Texture, False)
])


class LinkMaterial(Material):
    def check_valid(self):
        pass


class Visual(xmlr.Object):
    def __init__(self, geometry=None, material=None, origin=None, name=None):
        self.geometry = geometry
        self.material = material
        self.name = name
        self.origin = origin


xmlr.reflect(Visual, tag='visual', params=[
    xmlr.Attribute('name', str, False),
    origin_element,
    xmlr.Element('geometry', 'geometric'),
    xmlr.Element('material', LinkMaterial, False)
])


class Inertia(xmlr.Object):
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


xmlr.reflect(Inertia, tag='inertia',
             params=[xmlr.Attribute(key, float) for key in Inertia.KEYS])


class Inertial(xmlr.Object):
    def __init__(self, mass=0.0, inertia=None, origin=None):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin


xmlr.reflect(Inertial, tag='inertial', params=[
    origin_element,
    xmlr.Element('mass', 'element_value'),
    xmlr.Element('inertia', Inertia, False)
])


# FIXME: we are missing the reference position here.
class JointCalibration(xmlr.Object):
    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling


xmlr.reflect(JointCalibration, tag='calibration', params=[
    xmlr.Attribute('rising', float, False, 0),
    xmlr.Attribute('falling', float, False, 0)
])


class JointLimit(xmlr.Object):
    def __init__(self, effort=None, velocity=None, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper


xmlr.reflect(JointLimit, tag='limit', params=[
    xmlr.Attribute('effort', float),
    xmlr.Attribute('lower', float, False, 0),
    xmlr.Attribute('upper', float, False, 0),
    xmlr.Attribute('velocity', float)
])

# FIXME: we are missing __str__ here.


class JointMimic(xmlr.Object):
    def __init__(self, joint_name=None, multiplier=None, offset=None):
        self.joint = joint_name
        self.multiplier = multiplier
        self.offset = offset


xmlr.reflect(JointMimic, tag='mimic', params=[
    xmlr.Attribute('joint', str),
    xmlr.Attribute('multiplier', float, False),
    xmlr.Attribute('offset', float, False)
])


class SafetyController(xmlr.Object):
    def __init__(self, velocity=None, position=None, lower=None, upper=None):
        self.k_velocity = velocity
        self.k_position = position
        self.soft_lower_limit = lower
        self.soft_upper_limit = upper


xmlr.reflect(SafetyController, tag='safety_controller', params=[
    xmlr.Attribute('k_velocity', float),
    xmlr.Attribute('k_position', float, False, 0),
    xmlr.Attribute('soft_lower_limit', float, False, 0),
    xmlr.Attribute('soft_upper_limit', float, False, 0)
])


class Joint(xmlr.Object):
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

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def joint_type(self): return self.type

    @joint_type.setter
    def joint_type(self, value): self.type = value


xmlr.reflect(Joint, tag='joint', params=[
    name_attribute,
    xmlr.Attribute('type', str),
    origin_element,
    xmlr.Element('axis', 'element_xyz', False),
    xmlr.Element('parent', 'element_link'),
    xmlr.Element('child', 'element_link'),
    xmlr.Element('limit', JointLimit, False),
    xmlr.Element('dynamics', JointDynamics, False),
    xmlr.Element('safety_controller', SafetyController, False),
    xmlr.Element('calibration', JointCalibration, False),
    xmlr.Element('mimic', JointMimic, False),
])


class Link(xmlr.Object):
    def __init__(self, name=None, visual=None, inertial=None, collision=None,
                 origin=None):
        self.aggregate_init()
        self.name = name
        self.visuals = []
        if visual:
            self.visual = visual
        self.inertial = inertial
        self.collisions = []
        if collision:
            self.collision = collision
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
        if visual:
            self.add_aggregate('visual', visual)

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
        if collision:
            self.add_aggregate('collision', collision)

    # Properties for backwards compatibility
    visual = property(__get_visual, __set_visual)
    collision = property(__get_collision, __set_collision)


xmlr.reflect(Link, tag='link', params=[
    name_attribute,
    origin_element,
    xmlr.AggregateElement('visual', Visual),
    xmlr.AggregateElement('collision', Collision),
    xmlr.Element('inertial', Inertial, False),
])


class PR2Transmission(xmlr.Object):
    def __init__(self, name=None, joint=None, actuator=None, type=None,
                 mechanicalReduction=1):
        self.name = name
        self.type = type
        self.joint = joint
        self.actuator = actuator
        self.mechanicalReduction = mechanicalReduction


xmlr.reflect(PR2Transmission, tag='pr2_transmission', params=[
    name_attribute,
    xmlr.Attribute('type', str),
    xmlr.Element('joint', 'element_name'),
    xmlr.Element('actuator', 'element_name'),
    xmlr.Element('mechanicalReduction', float)
])


class Actuator(xmlr.Object):
    def __init__(self, name=None, mechanicalReduction=1):
        self.name = name
        self.mechanicalReduction = None


xmlr.reflect(Actuator, tag='actuator', params=[
    name_attribute,
    xmlr.Element('mechanicalReduction', float, required=False)
])


class TransmissionJoint(xmlr.Object):
    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.hardwareInterfaces = []

    def check_valid(self):
        assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"


xmlr.reflect(TransmissionJoint, tag='joint', params=[
    name_attribute,
    xmlr.AggregateElement('hardwareInterface', str),
])


class Transmission(xmlr.Object):
    """ New format: http://wiki.ros.org/urdf/XML/Transmission """

    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.joints = []
        self.actuators = []

    def check_valid(self):
        assert len(self.joints) > 0, "no joint defined"
        assert len(self.actuators) > 0, "no actuator defined"


xmlr.reflect(Transmission, tag='new_transmission', params=[
    name_attribute,
    xmlr.Element('type', str),
    xmlr.AggregateElement('joint', TransmissionJoint),
    xmlr.AggregateElement('actuator', Actuator)
])

xmlr.add_type('transmission',
              xmlr.DuckTypedFactory('transmission',
                                    [Transmission, PR2Transmission]))


# add the vector2 type
get_type('vector2')


class Tactile(xmlr.Object):
    def __init__(self, channel=None):
        self.channel = channel


class TactileArrayElement(xmlr.Object):
    ROWMAJOR = "row-major"
    COLUMNMAJOR = "column-major"

    def __init__(self, rows=None, cols=None, order=None, size=None, spacing=None, offset=None):
        self.rows = rows
        self.cols = cols
        self.order = order
        self.size = size
        self.offset = offset
        self.spacing = spacing

    def check_valid(self):
        if self.spacing is None:
            self.spacing = self.size
        # fix int here, it appears to not be possible in init because params are not set at init
        self.rows = int(self.rows) if self.rows is not None else None
        self.cols = int(self.cols) if self.cols is not None else None
        assert self.order in [self.ROWMAJOR, self.COLUMNMAJOR], ("order should be " +
                                                                 str(self.ROWMAJOR) + " or " + str(self.COLUMNMAJOR))


# xmlr.add_type('geometric', TactileArrayElement())

xmlr.reflect(TactileArrayElement, tag='tactile_array_element', params=[
    xmlr.Attribute('rows', float),
    xmlr.Attribute('cols', float),
    xmlr.Attribute('order', str, False, default="row-major"),
    xmlr.Attribute('size', 'vector2'),
    xmlr.Attribute('spacing', 'vector2', False),
    xmlr.Attribute('offset', 'vector2', False, default=[0, 0])
])


class TactileArray(Tactile):
    def __init__(self, channel=None, array=None):
        Tactile.__init__(self, channel=channel)
        self.array = array
        self.taxel = None

    # the test is very important in DuckTyping, if no fail, won't test the second type
    def check_valid(self):
        assert self.array is not None


xmlr.reflect(TactileArray, tag='tactile', params=[
    xmlr.Attribute('channel', str),
    xmlr.Element('array', TactileArrayElement),
    xmlr.Element('taxel', xmlr.RawType(), False)
])


class TactileTaxelElement(xmlr.Object):
    def __init__(self, idx=None, xyz=None, rpy=None, geometry=None):
        self.idx = int(idx) if idx is not None else None
        self.xyz = xyz
        self.rpy = rpy
        self.geometry = geometry


xmlr.reflect(TactileTaxelElement, tag='array', params=[
    xmlr.Attribute('idx', float),
    xmlr.Attribute('xyz', 'vector3', False, default=[0, 0, 0]),
    xmlr.Attribute('rpy', 'vector3', False, default=[0, 0, 0]),
    xmlr.Element('geometry', 'geometric'),
])


class TactileTaxels(Tactile):
    def __init__(self, channel=None, taxel=None):
        Tactile.__init__(self, channel=channel)
        self.aggregate_init()
        self.taxels = []
        if taxel:
            self.taxel = taxel
        self.array = None

    def __get_taxel(self):
        """Return the first taxel or None."""
        if self.taxels:
            return self.taxels[0]

    def __set_taxel(self, taxel):
        """Set the first taxel."""
        if self.taxels:
            self.taxels[0] = taxel
        else:
            self.taxels.append(taxel)
        if taxel:
            self.add_aggregate('taxel', taxel)

    # the test is very important in DuckTyping, if no fail, won't test the second type
    def check_valid(self):
        assert self.taxel is not None

    # Properties setter getter
    taxel = property(__get_taxel, __set_taxel)


xmlr.reflect(TactileTaxels, tag='tactile', params=[
    xmlr.Attribute('channel', str),
    xmlr.AggregateElement('taxel', TactileTaxelElement),
    xmlr.Element('array', xmlr.RawType(), False)
])


xmlr.add_type('tactile',
              xmlr.DuckTypedFactory('tactile',
                                    [TactileTaxels, TactileArray]))


class Sensor(xmlr.Object):
    """ UBI Sensor Base """

    def __init__(self, name=None, group=None, update_rate=None, parent=None, origin=None):
        self.name = name
        self.group = group
        self.update_rate = update_rate
        self.parent = parent
        self.origin = origin
        self.sensor = None


class SensorTactile(Sensor):
    """ UBI Sensor format """

    def __init__(self, name=None, group=None, update_rate=None, parent=None, origin=None, tactile=None):
        Sensor.__init__(self, name, group, update_rate, parent, origin)
        # one cannot just pass self.tactile to a sensor initialization
        # reflect needs the parameter to be part of the object so
        # member tactile is nedded
        self.tactile = tactile
        self.sensor = self.tactile

    def check_valid(self):
        # this test cannot be generalized to test sensor in the base class
        # because the check occurs before the parent element is filled
        assert self.tactile is not None, "no sensor defined"


xmlr.reflect(SensorTactile, tag='sensor_tactile', params=[
    name_attribute,
    xmlr.Attribute('group', str, False, default=""),
    xmlr.Attribute('update_rate', float),
    xmlr.Element('parent', 'element_link', False),
    origin_element,
    xmlr.Element('tactile', 'tactile')
])


class RayElement(xmlr.Object):
    def __init__(self, samples=None, resolution=None, min_angle=None, max_angle=None):
        self.samples = int(samples) if samples is not None else None
        self.resolution = int(resolution) if resolution is not None else None
        self.min_angle = min_angle
        self.max_angle = max_angle

    def check_valid(self):
        assert self.samples is not None


xmlr.reflect(RayElement, tag='ray_element', params=[
    xmlr.Attribute('samples', float),
    xmlr.Attribute('resolution', float),
    xmlr.Attribute('min_angle', float),
    xmlr.Attribute('max_angle', float)
])


class Ray(xmlr.Object):
    def __init__(self, horizontal=None, vertical=None):
        self.horizontal = horizontal
        self.vertical = vertical

    def check_valid(self):
        assert self.horizontal is not None and self.vertical is not None


xmlr.reflect(Ray, tag='ray', params=[
    xmlr.Element('horizontal', RayElement),
    xmlr.Element('vertical', RayElement)
])


class SensorRay(Sensor):
    """ UBI Sensor format """

    def __init__(self, name=None, group=None, update_rate=None, parent=None, origin=None, ray=None):
        Sensor.__init__(self, name, group, update_rate, parent, origin)
        # one cannot just pass self.ray to a sensor initialization
        # reflect needs the parameter to be part of the object so
        # member ray is nedded
        self.ray = ray
        self.sensor = self.ray

    def check_valid(self):
        # this test cannot be generalized to test sensor in the base class
        # because the check occurs before the parent element is filled
        assert self.ray is not None, "no sensor defined"


xmlr.reflect(SensorRay, tag='sensor_ray', params=[
    name_attribute,
    xmlr.Attribute('group', str, False, default=""),
    xmlr.Attribute('update_rate', float),
    xmlr.Element('parent', 'element_link', False),
    origin_element,
    xmlr.Element('ray', Ray)
])


class CameraImage(xmlr.Object):
    def __init__(self, width=None, height=None, format="R8G8B8", hfov=None, near=None, far=None):
        self.width = int(width) if width is not None else None
        self.height = int(height) if height is not None else None
        # format is optional: defaults to R8G8B8), but can be
        # (L8|R8G8B8|B8G8R8|BAYER_RGGB8|BAYER_BGGR8|BAYER_GBRG8|BAYER_GRBG8)
        self.format = format
        self.hfov = hfov
        self.near = near
        self.far = far

    def check_valid(self):
        assert self.width is not None


xmlr.reflect(CameraImage, tag='image', params=[
    xmlr.Attribute('width', float),
    xmlr.Attribute('height', float),
    xmlr.Attribute('format', str, False),
    xmlr.Attribute('hfov', float),
    xmlr.Attribute('near', float),
    xmlr.Attribute('far', float)
])


class Camera(xmlr.Object):
    def __init__(self, image=None):
        self.image = image

    def check_valid(self):
        assert self.image is not None


xmlr.reflect(Camera, tag='camera', params=[
    xmlr.Element('image', CameraImage, False)
])


class SensorCamera(Sensor):
    """ UBI Sensor format """

    def __init__(self, name=None, group=None, update_rate=None, parent=None, origin=None, camera=None):
        Sensor.__init__(self, name, group, update_rate, parent, origin)
        # one cannot just pass self.camera to a sensor initialization
        # reflect needs the parameter to be part of the object so
        # member camera is nedded
        self.camera = camera
        self.sensor = self.camera

    def check_valid(self):
        # this test cannot be generalized to test sensor in the base class
        # because the check occurs before the parent element is filled
        assert self.camera is not None, "no sensor defined"


xmlr.reflect(SensorCamera, tag='sensor_camera', params=[
    name_attribute,
    xmlr.Attribute('group', str, False, default=""),
    xmlr.Attribute('update_rate', float),
    xmlr.Element('parent', 'element_link', False),
    origin_element,
    xmlr.Element('camera', Camera)
])


class UnknownType(xmlr.ValueType):
    name = "unknown"

    def from_xml(self, node, path):
        self.node = node
        return self

    def write_xml(self, node):
        # pretty printing
        children = xml_children(self.node)
        list(map(node.append, children))
        # Copy attributes
        for (attrib_key, attrib_value) in self.node.attrib.items():
            node.set(attrib_key, attrib_value)


xmlr.add_type('sensor',
              xmlr.DuckTypedFactory('sensor',
                                    [SensorCamera, SensorRay, SensorTactile, UnknownType()]))


class Robot(xmlr.Object):
    SUPPORTED_VERSIONS = ["1.0"]

    def __init__(self, name=None, version="1.0"):
        self.aggregate_init()

        self.name = name
        if version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))

        self.version = version
        self.joints = []
        self.links = []
        self.sensors = []
        self.materials = []
        self.gazebos = []
        self.transmissions = []

        self.joint_map = {}
        self.link_map = {}
        self.sensor_map = {}

        self.parent_map = {}
        self.child_map = {}

    def add_aggregate(self, typeName, elem):
        xmlr.Object.add_aggregate(self, typeName, elem)

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
        elif typeName == 'sensor':
            sensor = elem
            if not isinstance(sensor, UnknownType):
                self.sensor_map[sensor.name] = sensor

    def add_link(self, link):
        self.add_aggregate('link', link)

    def add_joint(self, joint):
        self.add_aggregate('joint', joint)

    def add_sensor(self, sensor):
        self.add_aggregate('sensor', sensor)

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

    def post_read_xml(self):
        if self.version is None:
            self.version = "1.0"

        split = self.version.split(".")
        if len(split) != 2:
            raise ValueError("The version attribute should be in the form 'x.y'")

        if split[0] == '' or split[1] == '':
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if self.version not in self.SUPPORTED_VERSIONS:
            raise ValueError("Invalid version; only %s is supported" % (','.join(self.SUPPORTED_VERSIONS)))

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


xmlr.reflect(Robot, tag='robot', params=[
    xmlr.Attribute('name', str),
    xmlr.Attribute('version', str, False),
    xmlr.AggregateElement('link', Link),
    xmlr.AggregateElement('joint', Joint),
    xmlr.AggregateElement('sensor', 'sensor'),
    xmlr.AggregateElement('gazebo', xmlr.RawType()),
    xmlr.AggregateElement('transmission', 'transmission'),
    xmlr.AggregateElement('material', Material)
])

# Make an alias
URDF = Robot

xmlr.end_namespace()
