from urdf_parser_py import _now_private_property
import urdf_parser_py._xml_reflection as _xmlr

_xmlr.start_namespace('sdf')


class Pose(_xmlr.Object):
    def __init__(self, vec=None, extra=None):
        self.xyz = None
        self.rpy = None
        if vec is not None:
            assert isinstance(vec, list)
            count = len(vec)
            if len == 3:
                xyz = vec
            else:
                self.from_vec(vec)
        elif extra is not None:
            assert xyz is None, "Cannot specify 6-length vector and 3-length vector"  # noqa
            assert len(extra) == 3, "Invalid length"
            self.rpy = extra

    def from_vec(self, vec):
        assert len(vec) == 6, "Invalid length"
        self.xyz = vec[:3]
        self.rpy = vec[3:6]

    def as_vec(self):
        xyz = self.xyz if self.xyz else [0, 0, 0]
        rpy = self.rpy if self.rpy else [0, 0, 0]
        return xyz + rpy

    def _read_xml(self, node):
        # Better way to do this? Define type?
        vec = _xmlr.get_type('vector6').read_xml_value(node)
        self.from_vec(vec)

    def _write_xml(self, node):
        vec = self.as_vec()
        _xmlr.get_type('vector6').write_xml_value(node, vec)

    def _check_valid(self):
        assert self.xyz is not None or self.rpy is not None


_name_attribute = _xmlr.Attribute('name', str)
_pose_element = _xmlr.Element('pose', Pose, required=False)


class Entity(_xmlr.Object):
    def __init__(self, name=None, pose=None):
        self.name = name
        self.pose = pose


_xmlr.reflect(Entity, params=[
    _name_attribute,
    _pose_element
])


class Inertia(_xmlr.Object):
    _KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

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
             params=[_xmlr.Element(key, float) for key in Inertia._KEYS])


class Inertial(_xmlr.Object):
    def __init__(self, mass=0.0, inertia=None, pose=None):
        self.mass = mass
        self.inertia = inertia
        self.pose = pose


_xmlr.reflect(Inertial, tag='inertial', params=[
    _xmlr.Element('mass', float),
    _xmlr.Element('inertia', Inertia),
    _pose_element
])


class Link(Entity):
    def __init__(self, name=None, pose=None, inertial=None, kinematic=False):
        Entity.__init__(self, name, pose)
        self.inertial = inertial
        self.kinematic = kinematic


_xmlr.reflect(Link, tag='link', parent_cls=Entity, params=[
    _xmlr.Element('inertial', Inertial),
    _xmlr.Attribute('kinematic', bool, False),
    _xmlr.AggregateElement('visual', Visual, var='visuals'),
    _xmlr.AggregateElement('collision', Collision, var='collisions')
])


class Joint(Entity):
    pass


_xmlr.reflect(Joint, tag='joint', parent_cls=Entity, params=[])


class Model(Entity):
    def __init__(self, name=None, pose=None):
        Entity.__init__(self, name, pose)
        self.links = []
        self.joints = []
        self.plugins = []


_xmlr.reflect(Model, parent_cls=Entity, params=[
    _xmlr.AggregateElement('link', Link, var='links'),
    _xmlr.AggregateElement('joint', Joint, var='joints'),
    _xmlr.AggregateElement('plugin', Plugin, var='plugins')
])

_xmlr.end_namespace('sdf')
