"""
Python implementation of the URDF parser.
"""

import functools
import warnings


class _DeprecatedDescriptor(object):
    def __init__(self, attr):
        self._attr = attr

    def _warn(self):
        raise NotImplemented

    def __get__(self, obj, objtype):
        self._warn()
        if obj is None:
            return getattr(objtype, self._attr)
        else:
            return getattr(obj, self._attr)

    def __set__(self, obj, value):
        self._warn()
        setattr(obj, self._attr, value)

    def __del__(self, obj):
        self._warn()
        delattr(obj, self._attr)


class _NowPrivateDescriptor(_DeprecatedDescriptor):
    # Implements the descriptor interface to warn about deprecated access.
    def __init__(self, private):
        _DeprecatedDescriptor.__init__(self, private)
        self._private = private
        self._old_public = self._private.lstrip('_')
        self.__doc__ = "Deprecated propery '{}'".format(self._old_public)

    def _warn(self):
        warnings.warn(
            "'{}' is deprecated, and will be removed in future releases."
                .format(self._old_public),
            category=DeprecationWarning, stacklevel=1)


def _now_private_property(private):
    # Indicates that a property (or method) is now private.
    return _NowPrivateDescriptor(private)


class _RenamedDescriptor(_DeprecatedDescriptor):
    # Implements the descriptor interface to warn about deprecated access.
    def __init__(self, old, new):
        _DeprecatedDescriptor.__init__(self, new)
        self._old = old
        self._new = new
        self.__doc__ = "Deprecated propery '{}'".format(self._old)

    def _warn(self):
        warnings.warn(
            "'{}' is deprecated, please use '{}' instead.".format(
                self._old, self._new),
            category=DeprecationWarning, stacklevel=1)


def _renamed_property(old, new):
    return _RenamedDescriptor(old, new)
