from contextlib import contextmanager
import unittest
import warnings


class TestBase(unittest.TestCase):
    def setUp(self):
        # Ensure that all deprecations are seen as errors.
        warnings.simplefilter('error', DeprecationWarning)

    @contextmanager
    def catch_warnings(self):
        """Wraps nominal catch warnings to reset filters."""
        with warnings.catch_warnings(record=True) as w:
            # Do not error; instead, only warn once.
            warnings.simplefilter('once', DeprecationWarning)
            yield w
