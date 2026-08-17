# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/MotorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'p'
# Member 'v'
# Member 'c'
# Member 'vol'
# Member 'acc'
# Member 'can_count'
# Member 'can_count_last'
# Member 'can_error'
# Member 'j_p'
# Member 'j_v'
# Member 'j_a'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorState(type):
    """Metaclass of message 'MotorState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MAX_MOTOR_NUM': 9,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('astribot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'astribot_msgs.msg.MotorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MAX_MOTOR_NUM': cls.__constants['MAX_MOTOR_NUM'],
        }

    @property
    def MAX_MOTOR_NUM(self):
        """Message constant 'MAX_MOTOR_NUM'."""
        return Metaclass_MotorState.__constants['MAX_MOTOR_NUM']


class MotorState(metaclass=Metaclass_MotorState):
    """
    Message class 'MotorState'.

    Constants:
      MAX_MOTOR_NUM
    """

    __slots__ = [
        '_p',
        '_v',
        '_c',
        '_vol',
        '_acc',
        '_can_count',
        '_can_count_last',
        '_can_error',
        '_j_s',
        '_j_p',
        '_j_v',
        '_j_a',
    ]

    _fields_and_field_types = {
        'p': 'double[9]',
        'v': 'double[9]',
        'c': 'double[9]',
        'vol': 'double[9]',
        'acc': 'double[9]',
        'can_count': 'uint8[9]',
        'can_count_last': 'uint8[9]',
        'can_error': 'uint8[9]',
        'j_s': 'boolean[9]',
        'j_p': 'double[9]',
        'j_v': 'double[9]',
        'j_a': 'double[9]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'p' not in kwargs:
            self.p = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.p = numpy.array(kwargs.get('p'), dtype=numpy.float64)
            assert self.p.shape == (9, )
        if 'v' not in kwargs:
            self.v = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.v = numpy.array(kwargs.get('v'), dtype=numpy.float64)
            assert self.v.shape == (9, )
        if 'c' not in kwargs:
            self.c = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.c = numpy.array(kwargs.get('c'), dtype=numpy.float64)
            assert self.c.shape == (9, )
        if 'vol' not in kwargs:
            self.vol = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.vol = numpy.array(kwargs.get('vol'), dtype=numpy.float64)
            assert self.vol.shape == (9, )
        if 'acc' not in kwargs:
            self.acc = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.acc = numpy.array(kwargs.get('acc'), dtype=numpy.float64)
            assert self.acc.shape == (9, )
        if 'can_count' not in kwargs:
            self.can_count = numpy.zeros(9, dtype=numpy.uint8)
        else:
            self.can_count = numpy.array(kwargs.get('can_count'), dtype=numpy.uint8)
            assert self.can_count.shape == (9, )
        if 'can_count_last' not in kwargs:
            self.can_count_last = numpy.zeros(9, dtype=numpy.uint8)
        else:
            self.can_count_last = numpy.array(kwargs.get('can_count_last'), dtype=numpy.uint8)
            assert self.can_count_last.shape == (9, )
        if 'can_error' not in kwargs:
            self.can_error = numpy.zeros(9, dtype=numpy.uint8)
        else:
            self.can_error = numpy.array(kwargs.get('can_error'), dtype=numpy.uint8)
            assert self.can_error.shape == (9, )
        self.j_s = kwargs.get(
            'j_s',
            [bool() for x in range(9)]
        )
        if 'j_p' not in kwargs:
            self.j_p = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.j_p = numpy.array(kwargs.get('j_p'), dtype=numpy.float64)
            assert self.j_p.shape == (9, )
        if 'j_v' not in kwargs:
            self.j_v = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.j_v = numpy.array(kwargs.get('j_v'), dtype=numpy.float64)
            assert self.j_v.shape == (9, )
        if 'j_a' not in kwargs:
            self.j_a = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.j_a = numpy.array(kwargs.get('j_a'), dtype=numpy.float64)
            assert self.j_a.shape == (9, )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if all(self.p != other.p):
            return False
        if all(self.v != other.v):
            return False
        if all(self.c != other.c):
            return False
        if all(self.vol != other.vol):
            return False
        if all(self.acc != other.acc):
            return False
        if all(self.can_count != other.can_count):
            return False
        if all(self.can_count_last != other.can_count_last):
            return False
        if all(self.can_error != other.can_error):
            return False
        if self.j_s != other.j_s:
            return False
        if all(self.j_p != other.j_p):
            return False
        if all(self.j_v != other.j_v):
            return False
        if all(self.j_a != other.j_a):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def p(self):
        """Message field 'p'."""
        return self._p

    @p.setter
    def p(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'p' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'p' numpy.ndarray() must have a size of 9"
            self._p = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'p' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._p = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def v(self):
        """Message field 'v'."""
        return self._v

    @v.setter
    def v(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'v' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'v' numpy.ndarray() must have a size of 9"
            self._v = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'v' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._v = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def c(self):
        """Message field 'c'."""
        return self._c

    @c.setter
    def c(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'c' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'c' numpy.ndarray() must have a size of 9"
            self._c = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'c' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._c = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def vol(self):
        """Message field 'vol'."""
        return self._vol

    @vol.setter
    def vol(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'vol' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'vol' numpy.ndarray() must have a size of 9"
            self._vol = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'vol' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._vol = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def acc(self):
        """Message field 'acc'."""
        return self._acc

    @acc.setter
    def acc(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'acc' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'acc' numpy.ndarray() must have a size of 9"
            self._acc = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'acc' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._acc = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def can_count(self):
        """Message field 'can_count'."""
        return self._can_count

    @can_count.setter
    def can_count(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'can_count' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 9, \
                "The 'can_count' numpy.ndarray() must have a size of 9"
            self._can_count = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'can_count' field must be a set or sequence with length 9 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._can_count = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def can_count_last(self):
        """Message field 'can_count_last'."""
        return self._can_count_last

    @can_count_last.setter
    def can_count_last(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'can_count_last' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 9, \
                "The 'can_count_last' numpy.ndarray() must have a size of 9"
            self._can_count_last = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'can_count_last' field must be a set or sequence with length 9 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._can_count_last = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def can_error(self):
        """Message field 'can_error'."""
        return self._can_error

    @can_error.setter
    def can_error(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'can_error' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 9, \
                "The 'can_error' numpy.ndarray() must have a size of 9"
            self._can_error = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'can_error' field must be a set or sequence with length 9 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._can_error = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def j_s(self):
        """Message field 'j_s'."""
        return self._j_s

    @j_s.setter
    def j_s(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'j_s' field must be a set or sequence with length 9 and each value of type 'bool'"
        self._j_s = value

    @builtins.property
    def j_p(self):
        """Message field 'j_p'."""
        return self._j_p

    @j_p.setter
    def j_p(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'j_p' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'j_p' numpy.ndarray() must have a size of 9"
            self._j_p = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'j_p' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._j_p = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def j_v(self):
        """Message field 'j_v'."""
        return self._j_v

    @j_v.setter
    def j_v(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'j_v' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'j_v' numpy.ndarray() must have a size of 9"
            self._j_v = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'j_v' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._j_v = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def j_a(self):
        """Message field 'j_a'."""
        return self._j_a

    @j_a.setter
    def j_a(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'j_a' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'j_a' numpy.ndarray() must have a size of 9"
            self._j_a = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'j_a' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._j_a = numpy.array(value, dtype=numpy.float64)
