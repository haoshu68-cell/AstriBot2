# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/MotorCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'motor_id_list'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'kp'
# Member 'kd'
# Member 'p'
# Member 'v'
# Member 't_ff'
# Member 't_limit'
# Member 'break_relase'
# Member 'kd_slave'
# Member 'vel_slave'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorCommand(type):
    """Metaclass of message 'MotorCommand'."""

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
                'astribot_msgs.msg.MotorCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_command

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
        return Metaclass_MotorCommand.__constants['MAX_MOTOR_NUM']


class MotorCommand(metaclass=Metaclass_MotorCommand):
    """
    Message class 'MotorCommand'.

    Constants:
      MAX_MOTOR_NUM
    """

    __slots__ = [
        '_kp',
        '_kd',
        '_p',
        '_v',
        '_t_ff',
        '_t_limit',
        '_break_relase',
        '_kd_slave',
        '_vel_slave',
        '_motor_id_list',
    ]

    _fields_and_field_types = {
        'kp': 'double[9]',
        'kd': 'double[9]',
        'p': 'double[9]',
        'v': 'double[9]',
        't_ff': 'double[9]',
        't_limit': 'double[9]',
        'break_relase': 'int8[9]',
        'kd_slave': 'double[9]',
        'vel_slave': 'double[9]',
        'motor_id_list': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int8'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'kp' not in kwargs:
            self.kp = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.kp = numpy.array(kwargs.get('kp'), dtype=numpy.float64)
            assert self.kp.shape == (9, )
        if 'kd' not in kwargs:
            self.kd = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.kd = numpy.array(kwargs.get('kd'), dtype=numpy.float64)
            assert self.kd.shape == (9, )
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
        if 't_ff' not in kwargs:
            self.t_ff = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.t_ff = numpy.array(kwargs.get('t_ff'), dtype=numpy.float64)
            assert self.t_ff.shape == (9, )
        if 't_limit' not in kwargs:
            self.t_limit = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.t_limit = numpy.array(kwargs.get('t_limit'), dtype=numpy.float64)
            assert self.t_limit.shape == (9, )
        if 'break_relase' not in kwargs:
            self.break_relase = numpy.zeros(9, dtype=numpy.int8)
        else:
            self.break_relase = numpy.array(kwargs.get('break_relase'), dtype=numpy.int8)
            assert self.break_relase.shape == (9, )
        if 'kd_slave' not in kwargs:
            self.kd_slave = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.kd_slave = numpy.array(kwargs.get('kd_slave'), dtype=numpy.float64)
            assert self.kd_slave.shape == (9, )
        if 'vel_slave' not in kwargs:
            self.vel_slave = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.vel_slave = numpy.array(kwargs.get('vel_slave'), dtype=numpy.float64)
            assert self.vel_slave.shape == (9, )
        self.motor_id_list = array.array('i', kwargs.get('motor_id_list', []))

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
        if all(self.kp != other.kp):
            return False
        if all(self.kd != other.kd):
            return False
        if all(self.p != other.p):
            return False
        if all(self.v != other.v):
            return False
        if all(self.t_ff != other.t_ff):
            return False
        if all(self.t_limit != other.t_limit):
            return False
        if all(self.break_relase != other.break_relase):
            return False
        if all(self.kd_slave != other.kd_slave):
            return False
        if all(self.vel_slave != other.vel_slave):
            return False
        if self.motor_id_list != other.motor_id_list:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def kp(self):
        """Message field 'kp'."""
        return self._kp

    @kp.setter
    def kp(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'kp' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'kp' numpy.ndarray() must have a size of 9"
            self._kp = value
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
                "The 'kp' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._kp = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def kd(self):
        """Message field 'kd'."""
        return self._kd

    @kd.setter
    def kd(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'kd' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'kd' numpy.ndarray() must have a size of 9"
            self._kd = value
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
                "The 'kd' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._kd = numpy.array(value, dtype=numpy.float64)

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
    def t_ff(self):
        """Message field 't_ff'."""
        return self._t_ff

    @t_ff.setter
    def t_ff(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 't_ff' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 't_ff' numpy.ndarray() must have a size of 9"
            self._t_ff = value
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
                "The 't_ff' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._t_ff = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def t_limit(self):
        """Message field 't_limit'."""
        return self._t_limit

    @t_limit.setter
    def t_limit(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 't_limit' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 't_limit' numpy.ndarray() must have a size of 9"
            self._t_limit = value
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
                "The 't_limit' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._t_limit = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def break_relase(self):
        """Message field 'break_relase'."""
        return self._break_relase

    @break_relase.setter
    def break_relase(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int8, \
                "The 'break_relase' numpy.ndarray() must have the dtype of 'numpy.int8'"
            assert value.size == 9, \
                "The 'break_relase' numpy.ndarray() must have a size of 9"
            self._break_relase = value
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
                 all(val >= -128 and val < 128 for val in value)), \
                "The 'break_relase' field must be a set or sequence with length 9 and each value of type 'int' and each integer in [-128, 127]"
        self._break_relase = numpy.array(value, dtype=numpy.int8)

    @builtins.property
    def kd_slave(self):
        """Message field 'kd_slave'."""
        return self._kd_slave

    @kd_slave.setter
    def kd_slave(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'kd_slave' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'kd_slave' numpy.ndarray() must have a size of 9"
            self._kd_slave = value
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
                "The 'kd_slave' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._kd_slave = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def vel_slave(self):
        """Message field 'vel_slave'."""
        return self._vel_slave

    @vel_slave.setter
    def vel_slave(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'vel_slave' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'vel_slave' numpy.ndarray() must have a size of 9"
            self._vel_slave = value
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
                "The 'vel_slave' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._vel_slave = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def motor_id_list(self):
        """Message field 'motor_id_list'."""
        return self._motor_id_list

    @motor_id_list.setter
    def motor_id_list(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'motor_id_list' array.array() must have the type code of 'i'"
            self._motor_id_list = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'motor_id_list' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._motor_id_list = array.array('i', value)
