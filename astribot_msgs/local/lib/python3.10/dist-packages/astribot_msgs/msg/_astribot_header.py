# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/AstribotHeader.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AstribotHeader(type):
    """Metaclass of message 'AstribotHeader'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'astribot_msgs.msg.AstribotHeader')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__astribot_header
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__astribot_header
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__astribot_header
            cls._TYPE_SUPPORT = module.type_support_msg__msg__astribot_header
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__astribot_header

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AstribotHeader(metaclass=Metaclass_AstribotHeader):
    """Message class 'AstribotHeader'."""

    __slots__ = [
        '_seq',
        '_time_meas',
        '_time_pub',
    ]

    _fields_and_field_types = {
        'seq': 'uint64',
        'time_meas': 'uint64',
        'time_pub': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.seq = kwargs.get('seq', int())
        self.time_meas = kwargs.get('time_meas', int())
        self.time_pub = kwargs.get('time_pub', int())

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
        if self.seq != other.seq:
            return False
        if self.time_meas != other.time_meas:
            return False
        if self.time_pub != other.time_pub:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def seq(self):
        """Message field 'seq'."""
        return self._seq

    @seq.setter
    def seq(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'seq' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'seq' field must be an unsigned integer in [0, 18446744073709551615]"
        self._seq = value

    @builtins.property
    def time_meas(self):
        """Message field 'time_meas'."""
        return self._time_meas

    @time_meas.setter
    def time_meas(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_meas' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'time_meas' field must be an unsigned integer in [0, 18446744073709551615]"
        self._time_meas = value

    @builtins.property
    def time_pub(self):
        """Message field 'time_pub'."""
        return self._time_pub

    @time_pub.setter
    def time_pub(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_pub' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'time_pub' field must be an unsigned integer in [0, 18446744073709551615]"
        self._time_pub = value
