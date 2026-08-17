# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/EncoderCollectorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EncoderCollectorState(type):
    """Metaclass of message 'EncoderCollectorState'."""

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
                'astribot_msgs.msg.EncoderCollectorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__encoder_collector_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__encoder_collector_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__encoder_collector_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__encoder_collector_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__encoder_collector_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EncoderCollectorState(metaclass=Metaclass_EncoderCollectorState):
    """Message class 'EncoderCollectorState'."""

    __slots__ = [
        '_position_rad',
        '_velocity_rps',
        '_acceleration_rpss',
        '_rx_sequence_count',
    ]

    _fields_and_field_types = {
        'position_rad': 'float',
        'velocity_rps': 'float',
        'acceleration_rpss': 'float',
        'rx_sequence_count': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.position_rad = kwargs.get('position_rad', float())
        self.velocity_rps = kwargs.get('velocity_rps', float())
        self.acceleration_rpss = kwargs.get('acceleration_rpss', float())
        self.rx_sequence_count = kwargs.get('rx_sequence_count', int())

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
        if self.position_rad != other.position_rad:
            return False
        if self.velocity_rps != other.velocity_rps:
            return False
        if self.acceleration_rpss != other.acceleration_rpss:
            return False
        if self.rx_sequence_count != other.rx_sequence_count:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def position_rad(self):
        """Message field 'position_rad'."""
        return self._position_rad

    @position_rad.setter
    def position_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'position_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'position_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._position_rad = value

    @builtins.property
    def velocity_rps(self):
        """Message field 'velocity_rps'."""
        return self._velocity_rps

    @velocity_rps.setter
    def velocity_rps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_rps' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velocity_rps' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velocity_rps = value

    @builtins.property
    def acceleration_rpss(self):
        """Message field 'acceleration_rpss'."""
        return self._acceleration_rpss

    @acceleration_rpss.setter
    def acceleration_rpss(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'acceleration_rpss' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'acceleration_rpss' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._acceleration_rpss = value

    @builtins.property
    def rx_sequence_count(self):
        """Message field 'rx_sequence_count'."""
        return self._rx_sequence_count

    @rx_sequence_count.setter
    def rx_sequence_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rx_sequence_count' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rx_sequence_count' field must be an unsigned integer in [0, 255]"
        self._rx_sequence_count = value
