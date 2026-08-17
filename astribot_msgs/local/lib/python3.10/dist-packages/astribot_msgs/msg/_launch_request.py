# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/LaunchRequest.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LaunchRequest(type):
    """Metaclass of message 'LaunchRequest'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ENUM_UNKNOWN': 0,
        'ENUM_ASTRIBOT_CAMERA': 1,
        'ENUM_ASTRIBOT_JOY': 2,
        'ENUM_ASTRIBOT_VR': 3,
        'ENUM_ASTRIBOT_MIC': 4,
        'ENUM_ASTRIBOT_LIDAR': 5,
        'ENUM_ASTRIBOT_TELEOP': 6,
        'ENUM_DEACTIVE': 1,
        'ENUM_ACTIVE': 2,
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
                'astribot_msgs.msg.LaunchRequest')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__launch_request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__launch_request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__launch_request
            cls._TYPE_SUPPORT = module.type_support_msg__msg__launch_request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__launch_request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ENUM_UNKNOWN': cls.__constants['ENUM_UNKNOWN'],
            'ENUM_ASTRIBOT_CAMERA': cls.__constants['ENUM_ASTRIBOT_CAMERA'],
            'ENUM_ASTRIBOT_JOY': cls.__constants['ENUM_ASTRIBOT_JOY'],
            'ENUM_ASTRIBOT_VR': cls.__constants['ENUM_ASTRIBOT_VR'],
            'ENUM_ASTRIBOT_MIC': cls.__constants['ENUM_ASTRIBOT_MIC'],
            'ENUM_ASTRIBOT_LIDAR': cls.__constants['ENUM_ASTRIBOT_LIDAR'],
            'ENUM_ASTRIBOT_TELEOP': cls.__constants['ENUM_ASTRIBOT_TELEOP'],
            'ENUM_DEACTIVE': cls.__constants['ENUM_DEACTIVE'],
            'ENUM_ACTIVE': cls.__constants['ENUM_ACTIVE'],
        }

    @property
    def ENUM_UNKNOWN(self):
        """Message constant 'ENUM_UNKNOWN'."""
        return Metaclass_LaunchRequest.__constants['ENUM_UNKNOWN']

    @property
    def ENUM_ASTRIBOT_CAMERA(self):
        """Message constant 'ENUM_ASTRIBOT_CAMERA'."""
        return Metaclass_LaunchRequest.__constants['ENUM_ASTRIBOT_CAMERA']

    @property
    def ENUM_ASTRIBOT_JOY(self):
        """Message constant 'ENUM_ASTRIBOT_JOY'."""
        return Metaclass_LaunchRequest.__constants['ENUM_ASTRIBOT_JOY']

    @property
    def ENUM_ASTRIBOT_VR(self):
        """Message constant 'ENUM_ASTRIBOT_VR'."""
        return Metaclass_LaunchRequest.__constants['ENUM_ASTRIBOT_VR']

    @property
    def ENUM_ASTRIBOT_MIC(self):
        """Message constant 'ENUM_ASTRIBOT_MIC'."""
        return Metaclass_LaunchRequest.__constants['ENUM_ASTRIBOT_MIC']

    @property
    def ENUM_ASTRIBOT_LIDAR(self):
        """Message constant 'ENUM_ASTRIBOT_LIDAR'."""
        return Metaclass_LaunchRequest.__constants['ENUM_ASTRIBOT_LIDAR']

    @property
    def ENUM_ASTRIBOT_TELEOP(self):
        """Message constant 'ENUM_ASTRIBOT_TELEOP'."""
        return Metaclass_LaunchRequest.__constants['ENUM_ASTRIBOT_TELEOP']

    @property
    def ENUM_DEACTIVE(self):
        """Message constant 'ENUM_DEACTIVE'."""
        return Metaclass_LaunchRequest.__constants['ENUM_DEACTIVE']

    @property
    def ENUM_ACTIVE(self):
        """Message constant 'ENUM_ACTIVE'."""
        return Metaclass_LaunchRequest.__constants['ENUM_ACTIVE']


class LaunchRequest(metaclass=Metaclass_LaunchRequest):
    """
    Message class 'LaunchRequest'.

    Constants:
      ENUM_UNKNOWN
      ENUM_ASTRIBOT_CAMERA
      ENUM_ASTRIBOT_JOY
      ENUM_ASTRIBOT_VR
      ENUM_ASTRIBOT_MIC
      ENUM_ASTRIBOT_LIDAR
      ENUM_ASTRIBOT_TELEOP
      ENUM_DEACTIVE
      ENUM_ACTIVE
    """

    __slots__ = [
        '_launch_target',
        '_launch_action',
    ]

    _fields_and_field_types = {
        'launch_target': 'uint8',
        'launch_action': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.launch_target = kwargs.get('launch_target', int())
        self.launch_action = kwargs.get('launch_action', int())

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
        if self.launch_target != other.launch_target:
            return False
        if self.launch_action != other.launch_action:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def launch_target(self):
        """Message field 'launch_target'."""
        return self._launch_target

    @launch_target.setter
    def launch_target(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'launch_target' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'launch_target' field must be an unsigned integer in [0, 255]"
        self._launch_target = value

    @builtins.property
    def launch_action(self):
        """Message field 'launch_action'."""
        return self._launch_action

    @launch_action.setter
    def launch_action(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'launch_action' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'launch_action' field must be an unsigned integer in [0, 255]"
        self._launch_action = value
