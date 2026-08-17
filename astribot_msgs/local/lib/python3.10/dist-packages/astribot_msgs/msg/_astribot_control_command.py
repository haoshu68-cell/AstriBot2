# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/AstribotControlCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'dofs_list'
# Member 'command_list'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AstribotControlCommand(type):
    """Metaclass of message 'AstribotControlCommand'."""

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
                'astribot_msgs.msg.AstribotControlCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__astribot_control_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__astribot_control_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__astribot_control_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__astribot_control_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__astribot_control_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AstribotControlCommand(metaclass=Metaclass_AstribotControlCommand):
    """Message class 'AstribotControlCommand'."""

    __slots__ = [
        '_name_list',
        '_dofs_list',
        '_command_list',
        '_control_way',
        '_frame',
        '_use_wbc',
        '_add_default_torso',
    ]

    _fields_and_field_types = {
        'name_list': 'sequence<string>',
        'dofs_list': 'sequence<float>',
        'command_list': 'sequence<float>',
        'control_way': 'string',
        'frame': 'string',
        'use_wbc': 'boolean',
        'add_default_torso': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.name_list = kwargs.get('name_list', [])
        self.dofs_list = array.array('f', kwargs.get('dofs_list', []))
        self.command_list = array.array('f', kwargs.get('command_list', []))
        self.control_way = kwargs.get('control_way', str())
        self.frame = kwargs.get('frame', str())
        self.use_wbc = kwargs.get('use_wbc', bool())
        self.add_default_torso = kwargs.get('add_default_torso', bool())

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
        if self.name_list != other.name_list:
            return False
        if self.dofs_list != other.dofs_list:
            return False
        if self.command_list != other.command_list:
            return False
        if self.control_way != other.control_way:
            return False
        if self.frame != other.frame:
            return False
        if self.use_wbc != other.use_wbc:
            return False
        if self.add_default_torso != other.add_default_torso:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def name_list(self):
        """Message field 'name_list'."""
        return self._name_list

    @name_list.setter
    def name_list(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'name_list' field must be a set or sequence and each value of type 'str'"
        self._name_list = value

    @builtins.property
    def dofs_list(self):
        """Message field 'dofs_list'."""
        return self._dofs_list

    @dofs_list.setter
    def dofs_list(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'dofs_list' array.array() must have the type code of 'f'"
            self._dofs_list = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'dofs_list' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._dofs_list = array.array('f', value)

    @builtins.property
    def command_list(self):
        """Message field 'command_list'."""
        return self._command_list

    @command_list.setter
    def command_list(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'command_list' array.array() must have the type code of 'f'"
            self._command_list = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'command_list' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._command_list = array.array('f', value)

    @builtins.property
    def control_way(self):
        """Message field 'control_way'."""
        return self._control_way

    @control_way.setter
    def control_way(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'control_way' field must be of type 'str'"
        self._control_way = value

    @builtins.property
    def frame(self):
        """Message field 'frame'."""
        return self._frame

    @frame.setter
    def frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'frame' field must be of type 'str'"
        self._frame = value

    @builtins.property
    def use_wbc(self):
        """Message field 'use_wbc'."""
        return self._use_wbc

    @use_wbc.setter
    def use_wbc(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_wbc' field must be of type 'bool'"
        self._use_wbc = value

    @builtins.property
    def add_default_torso(self):
        """Message field 'add_default_torso'."""
        return self._add_default_torso

    @add_default_torso.setter
    def add_default_torso(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'add_default_torso' field must be of type 'bool'"
        self._add_default_torso = value
