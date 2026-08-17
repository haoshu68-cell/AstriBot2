# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/WholeBodyCtrlCmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'pose_world_to_torso_desired'
# Member 'pose_world_to_left_arm_desired'
# Member 'pose_world_to_right_arm_desired'
# Member 'pose_world_to_chassis_current'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WholeBodyCtrlCmd(type):
    """Metaclass of message 'WholeBodyCtrlCmd'."""

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
                'astribot_msgs.msg.WholeBodyCtrlCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__whole_body_ctrl_cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__whole_body_ctrl_cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__whole_body_ctrl_cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__whole_body_ctrl_cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__whole_body_ctrl_cmd

            from astribot_msgs.msg import AstribotHeader
            if AstribotHeader.__class__._TYPE_SUPPORT is None:
                AstribotHeader.__class__.__import_type_support__()

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WholeBodyCtrlCmd(metaclass=Metaclass_WholeBodyCtrlCmd):
    """Message class 'WholeBodyCtrlCmd'."""

    __slots__ = [
        '_header',
        '_left_arm_twist',
        '_right_arm_twist',
        '_torso_twist',
        '_pose_world_to_torso_desired',
        '_pose_world_to_left_arm_desired',
        '_pose_world_to_right_arm_desired',
        '_pose_world_to_chassis_current',
        '_torso_open_loop',
        '_enable_collision_avoidance',
        '_smooth_t',
        '_smooth_duration',
    ]

    _fields_and_field_types = {
        'header': 'astribot_msgs/AstribotHeader',
        'left_arm_twist': 'geometry_msgs/Twist',
        'right_arm_twist': 'geometry_msgs/Twist',
        'torso_twist': 'geometry_msgs/Twist',
        'pose_world_to_torso_desired': 'sequence<double>',
        'pose_world_to_left_arm_desired': 'sequence<double>',
        'pose_world_to_right_arm_desired': 'sequence<double>',
        'pose_world_to_chassis_current': 'sequence<double>',
        'torso_open_loop': 'boolean',
        'enable_collision_avoidance': 'boolean',
        'smooth_t': 'double',
        'smooth_duration': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['astribot_msgs', 'msg'], 'AstribotHeader'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from astribot_msgs.msg import AstribotHeader
        self.header = kwargs.get('header', AstribotHeader())
        from geometry_msgs.msg import Twist
        self.left_arm_twist = kwargs.get('left_arm_twist', Twist())
        from geometry_msgs.msg import Twist
        self.right_arm_twist = kwargs.get('right_arm_twist', Twist())
        from geometry_msgs.msg import Twist
        self.torso_twist = kwargs.get('torso_twist', Twist())
        self.pose_world_to_torso_desired = array.array('d', kwargs.get('pose_world_to_torso_desired', []))
        self.pose_world_to_left_arm_desired = array.array('d', kwargs.get('pose_world_to_left_arm_desired', []))
        self.pose_world_to_right_arm_desired = array.array('d', kwargs.get('pose_world_to_right_arm_desired', []))
        self.pose_world_to_chassis_current = array.array('d', kwargs.get('pose_world_to_chassis_current', []))
        self.torso_open_loop = kwargs.get('torso_open_loop', bool())
        self.enable_collision_avoidance = kwargs.get('enable_collision_avoidance', bool())
        self.smooth_t = kwargs.get('smooth_t', float())
        self.smooth_duration = kwargs.get('smooth_duration', float())

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
        if self.header != other.header:
            return False
        if self.left_arm_twist != other.left_arm_twist:
            return False
        if self.right_arm_twist != other.right_arm_twist:
            return False
        if self.torso_twist != other.torso_twist:
            return False
        if self.pose_world_to_torso_desired != other.pose_world_to_torso_desired:
            return False
        if self.pose_world_to_left_arm_desired != other.pose_world_to_left_arm_desired:
            return False
        if self.pose_world_to_right_arm_desired != other.pose_world_to_right_arm_desired:
            return False
        if self.pose_world_to_chassis_current != other.pose_world_to_chassis_current:
            return False
        if self.torso_open_loop != other.torso_open_loop:
            return False
        if self.enable_collision_avoidance != other.enable_collision_avoidance:
            return False
        if self.smooth_t != other.smooth_t:
            return False
        if self.smooth_duration != other.smooth_duration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from astribot_msgs.msg import AstribotHeader
            assert \
                isinstance(value, AstribotHeader), \
                "The 'header' field must be a sub message of type 'AstribotHeader'"
        self._header = value

    @builtins.property
    def left_arm_twist(self):
        """Message field 'left_arm_twist'."""
        return self._left_arm_twist

    @left_arm_twist.setter
    def left_arm_twist(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'left_arm_twist' field must be a sub message of type 'Twist'"
        self._left_arm_twist = value

    @builtins.property
    def right_arm_twist(self):
        """Message field 'right_arm_twist'."""
        return self._right_arm_twist

    @right_arm_twist.setter
    def right_arm_twist(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'right_arm_twist' field must be a sub message of type 'Twist'"
        self._right_arm_twist = value

    @builtins.property
    def torso_twist(self):
        """Message field 'torso_twist'."""
        return self._torso_twist

    @torso_twist.setter
    def torso_twist(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'torso_twist' field must be a sub message of type 'Twist'"
        self._torso_twist = value

    @builtins.property
    def pose_world_to_torso_desired(self):
        """Message field 'pose_world_to_torso_desired'."""
        return self._pose_world_to_torso_desired

    @pose_world_to_torso_desired.setter
    def pose_world_to_torso_desired(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'pose_world_to_torso_desired' array.array() must have the type code of 'd'"
            self._pose_world_to_torso_desired = value
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
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'pose_world_to_torso_desired' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._pose_world_to_torso_desired = array.array('d', value)

    @builtins.property
    def pose_world_to_left_arm_desired(self):
        """Message field 'pose_world_to_left_arm_desired'."""
        return self._pose_world_to_left_arm_desired

    @pose_world_to_left_arm_desired.setter
    def pose_world_to_left_arm_desired(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'pose_world_to_left_arm_desired' array.array() must have the type code of 'd'"
            self._pose_world_to_left_arm_desired = value
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
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'pose_world_to_left_arm_desired' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._pose_world_to_left_arm_desired = array.array('d', value)

    @builtins.property
    def pose_world_to_right_arm_desired(self):
        """Message field 'pose_world_to_right_arm_desired'."""
        return self._pose_world_to_right_arm_desired

    @pose_world_to_right_arm_desired.setter
    def pose_world_to_right_arm_desired(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'pose_world_to_right_arm_desired' array.array() must have the type code of 'd'"
            self._pose_world_to_right_arm_desired = value
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
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'pose_world_to_right_arm_desired' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._pose_world_to_right_arm_desired = array.array('d', value)

    @builtins.property
    def pose_world_to_chassis_current(self):
        """Message field 'pose_world_to_chassis_current'."""
        return self._pose_world_to_chassis_current

    @pose_world_to_chassis_current.setter
    def pose_world_to_chassis_current(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'pose_world_to_chassis_current' array.array() must have the type code of 'd'"
            self._pose_world_to_chassis_current = value
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
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'pose_world_to_chassis_current' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._pose_world_to_chassis_current = array.array('d', value)

    @builtins.property
    def torso_open_loop(self):
        """Message field 'torso_open_loop'."""
        return self._torso_open_loop

    @torso_open_loop.setter
    def torso_open_loop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'torso_open_loop' field must be of type 'bool'"
        self._torso_open_loop = value

    @builtins.property
    def enable_collision_avoidance(self):
        """Message field 'enable_collision_avoidance'."""
        return self._enable_collision_avoidance

    @enable_collision_avoidance.setter
    def enable_collision_avoidance(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'enable_collision_avoidance' field must be of type 'bool'"
        self._enable_collision_avoidance = value

    @builtins.property
    def smooth_t(self):
        """Message field 'smooth_t'."""
        return self._smooth_t

    @smooth_t.setter
    def smooth_t(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'smooth_t' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'smooth_t' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._smooth_t = value

    @builtins.property
    def smooth_duration(self):
        """Message field 'smooth_duration'."""
        return self._smooth_duration

    @smooth_duration.setter
    def smooth_duration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'smooth_duration' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'smooth_duration' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._smooth_duration = value
