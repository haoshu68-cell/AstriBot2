# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_navigation_msgs:msg/EnvelopeApplyStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EnvelopeApplyStatus(type):
    """Metaclass of message 'EnvelopeApplyStatus'."""

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
            module = import_type_support('astribot_navigation_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'astribot_navigation_msgs.msg.EnvelopeApplyStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__envelope_apply_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__envelope_apply_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__envelope_apply_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__envelope_apply_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__envelope_apply_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EnvelopeApplyStatus(metaclass=Metaclass_EnvelopeApplyStatus):
    """Message class 'EnvelopeApplyStatus'."""

    __slots__ = [
        '_header',
        '_coordinator_session_id',
        '_consumer_id',
        '_envelope_epoch',
        '_installed_geometry_hash',
        '_applied',
        '_reason',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'coordinator_session_id': 'string',
        'consumer_id': 'string',
        'envelope_epoch': 'uint64',
        'installed_geometry_hash': 'string',
        'applied': 'boolean',
        'reason': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.coordinator_session_id = kwargs.get('coordinator_session_id', str())
        self.consumer_id = kwargs.get('consumer_id', str())
        self.envelope_epoch = kwargs.get('envelope_epoch', int())
        self.installed_geometry_hash = kwargs.get('installed_geometry_hash', str())
        self.applied = kwargs.get('applied', bool())
        self.reason = kwargs.get('reason', str())

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
        if self.coordinator_session_id != other.coordinator_session_id:
            return False
        if self.consumer_id != other.consumer_id:
            return False
        if self.envelope_epoch != other.envelope_epoch:
            return False
        if self.installed_geometry_hash != other.installed_geometry_hash:
            return False
        if self.applied != other.applied:
            return False
        if self.reason != other.reason:
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
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def coordinator_session_id(self):
        """Message field 'coordinator_session_id'."""
        return self._coordinator_session_id

    @coordinator_session_id.setter
    def coordinator_session_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'coordinator_session_id' field must be of type 'str'"
        self._coordinator_session_id = value

    @builtins.property
    def consumer_id(self):
        """Message field 'consumer_id'."""
        return self._consumer_id

    @consumer_id.setter
    def consumer_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'consumer_id' field must be of type 'str'"
        self._consumer_id = value

    @builtins.property
    def envelope_epoch(self):
        """Message field 'envelope_epoch'."""
        return self._envelope_epoch

    @envelope_epoch.setter
    def envelope_epoch(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'envelope_epoch' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'envelope_epoch' field must be an unsigned integer in [0, 18446744073709551615]"
        self._envelope_epoch = value

    @builtins.property
    def installed_geometry_hash(self):
        """Message field 'installed_geometry_hash'."""
        return self._installed_geometry_hash

    @installed_geometry_hash.setter
    def installed_geometry_hash(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'installed_geometry_hash' field must be of type 'str'"
        self._installed_geometry_hash = value

    @builtins.property
    def applied(self):
        """Message field 'applied'."""
        return self._applied

    @applied.setter
    def applied(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'applied' field must be of type 'bool'"
        self._applied = value

    @builtins.property
    def reason(self):
        """Message field 'reason'."""
        return self._reason

    @reason.setter
    def reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'reason' field must be of type 'str'"
        self._reason = value
