# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:action/Storage.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Storage_Goal(type):
    """Metaclass of message 'Storage_Goal'."""

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
                'astribot_msgs.action.Storage_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Storage_Goal(metaclass=Metaclass_Storage_Goal):
    """Message class 'Storage_Goal'."""

    __slots__ = [
        '_topic_list',
        '_storage_type',
        '_trigger_time',
        '_archive_path',
    ]

    _fields_and_field_types = {
        'topic_list': 'string',
        'storage_type': 'int32',
        'trigger_time': 'int64',
        'archive_path': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.topic_list = kwargs.get('topic_list', str())
        self.storage_type = kwargs.get('storage_type', int())
        self.trigger_time = kwargs.get('trigger_time', int())
        self.archive_path = kwargs.get('archive_path', str())

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
        if self.topic_list != other.topic_list:
            return False
        if self.storage_type != other.storage_type:
            return False
        if self.trigger_time != other.trigger_time:
            return False
        if self.archive_path != other.archive_path:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def topic_list(self):
        """Message field 'topic_list'."""
        return self._topic_list

    @topic_list.setter
    def topic_list(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'topic_list' field must be of type 'str'"
        self._topic_list = value

    @builtins.property
    def storage_type(self):
        """Message field 'storage_type'."""
        return self._storage_type

    @storage_type.setter
    def storage_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'storage_type' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'storage_type' field must be an integer in [-2147483648, 2147483647]"
        self._storage_type = value

    @builtins.property
    def trigger_time(self):
        """Message field 'trigger_time'."""
        return self._trigger_time

    @trigger_time.setter
    def trigger_time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trigger_time' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'trigger_time' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._trigger_time = value

    @builtins.property
    def archive_path(self):
        """Message field 'archive_path'."""
        return self._archive_path

    @archive_path.setter
    def archive_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'archive_path' field must be of type 'str'"
        self._archive_path = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Storage_Result(type):
    """Metaclass of message 'Storage_Result'."""

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
                'astribot_msgs.action.Storage_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Storage_Result(metaclass=Metaclass_Storage_Result):
    """Message class 'Storage_Result'."""

    __slots__ = [
        '_dir_path',
        '_miss_topic_list',
    ]

    _fields_and_field_types = {
        'dir_path': 'string',
        'miss_topic_list': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.dir_path = kwargs.get('dir_path', str())
        self.miss_topic_list = kwargs.get('miss_topic_list', str())

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
        if self.dir_path != other.dir_path:
            return False
        if self.miss_topic_list != other.miss_topic_list:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def dir_path(self):
        """Message field 'dir_path'."""
        return self._dir_path

    @dir_path.setter
    def dir_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'dir_path' field must be of type 'str'"
        self._dir_path = value

    @builtins.property
    def miss_topic_list(self):
        """Message field 'miss_topic_list'."""
        return self._miss_topic_list

    @miss_topic_list.setter
    def miss_topic_list(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'miss_topic_list' field must be of type 'str'"
        self._miss_topic_list = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Storage_Feedback(type):
    """Metaclass of message 'Storage_Feedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATUS_RECORDING': 3,
        'STATUS_UPDATE': 4,
        'STATUS_STOP': 5,
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
                'astribot_msgs.action.Storage_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATUS_RECORDING': cls.__constants['STATUS_RECORDING'],
            'STATUS_UPDATE': cls.__constants['STATUS_UPDATE'],
            'STATUS_STOP': cls.__constants['STATUS_STOP'],
        }

    @property
    def STATUS_RECORDING(self):
        """Message constant 'STATUS_RECORDING'."""
        return Metaclass_Storage_Feedback.__constants['STATUS_RECORDING']

    @property
    def STATUS_UPDATE(self):
        """Message constant 'STATUS_UPDATE'."""
        return Metaclass_Storage_Feedback.__constants['STATUS_UPDATE']

    @property
    def STATUS_STOP(self):
        """Message constant 'STATUS_STOP'."""
        return Metaclass_Storage_Feedback.__constants['STATUS_STOP']


class Storage_Feedback(metaclass=Metaclass_Storage_Feedback):
    """
    Message class 'Storage_Feedback'.

    Constants:
      STATUS_RECORDING
      STATUS_UPDATE
      STATUS_STOP
    """

    __slots__ = [
        '_seconds',
        '_disk_space',
        '_miss_topic_list',
        '_status',
    ]

    _fields_and_field_types = {
        'seconds': 'int32',
        'disk_space': 'int32',
        'miss_topic_list': 'string',
        'status': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.seconds = kwargs.get('seconds', int())
        self.disk_space = kwargs.get('disk_space', int())
        self.miss_topic_list = kwargs.get('miss_topic_list', str())
        self.status = kwargs.get('status', int())

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
        if self.seconds != other.seconds:
            return False
        if self.disk_space != other.disk_space:
            return False
        if self.miss_topic_list != other.miss_topic_list:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def seconds(self):
        """Message field 'seconds'."""
        return self._seconds

    @seconds.setter
    def seconds(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'seconds' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'seconds' field must be an integer in [-2147483648, 2147483647]"
        self._seconds = value

    @builtins.property
    def disk_space(self):
        """Message field 'disk_space'."""
        return self._disk_space

    @disk_space.setter
    def disk_space(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'disk_space' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'disk_space' field must be an integer in [-2147483648, 2147483647]"
        self._disk_space = value

    @builtins.property
    def miss_topic_list(self):
        """Message field 'miss_topic_list'."""
        return self._miss_topic_list

    @miss_topic_list.setter
    def miss_topic_list(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'miss_topic_list' field must be of type 'str'"
        self._miss_topic_list = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'status' field must be an unsigned integer in [0, 4294967295]"
        self._status = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Storage_SendGoal_Request(type):
    """Metaclass of message 'Storage_SendGoal_Request'."""

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
                'astribot_msgs.action.Storage_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__send_goal__request

            from astribot_msgs.action import Storage
            if Storage.Goal.__class__._TYPE_SUPPORT is None:
                Storage.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Storage_SendGoal_Request(metaclass=Metaclass_Storage_SendGoal_Request):
    """Message class 'Storage_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'astribot_msgs/Storage_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['astribot_msgs', 'action'], 'Storage_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from astribot_msgs.action._storage import Storage_Goal
        self.goal = kwargs.get('goal', Storage_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from astribot_msgs.action._storage import Storage_Goal
            assert \
                isinstance(value, Storage_Goal), \
                "The 'goal' field must be a sub message of type 'Storage_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Storage_SendGoal_Response(type):
    """Metaclass of message 'Storage_SendGoal_Response'."""

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
                'astribot_msgs.action.Storage_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Storage_SendGoal_Response(metaclass=Metaclass_Storage_SendGoal_Response):
    """Message class 'Storage_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_Storage_SendGoal(type):
    """Metaclass of service 'Storage_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('astribot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'astribot_msgs.action.Storage_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__storage__send_goal

            from astribot_msgs.action import _storage
            if _storage.Metaclass_Storage_SendGoal_Request._TYPE_SUPPORT is None:
                _storage.Metaclass_Storage_SendGoal_Request.__import_type_support__()
            if _storage.Metaclass_Storage_SendGoal_Response._TYPE_SUPPORT is None:
                _storage.Metaclass_Storage_SendGoal_Response.__import_type_support__()


class Storage_SendGoal(metaclass=Metaclass_Storage_SendGoal):
    from astribot_msgs.action._storage import Storage_SendGoal_Request as Request
    from astribot_msgs.action._storage import Storage_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Storage_GetResult_Request(type):
    """Metaclass of message 'Storage_GetResult_Request'."""

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
                'astribot_msgs.action.Storage_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Storage_GetResult_Request(metaclass=Metaclass_Storage_GetResult_Request):
    """Message class 'Storage_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Storage_GetResult_Response(type):
    """Metaclass of message 'Storage_GetResult_Response'."""

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
                'astribot_msgs.action.Storage_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__get_result__response

            from astribot_msgs.action import Storage
            if Storage.Result.__class__._TYPE_SUPPORT is None:
                Storage.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Storage_GetResult_Response(metaclass=Metaclass_Storage_GetResult_Response):
    """Message class 'Storage_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'astribot_msgs/Storage_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['astribot_msgs', 'action'], 'Storage_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from astribot_msgs.action._storage import Storage_Result
        self.result = kwargs.get('result', Storage_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from astribot_msgs.action._storage import Storage_Result
            assert \
                isinstance(value, Storage_Result), \
                "The 'result' field must be a sub message of type 'Storage_Result'"
        self._result = value


class Metaclass_Storage_GetResult(type):
    """Metaclass of service 'Storage_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('astribot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'astribot_msgs.action.Storage_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__storage__get_result

            from astribot_msgs.action import _storage
            if _storage.Metaclass_Storage_GetResult_Request._TYPE_SUPPORT is None:
                _storage.Metaclass_Storage_GetResult_Request.__import_type_support__()
            if _storage.Metaclass_Storage_GetResult_Response._TYPE_SUPPORT is None:
                _storage.Metaclass_Storage_GetResult_Response.__import_type_support__()


class Storage_GetResult(metaclass=Metaclass_Storage_GetResult):
    from astribot_msgs.action._storage import Storage_GetResult_Request as Request
    from astribot_msgs.action._storage import Storage_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Storage_FeedbackMessage(type):
    """Metaclass of message 'Storage_FeedbackMessage'."""

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
                'astribot_msgs.action.Storage_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__storage__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__storage__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__storage__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__storage__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__storage__feedback_message

            from astribot_msgs.action import Storage
            if Storage.Feedback.__class__._TYPE_SUPPORT is None:
                Storage.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Storage_FeedbackMessage(metaclass=Metaclass_Storage_FeedbackMessage):
    """Message class 'Storage_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'astribot_msgs/Storage_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['astribot_msgs', 'action'], 'Storage_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from astribot_msgs.action._storage import Storage_Feedback
        self.feedback = kwargs.get('feedback', Storage_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from astribot_msgs.action._storage import Storage_Feedback
            assert \
                isinstance(value, Storage_Feedback), \
                "The 'feedback' field must be a sub message of type 'Storage_Feedback'"
        self._feedback = value


class Metaclass_Storage(type):
    """Metaclass of action 'Storage'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('astribot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'astribot_msgs.action.Storage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__storage

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from astribot_msgs.action import _storage
            if _storage.Metaclass_Storage_SendGoal._TYPE_SUPPORT is None:
                _storage.Metaclass_Storage_SendGoal.__import_type_support__()
            if _storage.Metaclass_Storage_GetResult._TYPE_SUPPORT is None:
                _storage.Metaclass_Storage_GetResult.__import_type_support__()
            if _storage.Metaclass_Storage_FeedbackMessage._TYPE_SUPPORT is None:
                _storage.Metaclass_Storage_FeedbackMessage.__import_type_support__()


class Storage(metaclass=Metaclass_Storage):

    # The goal message defined in the action definition.
    from astribot_msgs.action._storage import Storage_Goal as Goal
    # The result message defined in the action definition.
    from astribot_msgs.action._storage import Storage_Result as Result
    # The feedback message defined in the action definition.
    from astribot_msgs.action._storage import Storage_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from astribot_msgs.action._storage import Storage_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from astribot_msgs.action._storage import Storage_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from astribot_msgs.action._storage import Storage_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
