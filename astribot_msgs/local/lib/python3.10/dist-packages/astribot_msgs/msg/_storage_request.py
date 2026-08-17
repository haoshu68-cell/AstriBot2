# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:msg/StorageRequest.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StorageRequest(type):
    """Metaclass of message 'StorageRequest'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TYPE_RECORD_MODE': 0,
        'TYPE_START_RECORD': 1,
        'TYPE_STOP_RECORD': 2,
        'TYPE_SNAPSHOT_MODE': 3,
        'TYPE_TRIGGER_SNAPSHOT': 4,
        'TYPE_INACTIVE_MODE': 5,
        'TYPE_SAVE_DATA': 6,
        'TYPE_DELETE_DATA': 7,
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
                'astribot_msgs.msg.StorageRequest')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__storage_request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__storage_request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__storage_request
            cls._TYPE_SUPPORT = module.type_support_msg__msg__storage_request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__storage_request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TYPE_RECORD_MODE': cls.__constants['TYPE_RECORD_MODE'],
            'TYPE_START_RECORD': cls.__constants['TYPE_START_RECORD'],
            'TYPE_STOP_RECORD': cls.__constants['TYPE_STOP_RECORD'],
            'TYPE_SNAPSHOT_MODE': cls.__constants['TYPE_SNAPSHOT_MODE'],
            'TYPE_TRIGGER_SNAPSHOT': cls.__constants['TYPE_TRIGGER_SNAPSHOT'],
            'TYPE_INACTIVE_MODE': cls.__constants['TYPE_INACTIVE_MODE'],
            'TYPE_SAVE_DATA': cls.__constants['TYPE_SAVE_DATA'],
            'TYPE_DELETE_DATA': cls.__constants['TYPE_DELETE_DATA'],
        }

    @property
    def TYPE_RECORD_MODE(self):
        """Message constant 'TYPE_RECORD_MODE'."""
        return Metaclass_StorageRequest.__constants['TYPE_RECORD_MODE']

    @property
    def TYPE_START_RECORD(self):
        """Message constant 'TYPE_START_RECORD'."""
        return Metaclass_StorageRequest.__constants['TYPE_START_RECORD']

    @property
    def TYPE_STOP_RECORD(self):
        """Message constant 'TYPE_STOP_RECORD'."""
        return Metaclass_StorageRequest.__constants['TYPE_STOP_RECORD']

    @property
    def TYPE_SNAPSHOT_MODE(self):
        """Message constant 'TYPE_SNAPSHOT_MODE'."""
        return Metaclass_StorageRequest.__constants['TYPE_SNAPSHOT_MODE']

    @property
    def TYPE_TRIGGER_SNAPSHOT(self):
        """Message constant 'TYPE_TRIGGER_SNAPSHOT'."""
        return Metaclass_StorageRequest.__constants['TYPE_TRIGGER_SNAPSHOT']

    @property
    def TYPE_INACTIVE_MODE(self):
        """Message constant 'TYPE_INACTIVE_MODE'."""
        return Metaclass_StorageRequest.__constants['TYPE_INACTIVE_MODE']

    @property
    def TYPE_SAVE_DATA(self):
        """Message constant 'TYPE_SAVE_DATA'."""
        return Metaclass_StorageRequest.__constants['TYPE_SAVE_DATA']

    @property
    def TYPE_DELETE_DATA(self):
        """Message constant 'TYPE_DELETE_DATA'."""
        return Metaclass_StorageRequest.__constants['TYPE_DELETE_DATA']


class StorageRequest(metaclass=Metaclass_StorageRequest):
    """
    Message class 'StorageRequest'.

    Constants:
      TYPE_RECORD_MODE
      TYPE_START_RECORD
      TYPE_STOP_RECORD
      TYPE_SNAPSHOT_MODE
      TYPE_TRIGGER_SNAPSHOT
      TYPE_INACTIVE_MODE
      TYPE_SAVE_DATA
      TYPE_DELETE_DATA
    """

    __slots__ = [
        '_uuid',
        '_storage_type',
        '_tag_content',
        '_topic_list',
    ]

    _fields_and_field_types = {
        'uuid': 'string',
        'storage_type': 'uint8',
        'tag_content': 'string',
        'topic_list': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.uuid = kwargs.get('uuid', str())
        self.storage_type = kwargs.get('storage_type', int())
        self.tag_content = kwargs.get('tag_content', str())
        self.topic_list = kwargs.get('topic_list', str())

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
        if self.uuid != other.uuid:
            return False
        if self.storage_type != other.storage_type:
            return False
        if self.tag_content != other.tag_content:
            return False
        if self.topic_list != other.topic_list:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def uuid(self):
        """Message field 'uuid'."""
        return self._uuid

    @uuid.setter
    def uuid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'uuid' field must be of type 'str'"
        self._uuid = value

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
            assert value >= 0 and value < 256, \
                "The 'storage_type' field must be an unsigned integer in [0, 255]"
        self._storage_type = value

    @builtins.property
    def tag_content(self):
        """Message field 'tag_content'."""
        return self._tag_content

    @tag_content.setter
    def tag_content(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'tag_content' field must be of type 'str'"
        self._tag_content = value

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
