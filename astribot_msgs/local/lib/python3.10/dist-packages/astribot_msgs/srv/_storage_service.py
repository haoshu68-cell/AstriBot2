# generated from rosidl_generator_py/resource/_idl.py.em
# with input from astribot_msgs:srv/StorageService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StorageService_Request(type):
    """Metaclass of message 'StorageService_Request'."""

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
                'astribot_msgs.srv.StorageService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__storage_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__storage_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__storage_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__storage_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__storage_service__request

            from astribot_msgs.msg import StorageRequest
            if StorageRequest.__class__._TYPE_SUPPORT is None:
                StorageRequest.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StorageService_Request(metaclass=Metaclass_StorageService_Request):
    """Message class 'StorageService_Request'."""

    __slots__ = [
        '_request',
    ]

    _fields_and_field_types = {
        'request': 'astribot_msgs/StorageRequest',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['astribot_msgs', 'msg'], 'StorageRequest'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from astribot_msgs.msg import StorageRequest
        self.request = kwargs.get('request', StorageRequest())

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
        if self.request != other.request:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if __debug__:
            from astribot_msgs.msg import StorageRequest
            assert \
                isinstance(value, StorageRequest), \
                "The 'request' field must be a sub message of type 'StorageRequest'"
        self._request = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_StorageService_Response(type):
    """Metaclass of message 'StorageService_Response'."""

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
                'astribot_msgs.srv.StorageService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__storage_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__storage_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__storage_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__storage_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__storage_service__response

            from astribot_msgs.msg import StorageReponse
            if StorageReponse.__class__._TYPE_SUPPORT is None:
                StorageReponse.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StorageService_Response(metaclass=Metaclass_StorageService_Response):
    """Message class 'StorageService_Response'."""

    __slots__ = [
        '_response',
    ]

    _fields_and_field_types = {
        'response': 'astribot_msgs/StorageReponse',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['astribot_msgs', 'msg'], 'StorageReponse'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from astribot_msgs.msg import StorageReponse
        self.response = kwargs.get('response', StorageReponse())

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
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if __debug__:
            from astribot_msgs.msg import StorageReponse
            assert \
                isinstance(value, StorageReponse), \
                "The 'response' field must be a sub message of type 'StorageReponse'"
        self._response = value


class Metaclass_StorageService(type):
    """Metaclass of service 'StorageService'."""

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
                'astribot_msgs.srv.StorageService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__storage_service

            from astribot_msgs.srv import _storage_service
            if _storage_service.Metaclass_StorageService_Request._TYPE_SUPPORT is None:
                _storage_service.Metaclass_StorageService_Request.__import_type_support__()
            if _storage_service.Metaclass_StorageService_Response._TYPE_SUPPORT is None:
                _storage_service.Metaclass_StorageService_Response.__import_type_support__()


class StorageService(metaclass=Metaclass_StorageService):
    from astribot_msgs.srv._storage_service import StorageService_Request as Request
    from astribot_msgs.srv._storage_service import StorageService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
