// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from astribot_msgs:msg/AstribotHeader.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "astribot_msgs/msg/detail/astribot_header__struct.h"
#include "astribot_msgs/msg/detail/astribot_header__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool astribot_msgs__msg__astribot_header__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("astribot_msgs.msg._astribot_header.AstribotHeader", full_classname_dest, 49) == 0);
  }
  astribot_msgs__msg__AstribotHeader * ros_message = _ros_message;
  {  // seq
    PyObject * field = PyObject_GetAttrString(_pymsg, "seq");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->seq = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // time_meas
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_meas");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_meas = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // time_pub
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_pub");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_pub = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * astribot_msgs__msg__astribot_header__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AstribotHeader */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("astribot_msgs.msg._astribot_header");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AstribotHeader");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  astribot_msgs__msg__AstribotHeader * ros_message = (astribot_msgs__msg__AstribotHeader *)raw_ros_message;
  {  // seq
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->seq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "seq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_meas
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->time_meas);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_meas", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_pub
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->time_pub);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_pub", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
