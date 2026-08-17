// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from astribot_msgs:msg/EncoderCollectorState.idl
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
#include "astribot_msgs/msg/detail/encoder_collector_state__struct.h"
#include "astribot_msgs/msg/detail/encoder_collector_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool astribot_msgs__msg__encoder_collector_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[65];
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
    assert(strncmp("astribot_msgs.msg._encoder_collector_state.EncoderCollectorState", full_classname_dest, 64) == 0);
  }
  astribot_msgs__msg__EncoderCollectorState * ros_message = _ros_message;
  {  // position_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_rps
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_rps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_rps = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // acceleration_rpss
    PyObject * field = PyObject_GetAttrString(_pymsg, "acceleration_rpss");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->acceleration_rpss = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rx_sequence_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "rx_sequence_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rx_sequence_count = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * astribot_msgs__msg__encoder_collector_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EncoderCollectorState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("astribot_msgs.msg._encoder_collector_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EncoderCollectorState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  astribot_msgs__msg__EncoderCollectorState * ros_message = (astribot_msgs__msg__EncoderCollectorState *)raw_ros_message;
  {  // position_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_rps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_rps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_rps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acceleration_rpss
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->acceleration_rpss);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acceleration_rpss", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rx_sequence_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rx_sequence_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rx_sequence_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
