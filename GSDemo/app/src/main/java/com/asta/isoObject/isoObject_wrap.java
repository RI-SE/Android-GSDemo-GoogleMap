/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.1
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.asta.isoObject;

public class isoObject_wrap {
  public static SWIGTYPE_p_std__mapT_ISO22133__Events__EventType_std__string_t getDescriptions() {
    long cPtr = isoObject_wrapJNI.descriptions_get();
    return (cPtr == 0) ? null : new SWIGTYPE_p_std__mapT_ISO22133__Events__EventType_std__string_t(cPtr, false);
  }

  public static SWIGTYPE_p_std__mapT_ISO22133__ObjectStateID_std__string_t getStateNames() {
    long cPtr = isoObject_wrapJNI.stateNames_get();
    return (cPtr == 0) ? null : new SWIGTYPE_p_std__mapT_ISO22133__ObjectStateID_std__string_t(cPtr, false);
  }

  public static boolean LessThan(Transition lhs, Transition rhs) {
    return isoObject_wrapJNI.LessThan(Transition.getCPtr(lhs), lhs, Transition.getCPtr(rhs), rhs);
  }

  public static SWIGTYPE_p_std__setT_ISO22133__Transition_t getLanguage() {
    long cPtr = isoObject_wrapJNI.language_get();
    return (cPtr == 0) ? null : new SWIGTYPE_p_std__setT_ISO22133__Transition_t(cPtr, false);
  }

  public static SWIGTYPE_p_unsigned_int new_uint32ptr() {
    long cPtr = isoObject_wrapJNI.new_uint32ptr();
    return (cPtr == 0) ? null : new SWIGTYPE_p_unsigned_int(cPtr, false);
  }

  public static SWIGTYPE_p_unsigned_int copy_uint32ptr(long value) {
    long cPtr = isoObject_wrapJNI.copy_uint32ptr(value);
    return (cPtr == 0) ? null : new SWIGTYPE_p_unsigned_int(cPtr, false);
  }

  public static void delete_uint32ptr(SWIGTYPE_p_unsigned_int obj) {
    isoObject_wrapJNI.delete_uint32ptr(SWIGTYPE_p_unsigned_int.getCPtr(obj));
  }

  public static void uint32ptr_assign(SWIGTYPE_p_unsigned_int obj, long value) {
    isoObject_wrapJNI.uint32ptr_assign(SWIGTYPE_p_unsigned_int.getCPtr(obj), value);
  }

  public static long uint32ptr_value(SWIGTYPE_p_unsigned_int obj) {
    return isoObject_wrapJNI.uint32ptr_value(SWIGTYPE_p_unsigned_int.getCPtr(obj));
  }

}
