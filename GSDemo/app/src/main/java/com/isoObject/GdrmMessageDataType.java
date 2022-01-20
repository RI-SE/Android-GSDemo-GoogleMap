/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.isoObject;

public class GdrmMessageDataType {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected GdrmMessageDataType(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(GdrmMessageDataType obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  @SuppressWarnings("deprecation")
  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        isoObjectJNI.delete_GdrmMessageDataType(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setDataCode(GeneralDataRequestDataCodeType value) {
    isoObjectJNI.GdrmMessageDataType_dataCode_set(swigCPtr, this, value.swigValue());
  }

  public GeneralDataRequestDataCodeType getDataCode() {
    return GeneralDataRequestDataCodeType.swigToEnum(isoObjectJNI.GdrmMessageDataType_dataCode_get(swigCPtr, this));
  }

  public GdrmMessageDataType() {
    this(isoObjectJNI.new_GdrmMessageDataType(), true);
  }

}
