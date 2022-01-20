/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.isoObject;

public class Armed extends State {
  private transient long swigCPtr;

  protected Armed(long cPtr, boolean cMemoryOwn) {
    super(isoObjectJNI.Armed_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Armed obj) {
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
        isoObjectJNI.delete_Armed(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ObjectStateID getStateID() {
    return ObjectStateID.swigToEnum(isoObjectJNI.Armed_getStateID(swigCPtr, this));
  }

  public Armed() {
    this(isoObjectJNI.new_Armed(), true);
  }

}
