/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.1
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public class Off extends State {
  private transient long swigCPtr;

  protected Off(long cPtr, boolean cMemoryOwn) {
    super(isoObject_wrapJNI.Off_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Off obj) {
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
        isoObject_wrapJNI.delete_Off(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ObjectStateID getStateID() {
    return ObjectStateID.swigToEnum(isoObject_wrapJNI.Off_getStateID(swigCPtr, this));
  }

  public Off() {
    this(isoObject_wrapJNI.new_Off(), true);
  }

}
