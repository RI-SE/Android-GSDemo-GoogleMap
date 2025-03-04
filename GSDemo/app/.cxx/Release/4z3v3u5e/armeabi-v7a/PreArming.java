/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public class PreArming extends State {
  private transient long swigCPtr;

  protected PreArming(long cPtr, boolean cMemoryOwn) {
    super(isoObject_wrapJNI.PreArming_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(PreArming obj) {
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
        isoObject_wrapJNI.delete_PreArming(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ObjectStateID getStateID() {
    return ObjectStateID.swigToEnum(isoObject_wrapJNI.PreArming_getStateID(swigCPtr, this));
  }

  public void onEnter(TestObject obj) {
    isoObject_wrapJNI.PreArming_onEnter(swigCPtr, this, TestObject.getCPtr(obj), obj);
  }

  public PreArming() {
    this(isoObject_wrapJNI.new_PreArming(), true);
  }

}
