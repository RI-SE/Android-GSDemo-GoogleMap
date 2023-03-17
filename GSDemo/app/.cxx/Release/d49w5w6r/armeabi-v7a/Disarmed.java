/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public class Disarmed extends State {
  private transient long swigCPtr;

  protected Disarmed(long cPtr, boolean cMemoryOwn) {
    super(isoObject_wrapJNI.Disarmed_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Disarmed obj) {
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
        isoObject_wrapJNI.delete_Disarmed(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ObjectStateID getStateID() {
    return ObjectStateID.swigToEnum(isoObject_wrapJNI.Disarmed_getStateID(swigCPtr, this));
  }

  public void onEnter(TestObject obj) {
    isoObject_wrapJNI.Disarmed_onEnter(swigCPtr, this, TestObject.getCPtr(obj), obj);
  }

  public void onExit(TestObject obj) {
    isoObject_wrapJNI.Disarmed_onExit(swigCPtr, this, TestObject.getCPtr(obj), obj);
  }

  public Disarmed() {
    this(isoObject_wrapJNI.new_Disarmed(), true);
  }

}
