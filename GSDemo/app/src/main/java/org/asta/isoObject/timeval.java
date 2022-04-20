/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.1
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public class timeval {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected timeval(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(timeval obj) {
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
        isoObject_wrapJNI.delete_timeval(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setTv_sec(int value) {
    isoObject_wrapJNI.timeval_tv_sec_set(swigCPtr, this, value);
  }

  public int getTv_sec() {
    return isoObject_wrapJNI.timeval_tv_sec_get(swigCPtr, this);
  }

  public void setTv_usec(int value) {
    isoObject_wrapJNI.timeval_tv_usec_set(swigCPtr, this, value);
  }

  public int getTv_usec() {
    return isoObject_wrapJNI.timeval_tv_usec_get(swigCPtr, this);
  }

  public timeval() {
    this(isoObject_wrapJNI.new_timeval(), true);
  }

}