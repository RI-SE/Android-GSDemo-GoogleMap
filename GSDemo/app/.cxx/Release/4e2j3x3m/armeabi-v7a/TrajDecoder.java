/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public class TrajDecoder {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected TrajDecoder(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(TrajDecoder obj) {
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
        isoObject_wrapJNI.delete_TrajDecoder(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public TrajDecoder(boolean debug) {
    this(isoObject_wrapJNI.new_TrajDecoder__SWIG_0(debug), true);
  }

  public TrajDecoder() {
    this(isoObject_wrapJNI.new_TrajDecoder__SWIG_1(), true);
  }

  public int DecodeTRAJ(SWIGTYPE_p_std__vectorT_char_t arg0) {
    return isoObject_wrapJNI.TrajDecoder_DecodeTRAJ(swigCPtr, this, SWIGTYPE_p_std__vectorT_char_t.getCPtr(arg0));
  }

  public boolean ExpectingTrajPoints() {
    return isoObject_wrapJNI.TrajDecoder_ExpectingTrajPoints(swigCPtr, this);
  }

  public TrajectoryHeaderType getTrajHeader() {
    return new TrajectoryHeaderType(isoObject_wrapJNI.TrajDecoder_getTrajHeader(swigCPtr, this), true);
  }

  public TrajectoryWaypointVector getTraj() {
    return new TrajectoryWaypointVector(isoObject_wrapJNI.TrajDecoder_getTraj(swigCPtr, this), true);
  }

}