/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public class RequestControlActionType {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected RequestControlActionType(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(RequestControlActionType obj) {
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
        isoObject_wrapJNI.delete_RequestControlActionType(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setExecutingID(long value) {
    isoObject_wrapJNI.RequestControlActionType_executingID_set(swigCPtr, this, value);
  }

  public long getExecutingID() {
    return isoObject_wrapJNI.RequestControlActionType_executingID_get(swigCPtr, this);
  }

  public void setDataTimestamp(timeval value) {
    isoObject_wrapJNI.RequestControlActionType_dataTimestamp_set(swigCPtr, this, timeval.getCPtr(value), value);
  }

  public timeval getDataTimestamp() {
    long cPtr = isoObject_wrapJNI.RequestControlActionType_dataTimestamp_get(swigCPtr, this);
    return (cPtr == 0) ? null : new timeval(cPtr, false);
  }

  static public class steeringActions {
    private transient long swigCPtr;
    protected transient boolean swigCMemOwn;
  
    protected steeringActions(long cPtr, boolean cMemoryOwn) {
      swigCMemOwn = cMemoryOwn;
      swigCPtr = cPtr;
    }
  
    protected static long getCPtr(steeringActions obj) {
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
          isoObject_wrapJNI.delete_RequestControlActionType_steeringActions(swigCPtr);
        }
        swigCPtr = 0;
      }
    }
  
    public void setPct(double value) {
      isoObject_wrapJNI.RequestControlActionType_steeringActions_pct_set(swigCPtr, this, value);
    }
  
    public double getPct() {
      return isoObject_wrapJNI.RequestControlActionType_steeringActions_pct_get(swigCPtr, this);
    }
  
    public void setRad(double value) {
      isoObject_wrapJNI.RequestControlActionType_steeringActions_rad_set(swigCPtr, this, value);
    }
  
    public double getRad() {
      return isoObject_wrapJNI.RequestControlActionType_steeringActions_rad_get(swigCPtr, this);
    }
  
    public steeringActions() {
      this(isoObject_wrapJNI.new_RequestControlActionType_steeringActions(), true);
    }
  
  }

  public void setSteeringAction(RequestControlActionType.steeringActions value) {
    isoObject_wrapJNI.RequestControlActionType_steeringAction_set(swigCPtr, this, RequestControlActionType.steeringActions.getCPtr(value), value);
  }

  public RequestControlActionType.steeringActions getSteeringAction() {
    long cPtr = isoObject_wrapJNI.RequestControlActionType_steeringAction_get(swigCPtr, this);
    return (cPtr == 0) ? null : new RequestControlActionType.steeringActions(cPtr, false);
  }

  static public class speedActions {
    private transient long swigCPtr;
    protected transient boolean swigCMemOwn;
  
    protected speedActions(long cPtr, boolean cMemoryOwn) {
      swigCMemOwn = cMemoryOwn;
      swigCPtr = cPtr;
    }
  
    protected static long getCPtr(speedActions obj) {
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
          isoObject_wrapJNI.delete_RequestControlActionType_speedActions(swigCPtr);
        }
        swigCPtr = 0;
      }
    }
  
    public void setPct(double value) {
      isoObject_wrapJNI.RequestControlActionType_speedActions_pct_set(swigCPtr, this, value);
    }
  
    public double getPct() {
      return isoObject_wrapJNI.RequestControlActionType_speedActions_pct_get(swigCPtr, this);
    }
  
    public void setM_s(double value) {
      isoObject_wrapJNI.RequestControlActionType_speedActions_m_s_set(swigCPtr, this, value);
    }
  
    public double getM_s() {
      return isoObject_wrapJNI.RequestControlActionType_speedActions_m_s_get(swigCPtr, this);
    }
  
    public speedActions() {
      this(isoObject_wrapJNI.new_RequestControlActionType_speedActions(), true);
    }
  
  }

  public void setSpeedAction(RequestControlActionType.speedActions value) {
    isoObject_wrapJNI.RequestControlActionType_speedAction_set(swigCPtr, this, RequestControlActionType.speedActions.getCPtr(value), value);
  }

  public RequestControlActionType.speedActions getSpeedAction() {
    long cPtr = isoObject_wrapJNI.RequestControlActionType_speedAction_get(swigCPtr, this);
    return (cPtr == 0) ? null : new RequestControlActionType.speedActions(cPtr, false);
  }

  public void setIsSteeringActionValid(boolean value) {
    isoObject_wrapJNI.RequestControlActionType_isSteeringActionValid_set(swigCPtr, this, value);
  }

  public boolean getIsSteeringActionValid() {
    return isoObject_wrapJNI.RequestControlActionType_isSteeringActionValid_get(swigCPtr, this);
  }

  public void setIsSpeedActionValid(boolean value) {
    isoObject_wrapJNI.RequestControlActionType_isSpeedActionValid_set(swigCPtr, this, value);
  }

  public boolean getIsSpeedActionValid() {
    return isoObject_wrapJNI.RequestControlActionType_isSpeedActionValid_get(swigCPtr, this);
  }

  public void setSteeringUnit(ISOUnitType value) {
    isoObject_wrapJNI.RequestControlActionType_steeringUnit_set(swigCPtr, this, value.swigValue());
  }

  public ISOUnitType getSteeringUnit() {
    return ISOUnitType.swigToEnum(isoObject_wrapJNI.RequestControlActionType_steeringUnit_get(swigCPtr, this));
  }

  public void setSpeedUnit(ISOUnitType value) {
    isoObject_wrapJNI.RequestControlActionType_speedUnit_set(swigCPtr, this, value.swigValue());
  }

  public ISOUnitType getSpeedUnit() {
    return ISOUnitType.swigToEnum(isoObject_wrapJNI.RequestControlActionType_speedUnit_get(swigCPtr, this));
  }

  public RequestControlActionType() {
    this(isoObject_wrapJNI.new_RequestControlActionType(), true);
  }

}