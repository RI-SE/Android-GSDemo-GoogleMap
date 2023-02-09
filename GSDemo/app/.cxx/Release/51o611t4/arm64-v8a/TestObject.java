/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.1
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public class TestObject {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected TestObject(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(TestObject obj) {
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
        isoObject_wrapJNI.delete_TestObject(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  protected void swigDirectorDisconnect() {
    swigCMemOwn = false;
    delete();
  }

  public void swigReleaseOwnership() {
    swigCMemOwn = false;
    isoObject_wrapJNI.TestObject_change_ownership(this, swigCPtr, false);
  }

  public void swigTakeOwnership() {
    swigCMemOwn = true;
    isoObject_wrapJNI.TestObject_change_ownership(this, swigCPtr, true);
  }

  public TestObject(String listenIP) {
    this(isoObject_wrapJNI.new_TestObject__SWIG_0(listenIP), true);
    isoObject_wrapJNI.TestObject_director_connect(this, swigCPtr, true, true);
  }

  public TestObject() {
    this(isoObject_wrapJNI.new_TestObject__SWIG_1(), true);
    isoObject_wrapJNI.TestObject_director_connect(this, swigCPtr, true, true);
  }

  public void disconnect() {
    isoObject_wrapJNI.TestObject_disconnect(swigCPtr, this);
  }

  public void setPosition(CartesianPosition pos) {
    isoObject_wrapJNI.TestObject_setPosition(swigCPtr, this, CartesianPosition.getCPtr(pos), pos);
  }

  public void setSpeed(SpeedType spd) {
    isoObject_wrapJNI.TestObject_setSpeed(swigCPtr, this, SpeedType.getCPtr(spd), spd);
  }

  public void setAcceleration(AccelerationType acc) {
    isoObject_wrapJNI.TestObject_setAcceleration(swigCPtr, this, AccelerationType.getCPtr(acc), acc);
  }

  public void setDriveDirection(SWIGTYPE_p_DriveDirectionType drd) {
    isoObject_wrapJNI.TestObject_setDriveDirection(swigCPtr, this, SWIGTYPE_p_DriveDirectionType.getCPtr(drd));
  }

  public void setObjectState(ObjectStateID ost) {
    isoObject_wrapJNI.TestObject_setObjectState(swigCPtr, this, ost.swigValue());
  }

  public void setName(String nm) {
    isoObject_wrapJNI.TestObject_setName(swigCPtr, this, nm);
  }

  public void setReadyToArm(int rdy) {
    isoObject_wrapJNI.TestObject_setReadyToArm(swigCPtr, this, rdy);
  }

  public void setErrorState(char err) {
    isoObject_wrapJNI.TestObject_setErrorState(swigCPtr, this, err);
  }

  public String getCurrentStateName() {
    return isoObject_wrapJNI.TestObject_getCurrentStateName(swigCPtr, this);
  }

  public String getName() {
    return isoObject_wrapJNI.TestObject_getName(swigCPtr, this);
  }

  public CartesianPosition getPosition() {
    return new CartesianPosition(isoObject_wrapJNI.TestObject_getPosition(swigCPtr, this), true);
  }

  public SpeedType getSpeed() {
    return new SpeedType(isoObject_wrapJNI.TestObject_getSpeed(swigCPtr, this), true);
  }

  public AccelerationType getAcceleration() {
    return new AccelerationType(isoObject_wrapJNI.TestObject_getAcceleration(swigCPtr, this), true);
  }

  public SWIGTYPE_p_DriveDirectionType getDriveDirection() {
    return new SWIGTYPE_p_DriveDirectionType(isoObject_wrapJNI.TestObject_getDriveDirection(swigCPtr, this), true);
  }

  public TrajectoryHeaderType getTrajectoryHeader() {
    return new TrajectoryHeaderType(isoObject_wrapJNI.TestObject_getTrajectoryHeader(swigCPtr, this), true);
  }

  public SWIGTYPE_p_std__vectorT_TrajectoryWaypointType_t getTrajectory() {
    return new SWIGTYPE_p_std__vectorT_TrajectoryWaypointType_t(isoObject_wrapJNI.TestObject_getTrajectory(swigCPtr, this), true);
  }

  public GeographicPositionType getOrigin() {
    return new GeographicPositionType(isoObject_wrapJNI.TestObject_getOrigin(swigCPtr, this), true);
  }

  public String getLocalIP() {
    return isoObject_wrapJNI.TestObject_getLocalIP(swigCPtr, this);
  }

  public long getTransmitterID() {
    return isoObject_wrapJNI.TestObject_getTransmitterID(swigCPtr, this);
  }

  protected void handleAbort() {
    if (getClass() == TestObject.class) isoObject_wrapJNI.TestObject_handleAbort(swigCPtr, this); else isoObject_wrapJNI.TestObject_handleAbortSwigExplicitTestObject(swigCPtr, this);
  }

  protected int handleVendorSpecificMessage(int msgType, SWIGTYPE_p_std__vectorT_char_t data) {
    return (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_handleVendorSpecificMessage(swigCPtr, this, msgType, SWIGTYPE_p_std__vectorT_char_t.getCPtr(data)) : isoObject_wrapJNI.TestObject_handleVendorSpecificMessageSwigExplicitTestObject(swigCPtr, this, msgType, SWIGTYPE_p_std__vectorT_char_t.getCPtr(data));
  }

  protected Unknown createUnknown() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createUnknown(swigCPtr, this) : isoObject_wrapJNI.TestObject_createUnknownSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new Unknown(cPtr, false);
  }

  protected Off createOff() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createOff(swigCPtr, this) : isoObject_wrapJNI.TestObject_createOffSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new Off(cPtr, false);
  }

  protected Init createInit() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createInit(swigCPtr, this) : isoObject_wrapJNI.TestObject_createInitSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new Init(cPtr, false);
  }

  protected Armed createArmed() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createArmed(swigCPtr, this) : isoObject_wrapJNI.TestObject_createArmedSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new Armed(cPtr, false);
  }

  protected Disarmed createDisarmed() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createDisarmed(swigCPtr, this) : isoObject_wrapJNI.TestObject_createDisarmedSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new Disarmed(cPtr, false);
  }

  protected Running createRunning() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createRunning(swigCPtr, this) : isoObject_wrapJNI.TestObject_createRunningSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new Running(cPtr, false);
  }

  protected PostRun createPostRun() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createPostRun(swigCPtr, this) : isoObject_wrapJNI.TestObject_createPostRunSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new PostRun(cPtr, false);
  }

  protected RemoteControlled createRemoteControlled() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createRemoteControlled(swigCPtr, this) : isoObject_wrapJNI.TestObject_createRemoteControlledSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new RemoteControlled(cPtr, false);
  }

  protected Aborting createAborting() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createAborting(swigCPtr, this) : isoObject_wrapJNI.TestObject_createAbortingSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new Aborting(cPtr, false);
  }

  protected PreArming createPreArming() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createPreArming(swigCPtr, this) : isoObject_wrapJNI.TestObject_createPreArmingSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new PreArming(cPtr, false);
  }

  protected PreRunning createPreRunning() {
    long cPtr = (getClass() == TestObject.class) ? isoObject_wrapJNI.TestObject_createPreRunning(swigCPtr, this) : isoObject_wrapJNI.TestObject_createPreRunningSwigExplicitTestObject(swigCPtr, this);
    return (cPtr == 0) ? null : new PreRunning(cPtr, false);
  }

  protected void onStateChange() {
    if (getClass() == TestObject.class) isoObject_wrapJNI.TestObject_onStateChange(swigCPtr, this); else isoObject_wrapJNI.TestObject_onStateChangeSwigExplicitTestObject(swigCPtr, this);
  }

  protected void onOSEM(ObjectSettingsType arg0) {
    if (getClass() == TestObject.class) isoObject_wrapJNI.TestObject_onOSEM(swigCPtr, this, ObjectSettingsType.getCPtr(arg0), arg0); else isoObject_wrapJNI.TestObject_onOSEMSwigExplicitTestObject(swigCPtr, this, ObjectSettingsType.getCPtr(arg0), arg0);
  }

  protected void onHEAB(HeabMessageDataType arg0) {
    if (getClass() == TestObject.class) isoObject_wrapJNI.TestObject_onHEAB(swigCPtr, this, HeabMessageDataType.getCPtr(arg0), arg0); else isoObject_wrapJNI.TestObject_onHEABSwigExplicitTestObject(swigCPtr, this, HeabMessageDataType.getCPtr(arg0), arg0);
  }

  protected void onTRAJ() {
    if (getClass() == TestObject.class) isoObject_wrapJNI.TestObject_onTRAJ(swigCPtr, this); else isoObject_wrapJNI.TestObject_onTRAJSwigExplicitTestObject(swigCPtr, this);
  }

  protected void onOSTM(SWIGTYPE_p_ObjectCommandType arg0) {
    if (getClass() == TestObject.class) isoObject_wrapJNI.TestObject_onOSTM(swigCPtr, this, SWIGTYPE_p_ObjectCommandType.getCPtr(arg0)); else isoObject_wrapJNI.TestObject_onOSTMSwigExplicitTestObject(swigCPtr, this, SWIGTYPE_p_ObjectCommandType.getCPtr(arg0));
  }

  protected void onSTRT(StartMessageType arg0) {
    if (getClass() == TestObject.class) isoObject_wrapJNI.TestObject_onSTRT(swigCPtr, this, StartMessageType.getCPtr(arg0), arg0); else isoObject_wrapJNI.TestObject_onSTRTSwigExplicitTestObject(swigCPtr, this, StartMessageType.getCPtr(arg0), arg0);
  }

}
