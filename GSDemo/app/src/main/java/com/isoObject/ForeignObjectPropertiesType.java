/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.isoObject;

public class ForeignObjectPropertiesType {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected ForeignObjectPropertiesType(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(ForeignObjectPropertiesType obj) {
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
        isoObjectJNI.delete_ForeignObjectPropertiesType(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public void setForeignTransmitterID(long value) {
    isoObjectJNI.ForeignObjectPropertiesType_foreignTransmitterID_set(swigCPtr, this, value);
  }

  public long getForeignTransmitterID() {
    return isoObjectJNI.ForeignObjectPropertiesType_foreignTransmitterID_get(swigCPtr, this);
  }

  public void setObjectType(ObjectCategoryType value) {
    isoObjectJNI.ForeignObjectPropertiesType_objectType_set(swigCPtr, this, value.swigValue());
  }

  public ObjectCategoryType getObjectType() {
    return ObjectCategoryType.swigToEnum(isoObjectJNI.ForeignObjectPropertiesType_objectType_get(swigCPtr, this));
  }

  public void setActorType(ActorType value) {
    isoObjectJNI.ForeignObjectPropertiesType_actorType_set(swigCPtr, this, value.swigValue());
  }

  public ActorType getActorType() {
    return ActorType.swigToEnum(isoObjectJNI.ForeignObjectPropertiesType_actorType_get(swigCPtr, this));
  }

  public void setOperationMode(OperationMode value) {
    isoObjectJNI.ForeignObjectPropertiesType_operationMode_set(swigCPtr, this, value.swigValue());
  }

  public OperationMode getOperationMode() {
    return OperationMode.swigToEnum(isoObjectJNI.ForeignObjectPropertiesType_operationMode_get(swigCPtr, this));
  }

  public void setMass_kg(double value) {
    isoObjectJNI.ForeignObjectPropertiesType_mass_kg_set(swigCPtr, this, value);
  }

  public double getMass_kg() {
    return isoObjectJNI.ForeignObjectPropertiesType_mass_kg_get(swigCPtr, this);
  }

  public void setObjectXDimension_m(double value) {
    isoObjectJNI.ForeignObjectPropertiesType_objectXDimension_m_set(swigCPtr, this, value);
  }

  public double getObjectXDimension_m() {
    return isoObjectJNI.ForeignObjectPropertiesType_objectXDimension_m_get(swigCPtr, this);
  }

  public void setObjectYDimension_m(double value) {
    isoObjectJNI.ForeignObjectPropertiesType_objectYDimension_m_set(swigCPtr, this, value);
  }

  public double getObjectYDimension_m() {
    return isoObjectJNI.ForeignObjectPropertiesType_objectYDimension_m_get(swigCPtr, this);
  }

  public void setObjectZDimension_m(double value) {
    isoObjectJNI.ForeignObjectPropertiesType_objectZDimension_m_set(swigCPtr, this, value);
  }

  public double getObjectZDimension_m() {
    return isoObjectJNI.ForeignObjectPropertiesType_objectZDimension_m_get(swigCPtr, this);
  }

  public void setPositionDisplacementX_m(double value) {
    isoObjectJNI.ForeignObjectPropertiesType_positionDisplacementX_m_set(swigCPtr, this, value);
  }

  public double getPositionDisplacementX_m() {
    return isoObjectJNI.ForeignObjectPropertiesType_positionDisplacementX_m_get(swigCPtr, this);
  }

  public void setPositionDisplacementY_m(double value) {
    isoObjectJNI.ForeignObjectPropertiesType_positionDisplacementY_m_set(swigCPtr, this, value);
  }

  public double getPositionDisplacementY_m() {
    return isoObjectJNI.ForeignObjectPropertiesType_positionDisplacementY_m_get(swigCPtr, this);
  }

  public void setPositionDisplacementZ_m(double value) {
    isoObjectJNI.ForeignObjectPropertiesType_positionDisplacementZ_m_set(swigCPtr, this, value);
  }

  public double getPositionDisplacementZ_m() {
    return isoObjectJNI.ForeignObjectPropertiesType_positionDisplacementZ_m_get(swigCPtr, this);
  }

  public void setIsMassValid(boolean value) {
    isoObjectJNI.ForeignObjectPropertiesType_isMassValid_set(swigCPtr, this, value);
  }

  public boolean getIsMassValid() {
    return isoObjectJNI.ForeignObjectPropertiesType_isMassValid_get(swigCPtr, this);
  }

  public void setIsObjectXDimensionValid(boolean value) {
    isoObjectJNI.ForeignObjectPropertiesType_isObjectXDimensionValid_set(swigCPtr, this, value);
  }

  public boolean getIsObjectXDimensionValid() {
    return isoObjectJNI.ForeignObjectPropertiesType_isObjectXDimensionValid_get(swigCPtr, this);
  }

  public void setIsObjectYDimensionValid(boolean value) {
    isoObjectJNI.ForeignObjectPropertiesType_isObjectYDimensionValid_set(swigCPtr, this, value);
  }

  public boolean getIsObjectYDimensionValid() {
    return isoObjectJNI.ForeignObjectPropertiesType_isObjectYDimensionValid_get(swigCPtr, this);
  }

  public void setIsObjectZDimensionValid(boolean value) {
    isoObjectJNI.ForeignObjectPropertiesType_isObjectZDimensionValid_set(swigCPtr, this, value);
  }

  public boolean getIsObjectZDimensionValid() {
    return isoObjectJNI.ForeignObjectPropertiesType_isObjectZDimensionValid_get(swigCPtr, this);
  }

  public void setIsObjectXDisplacementValid(boolean value) {
    isoObjectJNI.ForeignObjectPropertiesType_isObjectXDisplacementValid_set(swigCPtr, this, value);
  }

  public boolean getIsObjectXDisplacementValid() {
    return isoObjectJNI.ForeignObjectPropertiesType_isObjectXDisplacementValid_get(swigCPtr, this);
  }

  public void setIsObjectYDisplacementValid(boolean value) {
    isoObjectJNI.ForeignObjectPropertiesType_isObjectYDisplacementValid_set(swigCPtr, this, value);
  }

  public boolean getIsObjectYDisplacementValid() {
    return isoObjectJNI.ForeignObjectPropertiesType_isObjectYDisplacementValid_get(swigCPtr, this);
  }

  public void setIsObjectZDisplacementValid(boolean value) {
    isoObjectJNI.ForeignObjectPropertiesType_isObjectZDisplacementValid_set(swigCPtr, this, value);
  }

  public boolean getIsObjectZDisplacementValid() {
    return isoObjectJNI.ForeignObjectPropertiesType_isObjectZDisplacementValid_get(swigCPtr, this);
  }

  public ForeignObjectPropertiesType() {
    this(isoObjectJNI.new_ForeignObjectPropertiesType(), true);
  }

}
