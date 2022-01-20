/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.isoObject;

public final class DriveDirectionType {
  public final static DriveDirectionType OBJECT_DRIVE_DIRECTION_FORWARD = new DriveDirectionType("OBJECT_DRIVE_DIRECTION_FORWARD");
  public final static DriveDirectionType OBJECT_DRIVE_DIRECTION_BACKWARD = new DriveDirectionType("OBJECT_DRIVE_DIRECTION_BACKWARD");
  public final static DriveDirectionType OBJECT_DRIVE_DIRECTION_UNAVAILABLE = new DriveDirectionType("OBJECT_DRIVE_DIRECTION_UNAVAILABLE");

  public final int swigValue() {
    return swigValue;
  }

  public String toString() {
    return swigName;
  }

  public static DriveDirectionType swigToEnum(int swigValue) {
    if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
      return swigValues[swigValue];
    for (int i = 0; i < swigValues.length; i++)
      if (swigValues[i].swigValue == swigValue)
        return swigValues[i];
    throw new IllegalArgumentException("No enum " + DriveDirectionType.class + " with value " + swigValue);
  }

  private DriveDirectionType(String swigName) {
    this.swigName = swigName;
    this.swigValue = swigNext++;
  }

  private DriveDirectionType(String swigName, int swigValue) {
    this.swigName = swigName;
    this.swigValue = swigValue;
    swigNext = swigValue+1;
  }

  private DriveDirectionType(String swigName, DriveDirectionType swigEnum) {
    this.swigName = swigName;
    this.swigValue = swigEnum.swigValue;
    swigNext = this.swigValue+1;
  }

  private static DriveDirectionType[] swigValues = { OBJECT_DRIVE_DIRECTION_FORWARD, OBJECT_DRIVE_DIRECTION_BACKWARD, OBJECT_DRIVE_DIRECTION_UNAVAILABLE };
  private static int swigNext = 0;
  private final int swigValue;
  private final String swigName;
}

