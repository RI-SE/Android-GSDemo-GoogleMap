/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public final class TrajectoryInfoType {
  public final static TrajectoryInfoType TRAJECTORY_INFO_RELATIVE_TO_OBJECT = new TrajectoryInfoType("TRAJECTORY_INFO_RELATIVE_TO_OBJECT", isoObject_wrapJNI.TRAJECTORY_INFO_RELATIVE_TO_OBJECT_get());
  public final static TrajectoryInfoType TRAJECTORY_INFO_RELATIVE_TO_ORIGIN = new TrajectoryInfoType("TRAJECTORY_INFO_RELATIVE_TO_ORIGIN", isoObject_wrapJNI.TRAJECTORY_INFO_RELATIVE_TO_ORIGIN_get());
  public final static TrajectoryInfoType TRAJECTORY_INFO_DELETE_TRAJECTORY = new TrajectoryInfoType("TRAJECTORY_INFO_DELETE_TRAJECTORY", isoObject_wrapJNI.TRAJECTORY_INFO_DELETE_TRAJECTORY_get());

  public final int swigValue() {
    return swigValue;
  }

  public String toString() {
    return swigName;
  }

  public static TrajectoryInfoType swigToEnum(int swigValue) {
    if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
      return swigValues[swigValue];
    for (int i = 0; i < swigValues.length; i++)
      if (swigValues[i].swigValue == swigValue)
        return swigValues[i];
    throw new IllegalArgumentException("No enum " + TrajectoryInfoType.class + " with value " + swigValue);
  }

  private TrajectoryInfoType(String swigName) {
    this.swigName = swigName;
    this.swigValue = swigNext++;
  }

  private TrajectoryInfoType(String swigName, int swigValue) {
    this.swigName = swigName;
    this.swigValue = swigValue;
    swigNext = swigValue+1;
  }

  private TrajectoryInfoType(String swigName, TrajectoryInfoType swigEnum) {
    this.swigName = swigName;
    this.swigValue = swigEnum.swigValue;
    swigNext = this.swigValue+1;
  }

  private static TrajectoryInfoType[] swigValues = { TRAJECTORY_INFO_RELATIVE_TO_OBJECT, TRAJECTORY_INFO_RELATIVE_TO_ORIGIN, TRAJECTORY_INFO_DELETE_TRAJECTORY };
  private static int swigNext = 0;
  private final int swigValue;
  private final String swigName;
}

