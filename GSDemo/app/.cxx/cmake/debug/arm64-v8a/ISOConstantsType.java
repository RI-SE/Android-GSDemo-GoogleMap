/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.1
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package org.asta.isoObject;

public final class ISOConstantsType {
  public final static ISOConstantsType ISO_TRAJ_HEADER_SIZE = new ISOConstantsType("ISO_TRAJ_HEADER_SIZE", isoObject_wrapJNI.ISO_TRAJ_HEADER_SIZE_get());
  public final static ISOConstantsType ISO_TRAJ_WAYPOINT_SIZE = new ISOConstantsType("ISO_TRAJ_WAYPOINT_SIZE", isoObject_wrapJNI.ISO_TRAJ_WAYPOINT_SIZE_get());

  public final int swigValue() {
    return swigValue;
  }

  public String toString() {
    return swigName;
  }

  public static ISOConstantsType swigToEnum(int swigValue) {
    if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
      return swigValues[swigValue];
    for (int i = 0; i < swigValues.length; i++)
      if (swigValues[i].swigValue == swigValue)
        return swigValues[i];
    throw new IllegalArgumentException("No enum " + ISOConstantsType.class + " with value " + swigValue);
  }

  private ISOConstantsType(String swigName) {
    this.swigName = swigName;
    this.swigValue = swigNext++;
  }

  private ISOConstantsType(String swigName, int swigValue) {
    this.swigName = swigName;
    this.swigValue = swigValue;
    swigNext = swigValue+1;
  }

  private ISOConstantsType(String swigName, ISOConstantsType swigEnum) {
    this.swigName = swigName;
    this.swigValue = swigEnum.swigValue;
    swigNext = this.swigValue+1;
  }

  private static ISOConstantsType[] swigValues = { ISO_TRAJ_HEADER_SIZE, ISO_TRAJ_WAYPOINT_SIZE };
  private static int swigNext = 0;
  private final int swigValue;
  private final String swigName;
}

