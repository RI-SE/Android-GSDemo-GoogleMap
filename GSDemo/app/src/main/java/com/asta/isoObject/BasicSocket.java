/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.1
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.asta.isoObject;

public class BasicSocket {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected BasicSocket(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(BasicSocket obj) {
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
        isoObject_wrapJNI.delete_BasicSocket(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  static public class HostInfo {
    private transient long swigCPtr;
    protected transient boolean swigCMemOwn;
  
    protected HostInfo(long cPtr, boolean cMemoryOwn) {
      swigCMemOwn = cMemoryOwn;
      swigCPtr = cPtr;
    }
  
    protected static long getCPtr(HostInfo obj) {
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
          isoObject_wrapJNI.delete_BasicSocket_HostInfo(swigCPtr);
        }
        swigCPtr = 0;
      }
    }
  
    public void setAddress(String value) {
      isoObject_wrapJNI.BasicSocket_HostInfo_address_set(swigCPtr, this, value);
    }
  
    public String getAddress() {
      return isoObject_wrapJNI.BasicSocket_HostInfo_address_get(swigCPtr, this);
    }
  
    public void setPort(int value) {
      isoObject_wrapJNI.BasicSocket_HostInfo_port_set(swigCPtr, this, value);
    }
  
    public int getPort() {
      return isoObject_wrapJNI.BasicSocket_HostInfo_port_get(swigCPtr, this);
    }
  
    public HostInfo() {
      this(isoObject_wrapJNI.new_BasicSocket_HostInfo(), true);
    }
  
  }

  public BasicSocket() {
    this(isoObject_wrapJNI.new_BasicSocket__SWIG_0(), true);
  }

  public BasicSocket(BasicSocket.SocketType type, boolean debug) {
    this(isoObject_wrapJNI.new_BasicSocket__SWIG_1(type.swigValue(), debug), true);
  }

  public BasicSocket(BasicSocket.SocketType type) {
    this(isoObject_wrapJNI.new_BasicSocket__SWIG_2(type.swigValue()), true);
  }

  public BasicSocket(int sockfd, boolean debug) {
    this(isoObject_wrapJNI.new_BasicSocket__SWIG_3(sockfd, debug), true);
  }

  public BasicSocket(int sockfd) {
    this(isoObject_wrapJNI.new_BasicSocket__SWIG_4(sockfd), true);
  }

  public BasicSocket(BasicSocket other) {
    this(isoObject_wrapJNI.new_BasicSocket__SWIG_5(BasicSocket.getCPtr(other), other), true);
  }

  public BasicSocket SocketEqual(BasicSocket other) {
    return new BasicSocket(isoObject_wrapJNI.BasicSocket_SocketEqual__SWIG_0(swigCPtr, this, BasicSocket.getCPtr(other), other), false);
  }

  public void setDebug(boolean enable) {
    isoObject_wrapJNI.BasicSocket_setDebug__SWIG_0(swigCPtr, this, enable);
  }

  public void setDebug() {
    isoObject_wrapJNI.BasicSocket_setDebug__SWIG_1(swigCPtr, this);
  }

  public void setReuseAddr(boolean reuseAddr) {
    isoObject_wrapJNI.BasicSocket_setReuseAddr__SWIG_0(swigCPtr, this, reuseAddr);
  }

  public void setReuseAddr() {
    isoObject_wrapJNI.BasicSocket_setReuseAddr__SWIG_1(swigCPtr, this);
  }

  public void setKeepAlive(boolean keepAlive) {
    isoObject_wrapJNI.BasicSocket_setKeepAlive__SWIG_0(swigCPtr, this, keepAlive);
  }

  public void setKeepAlive() {
    isoObject_wrapJNI.BasicSocket_setKeepAlive__SWIG_1(swigCPtr, this);
  }

  public void setLinger(boolean linger) {
    isoObject_wrapJNI.BasicSocket_setLinger__SWIG_0(swigCPtr, this, linger);
  }

  public void setLinger() {
    isoObject_wrapJNI.BasicSocket_setLinger__SWIG_1(swigCPtr, this);
  }

  public void setLingerSeconds(int arg0) {
    isoObject_wrapJNI.BasicSocket_setLingerSeconds(swigCPtr, this, arg0);
  }

  public void setBlocking(boolean blocking) {
    isoObject_wrapJNI.BasicSocket_setBlocking__SWIG_0(swigCPtr, this, blocking);
  }

  public void setBlocking() {
    isoObject_wrapJNI.BasicSocket_setBlocking__SWIG_1(swigCPtr, this);
  }

  public boolean getBlocking() {
    return isoObject_wrapJNI.BasicSocket_getBlocking(swigCPtr, this);
  }

  public BasicSocket.SocketType getType() {
    return BasicSocket.SocketType.swigToEnum(isoObject_wrapJNI.BasicSocket_getType(swigCPtr, this));
  }

  public String getRemoteIP() {
    return isoObject_wrapJNI.BasicSocket_getRemoteIP(swigCPtr, this);
  }

  public String getLocalIP() {
    return isoObject_wrapJNI.BasicSocket_getLocalIP(swigCPtr, this);
  }

  public int getRemotePort() {
    return isoObject_wrapJNI.BasicSocket_getRemotePort(swigCPtr, this);
  }

  public int getLocalPort() {
    return isoObject_wrapJNI.BasicSocket_getLocalPort(swigCPtr, this);
  }

  public void close() {
    isoObject_wrapJNI.BasicSocket_close(swigCPtr, this);
  }

  public void open(BasicSocket.SocketType type) {
    isoObject_wrapJNI.BasicSocket_open(swigCPtr, this, type.swigValue());
  }

  public final static class SocketType {
    public final static BasicSocket.SocketType STREAM = new BasicSocket.SocketType("STREAM", isoObject_wrapJNI.BasicSocket_STREAM_get());
    public final static BasicSocket.SocketType DATAGRAM = new BasicSocket.SocketType("DATAGRAM", isoObject_wrapJNI.BasicSocket_DATAGRAM_get());

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static SocketType swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + SocketType.class + " with value " + swigValue);
    }

    private SocketType(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private SocketType(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private SocketType(String swigName, SocketType swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static SocketType[] swigValues = { STREAM, DATAGRAM };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
