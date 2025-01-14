package com.dji.GSDemo.GoogleMap;

import org.locationtech.proj4j.ProjCoordinate;

import java.util.ArrayList;
import java.util.List;

public class WaypointSetting {

    public ProjCoordinate geo;
    public ProjCoordinate poi;
    public int heading;
    public float speed;
    public double radius;

    public WaypointSetting(ProjCoordinate geollh, ProjCoordinate poiXyz)
    {
      this.geo = geollh;
      this.poi = poiXyz;
    }
}
