package com.dji.GSDemo.GoogleMap;

import android.app.Application;
import android.content.Context;
import android.os.AsyncTask;
import android.util.Log;

import com.google.android.gms.maps.model.LatLng;
import com.secneo.sdk.Helper;

import org.asta.isoObject.CartesianPosition;
import org.asta.isoObject.SpeedType;
import org.asta.isoObject.TrajectoryWaypointVector;
import org.locationtech.proj4j.CRSFactory;
import org.locationtech.proj4j.CoordinateReferenceSystem;
import org.locationtech.proj4j.CoordinateTransform;
import org.locationtech.proj4j.CoordinateTransformFactory;
import org.locationtech.proj4j.ProjCoordinate;

public class MApplication extends Application {
    static {
        try {
            System.loadLibrary("isoObject_wrap");
        }catch(Exception e)
        {
            Log.wtf("Error", e);
        }



    }


    private DJIDemoApplication fpvDemoApplication;
    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);
        Helper.install(MApplication.this);
        if (fpvDemoApplication == null) {
            fpvDemoApplication = new DJIDemoApplication();
            fpvDemoApplication.setContext(this);
        }
    }





    @Override
    public void onCreate() {

        super.onCreate();
        // Uncomment following lines to test communication from ISOObject <-> ATOS, see Task class below


         //Task droneTask = new Task();
         //droneTask.run();

        // Uncomment following line to start connection to DJI drone and ISOObject <-> ATOS
        fpvDemoApplication.onCreate();

    }
}


/* Testing class to debug communication from ISOObject <-> ATOS */
class Task implements Runnable {
    CRSFactory crsFactory = new CRSFactory();
    CoordinateReferenceSystem WGS84 = crsFactory.createFromParameters("WGS84","+proj=longlat +datum=WGS84 +no_defs");

    @Override
    public void run() {
        IsoDrone drone = new IsoDrone("192.168.72.229");

        double droneLocationLat = 181, droneLocationLng = 181, droneAltitude = 0;

        double test = 0.01;
        String lastDroneState = "";
        boolean printOnce = true;
        while (true) {
            try {
                Thread.sleep(100);
               //Log.wtf("Name", drone.getName());
               Log.wtf("State", drone.getCurrentStateName());
               /*Log.wtf("IPv4", Utils.getIPAddress(true)); // IPv4*/

                //Log.wtf("Lat: ", String.valueOf(drone.getOrigin().getLatitude_deg()));
                //Log.wtf("Log: ", String.valueOf(drone.getOrigin().getLongitude_deg()));
                //Log.wtf("alt: ", String.valueOf(drone.getOrigin().getAltitude_m()));

                if (drone.getCurrentStateName().equals("Armed") || (drone.getCurrentStateName().equals("Running"))) {

                    CartesianPosition dronePos = new CartesianPosition();
                    SpeedType droneSpeed = new SpeedType();

                    droneSpeed.setIsLateralValid(true);
                    droneSpeed.setIsLongitudinalValid(true);
                    droneSpeed.setLateral_m_s(2);
                    droneSpeed.setLongitudinal_m_s(2);

                    dronePos.setXCoord_m(test);
                    dronePos.setYCoord_m(20);
                    dronePos.setZCoord_m(30);
                    dronePos.setHeading_rad(0);
                    test += 0.1;
                    dronePos.setIsPositionValid(true);
                    dronePos.setIsHeadingValid(true);
                    drone.setPosition(dronePos);
                    drone.setSpeed(droneSpeed);

                    // Debug prints
                    // Log.wtf("TrajName: ", drone.getTrajectoryHeader().getTrajectoryName());



                }

                if (drone.getCurrentStateName().equals("Armed") || (drone.getCurrentStateName().equals("Running"))) {

                    double isoLat = drone.getOrigin().getLatitude_deg();
                    double isoLog = drone.getOrigin().getLongitude_deg();

                    LatLng isoOrigin = new LatLng(isoLat, isoLog);

                    ProjCoordinate dronePosition = new ProjCoordinate(droneLocationLat, droneLocationLng);
                    ProjCoordinate result = coordGeoToCart(isoOrigin, dronePosition);

                    CartesianPosition monrPos = new CartesianPosition();
                    monrPos.setXCoord_m(result.x);
                    monrPos.setYCoord_m(result.y);
                    monrPos.setZCoord_m(droneAltitude);
                    monrPos.setIsPositionValid(true);

                    LatLng droneStart = new LatLng(droneLocationLat, droneLocationLng);
                    ProjCoordinate pc = coordGeoToCart(droneStart, dronePosition);
                    double heading = Math.acos(pc.x / (Math.sqrt(Math.pow(pc.x, 2) + Math.pow(pc.y, 2))));
                    monrPos.setHeading_rad(heading);
                    monrPos.setIsHeadingValid(true);

                    drone.setPosition(monrPos);
                }

                if (drone.getCurrentStateName().equals("Init") && lastDroneState != "Init") {
                    Log.wtf("Error", "Init");

                    lastDroneState = "Init";
                } else if (drone.getCurrentStateName().equals("PreArming") && lastDroneState != "PreArming") {
                    Log.wtf("Error", "PreArming");
                    lastDroneState = "PreArming";

                } else if (drone.getCurrentStateName().equals("Armed") && lastDroneState != "Armed") {
                    //button = (Button)findViewById(R.id.pauseresume);
                    //button.setText("Arming");


                    //trajectory

                    Log.wtf("TrajName: ", drone.getTrajectoryHeader().getTrajectoryName());
                    TrajectoryWaypointVector traj = drone.getTrajectory();
                    drone.reducedTraj = drone.getTrajectory();

                    //Reduce points in traj if to large (99 max amount of waypoints)
                    if (traj.size() > 99) {
                        double epsilon = 0.001;
                        do {
                            drone.reducePoints(epsilon);
                            epsilon += 0.001;
                        } while (drone.getReducedTraj().size() > 99 && epsilon < 0.08);
                        Log.wtf("newTraj", String.valueOf(drone.getReducedTraj().size()));
                        //waypointSettings.clear();
                        drone.removePointsToClose();
                        if(printOnce) {
                            TrajectoryWaypointVector trajRedux = drone.getReducedTraj();
                            Log.wtf("point over 99", "many many points");

                            for (int i = 0; i < trajRedux.size(); i++) {
                                //Log.wtf("point " + String.valueOf(i) + " X: ", String.valueOf(trajRedux.get(i).getPos().getXCoord_m()));
                                //Log.wtf("point " + String.valueOf(i) + " Y: ", String.valueOf(trajRedux.get(i).getPos().getYCoord_m()));
                                ProjCoordinate proj = coordCartToGeo(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), new ProjCoordinate(trajRedux.get(i).getPos().getXCoord_m(), trajRedux.get(i).getPos().getYCoord_m(), trajRedux.get(i).getPos().getZCoord_m()));
                                Log.wtf("LogLat:", " " + proj.toString());

                            }
                            printOnce = false;
                        }
                        generateWaypointsFromTraj(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), drone.getReducedTraj()); //Use reduced
                    } else {
                        //waypointSettings.clear();
                        drone.reducedTraj = drone.getTrajectory();
                        drone.removePointsToClose();
                        if(printOnce) {
                            TrajectoryWaypointVector trajRedux = drone.getReducedTraj();
                            Log.wtf("point less than 99", "so fewstorasdd points");

                            for (int i = 0; i < trajRedux.size(); i++) {
                                ProjCoordinate proj = coordCartToGeo(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), new ProjCoordinate(trajRedux.get(i).getPos().getXCoord_m(), trajRedux.get(i).getPos().getYCoord_m(), trajRedux.get(i).getPos().getZCoord_m()));
                                Log.wtf("LogLat:", " " + proj.toString());
                            }
                            printOnce = false;
                        }

                        Log.wtf("point less than 99", "Lat: " + Double.toString(drone.getOrigin().getLatitude_deg())+ " Long:" + Double.toString(drone.getOrigin().getLongitude_deg()));

                        generateWaypointsFromTraj(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), traj); // use set traj
                    }

                    //generateTestCircleCoordinates(new LatLng(droneLocationLat, droneLocationLng), 10, 3, 1,19, true);

                    Log.wtf("Error", "Armed");
                    lastDroneState = "Armed";
                } else if (drone.getCurrentStateName().equals("Disarmed") && lastDroneState != "Disarmed") {

                    Log.wtf("Error", "Disarmed");
                    lastDroneState = "Disarmed";
                } else if (drone.getCurrentStateName().equals("PreRunning") && lastDroneState != "PreRunning") {
                    Log.wtf("Error", "PreRunning");
                    lastDroneState = "PreRunning";
                } else if (drone.getCurrentStateName().equals("Running") && lastDroneState != "Running") {
                    Log.wtf("Error", "Running");
                    lastDroneState = "Running";
                } else if (drone.getCurrentStateName().equals("NormalStop") && lastDroneState != "NormalStop") {
                    lastDroneState = "NormalStop";
                } else if (drone.getCurrentStateName().equals("EmergencyStop") && lastDroneState != "EmergencyStop") {
                    Log.wtf("Error", "EmergencyStop");
                    lastDroneState = "EmergencyStop";
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


    private int convertToDroneYawRangeDeg(double yaw){
        int droneYaw = 0;
        if(yaw >= 0 && yaw <= 90) droneYaw = 90 - (int)(yaw);
        else if(yaw > 90 && yaw < 270) droneYaw = (90 - (int) (yaw));
        else if(yaw >= 270) droneYaw = ( 450 - (int)(yaw));
        return droneYaw;
    }

    void generateWaypointsFromTraj(LatLng origin, TrajectoryWaypointVector trajectory) {

        //WaypointSetting wpsHeadfix = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(newX, newY, 15)), new ProjCoordinate());
        WaypointSetting wpsHeadfix = new WaypointSetting(new ProjCoordinate(57.7779968, 12.780582, 15), new ProjCoordinate());


        wpsHeadfix.heading = convertToDroneYawRangeDeg((180/Math.PI)*trajectory.get(0).getPos().getHeading_rad());
        wpsHeadfix.geo.z = 11;//trajectory.get(i).getPos().getZCoord_m();
        wpsHeadfix.speed = (float)trajectory.get(0).getSpd().getLongitudinal_m_s(); //Possible lossy conversion?
        //this.waypointSettings.add(wpsHeadfix);

        for (int i = 0; i < trajectory.size(); i++) {

            WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(trajectory.get(i).getPos().getXCoord_m(), trajectory.get(i).getPos().getYCoord_m(), trajectory.get(i).getPos().getZCoord_m())), new ProjCoordinate());
           // this.waypointSettings.add(wps);
           // this.waypointSettings.get(i).heading = convertToDroneYawRangeDeg((180 / Math.PI) * trajectory.get(i).getPos().getHeading_rad());
           // this.waypointSettings.get(i).geo.z = 10; //trajectory.get(i).getPos().getZCoord_m();
           // this.waypointSettings.get(i).speed = (float) trajectory.get(i).getSpd().getLongitudinal_m_s(); //Possible lossy conversion?

        }

        WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(0, 0)), new ProjCoordinate());
        wps.heading = 0;
        wps.speed = 5;
        wps.geo.z = 10;
        //this.waypointSettings.add(wps);
    }

    ProjCoordinate coordCartToGeo(LatLng origin, ProjCoordinate xyz){
        String projStr = buildOriginProjString(origin.latitude, origin.longitude);
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateReferenceSystem UTM = crsFactory.createFromParameters("UTM", projStr);
        CoordinateTransform utmToWgs = ctFactory.createTransform(UTM, WGS84);
        ProjCoordinate result = new ProjCoordinate();
        utmToWgs.transform(new ProjCoordinate(xyz.x, xyz.y), result);
        return result;
    }

    ProjCoordinate coordGeoToCart(LatLng origin, ProjCoordinate llh){
        String projStr = buildOriginProjString(origin.latitude, origin.longitude);
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateReferenceSystem UTM = crsFactory.createFromParameters("UTM", projStr);
        CoordinateTransform wgsToUtm = ctFactory.createTransform(WGS84, UTM);
        ProjCoordinate result = new ProjCoordinate();
        wgsToUtm.transform(new ProjCoordinate(llh.y, llh.x), result);
        result.z = llh.z;
        return result;
    }

    private String buildOriginProjString(double latitude, double longitude){
        final StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("+proj=tmerc +lat_0=" + latitude + " +lon_0=" + longitude + " +k=0.9996 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs");
        return stringBuffer.toString();
    }


}
