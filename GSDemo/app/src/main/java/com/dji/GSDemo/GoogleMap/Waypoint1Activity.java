package com.dji.GSDemo.GoogleMap;

import static dji.common.mission.MissionState.READY_TO_RETRY_UPLOAD;
import static dji.common.mission.waypoint.WaypointMissionState.EXECUTING;
import static dji.common.mission.waypoint.WaypointMissionState.EXECUTION_PAUSED;
import static dji.common.mission.waypoint.WaypointMissionState.READY_TO_EXECUTE;
import static dji.common.mission.waypoint.WaypointMissionState.READY_TO_UPLOAD;

import android.Manifest;
import android.app.AlertDialog;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;

import com.google.android.gms.maps.CameraUpdate;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionDownloadEvent;
import dji.common.mission.waypoint.WaypointMissionExecutionEvent;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionUploadEvent;
import dji.common.mission.waypoint.WaypointTurnMode;
import dji.common.model.LocationCoordinate2D;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.mission.waypoint.WaypointMissionOperatorListener;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.useraccount.UserAccountManager;


import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.DiagonalMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.asta.isoObject.*;
import org.locationtech.proj4j.CRSFactory;
import org.locationtech.proj4j.CoordinateReferenceSystem;
import org.locationtech.proj4j.CoordinateTransform;
import org.locationtech.proj4j.CoordinateTransformFactory;
import org.locationtech.proj4j.ProjCoordinate;

import org.apache.commons.math3.*;


public class Waypoint1Activity extends FragmentActivity implements View.OnClickListener, GoogleMap.OnMapClickListener, OnMapReadyCallback {


    static {
        try {
            System.loadLibrary("isoObject_wrap");
        }catch(Exception e)
        {
            Log.wtf("Error", e);
        }
    }

    protected static final String TAG = "GSDemoActivity";
    private IsoDrone drone;
    private String lastDroneState = "";
    private GoogleMap gMap;

    private Button locate, add, clear, testcircle;
    private Button config, upload, start, stop;
    private TextView positionStatus;

    private boolean isAdd = false;

    private LatLng startLatLong;
    private double droneLocationLat = 181, droneLocationLng = 181, droneAltitude = 0, droneHeading = 0;
    private final Map<Integer, Marker> mMarkers = new ConcurrentHashMap<Integer, Marker>();
    private Marker droneMarker = null;

    private float altitude = 100.0f;
    private float mSpeed = 15;
    private double droneRot = 0;

    private List<Waypoint> waypointList = new ArrayList<>();

    public static WaypointMission.Builder waypointMissionBuilder;
    private FlightController mFlightController;
    private WaypointMissionOperator instance;
    private WaypointMissionFinishedAction mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
    private WaypointMissionHeadingMode mHeadingMode = WaypointMissionHeadingMode.AUTO;
    public CRSFactory crsFactory = new CRSFactory();
    public CoordinateReferenceSystem WGS84 = crsFactory.createFromParameters("WGS84","+proj=longlat +datum=WGS84 +no_defs");

    private ArrayList<WaypointSetting> waypointSettings = new ArrayList<>();

    private Double mRot = 0.0;
    private boolean missionUploaded = false;
    private WaypointMissionOperatorListener listener;

    public double getDroneLocationLat(){return droneLocationLat;}
    public double getDroneLocationLng(){return droneLocationLng;}

    @Override
    public void onAttachedToWindow() {
        setUpListener();
    }

    @Override
    protected void onResume(){
        super.onResume();
        initFlightController();
    }

    @Override
    protected void onPause(){
        super.onPause();
    }

    @Override
    protected void onDestroy(){
        unregisterReceiver(mReceiver);
        removeListener();
        super.onDestroy();
    }

    /**
     * @Description : RETURN Button RESPONSE FUNCTION
     */
    public void onReturn(View view){
        Log.d(TAG, "onReturn");
        this.finish();
    }

    private void setResultToToast(final String string){
        Waypoint1Activity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(Waypoint1Activity.this, string, Toast.LENGTH_SHORT).show();
            }
        });
    }

    private void initUI() {

        locate = (Button) findViewById(R.id.locate);
        add = (Button) findViewById(R.id.add);
        clear = (Button) findViewById(R.id.clear);
        testcircle = (Button) findViewById(R.id.testcircle);
        config = (Button) findViewById(R.id.pauseresume);
        upload = (Button) findViewById(R.id.arm);
        start = (Button) findViewById(R.id.start);
        stop = (Button) findViewById(R.id.stop);
        positionStatus = (TextView)findViewById(R.id.textStatusxyzhead);

        locate.setOnClickListener(this);
        add.setOnClickListener(this);
        clear.setOnClickListener(this);
        testcircle.setOnClickListener(this);
        config.setOnClickListener(this);
        upload.setOnClickListener(this);
        start.setOnClickListener(this);
        stop.setOnClickListener(this);

        add.setEnabled(false);
    }

    private void generateWaypointsFromTraj(LatLng origin, TrajectoryWaypointVector trajectory){

        //Add extra point before first point to correct heading
        //add trajPoints
        RealMatrix A = MatrixUtils.createRealMatrix(trajectory.size(), trajectory.size());
        ArrayRealVector b = new ArrayRealVector(trajectory.size());
        for (int i = 0; i < trajectory.size(); ++i) {
            A.setEntry(i,i,1);
            if (i+1 < trajectory.size() ) {
                A.setEntry(i,i+1,1);
                Double dist = Math.sqrt(
                        Math.pow(trajectory.get(i).getPos().getXCoord_m() - trajectory.get(i+1).getPos().getXCoord_m(), 2)
                                + Math.pow(trajectory.get(i).getPos().getYCoord_m() - trajectory.get(i+1).getPos().getYCoord_m(), 2));
                b.setEntry(i, dist);
            }
            else {
                b.setEntry(i,0.2);
            }
        }
        DecompositionSolver solver = new LUDecomposition(A).getSolver();
        RealVector radii = solver.solve(b);

        //Get altitude from GUI
        EditText zAltitude   = (EditText)findViewById(R.id.zAltitude);
        zAltitude.getText();

        for(int i = 0; i < trajectory.size(); i ++){
            WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(trajectory.get(i).getPos().getXCoord_m(), trajectory.get(i).getPos().getYCoord_m(), trajectory.get(i).getPos().getZCoord_m())), new ProjCoordinate());
            this.waypointSettings.add(wps);
            this.waypointSettings.get(i).heading = (int)yawToHeading((180/Math.PI)*trajectory.get(i).getPos().getHeading_rad());
            this.waypointSettings.get(i).geo.z = Double.parseDouble(zAltitude.getText().toString());//trajectory.get(i).getPos().getZCoord_m();
            this.waypointSettings.get(i).speed = (float)trajectory.get(i).getSpd().getLongitudinal_m_s(); //Possible lossy conversion?
            this.waypointSettings.get(i).radius = radii.getEntry(i) -0.01;
        }

        //add landing point
       // WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(0, 0)), new ProjCoordinate());
       // wps.heading = 0;
       // wps.speed = 15;
       // wps.geo.z = 6;
       // this.waypointSettings.add(wps);
    }



    private void generateTestCircleCoordinates(LatLng origin, double radius, double altitude, float speed, int nofPoints, boolean headingTowardsCenter){

        double angularStep = 2*Math.PI/nofPoints;
        double currentAngle = 0;
        double currentAngleRot = 0;
        int wpHeading = 0;
        for(int i = 0; i <= nofPoints; i ++){

            if(headingTowardsCenter) {
                currentAngleRot = rotateUnitCircleAngleToDroneYawRad(currentAngle, Math.PI);
            } else currentAngleRot = currentAngle;
            wpHeading = (int)yawToHeading((180/Math.PI)*currentAngleRot);

            WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(radius*Math.cos(currentAngle), radius*Math.sin(currentAngle), 0)), new ProjCoordinate());
            this.waypointSettings.add(wps);
            this.waypointSettings.get(i).heading = wpHeading;
            this.waypointSettings.get(i).geo.z = altitude;
            this.waypointSettings.get(i).speed = speed;
            currentAngle += angularStep;
        }
        WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(0, 0)), new ProjCoordinate());
        wps.heading = 0;
        wps.speed = speed;
        wps.geo.z = altitude;
        this.waypointSettings.add(wps);
    }

    private Double rotateUnitCircleAngleToNorthHeadingRad(double yaw){
        Double yawRot = 0.0;
        if (yaw >= 0 && yaw <= Math.PI/2) yawRot = Math.PI/2 - yaw;
        else if (yaw > Math.PI/2 && yaw <= 2*Math.PI) yawRot = Math.PI/2 + 2*Math.PI - yaw;
        return yawRot;
    }

    private Double rotateUnitCircleAngleToDroneYawRad(double yaw, double rot){
        Double yawRot = 0.0;
        if (yaw >= 0 && yaw <= Math.PI) yawRot = yaw + rot;
        else if (yaw > Math.PI && yaw <= 2 * Math.PI) yawRot = yaw - rot;
        return yawRot;
    }

    private double headingToYaw(double heading_deg){
        return wrapAngle360(90-heading_deg);
    }

    private double yawToHeading(double yaw_deg){
        return wrapAngle180(90-yaw_deg);
    }

    private double wrapAngle360(double yaw_deg){
        while (yaw_deg < 0) {
            yaw_deg += 360;
        }
        while (yaw_deg > 360) {
            yaw_deg -= 360;
        }
        return yaw_deg;
    }

    private double wrapAngle180(double yaw_deg){
        while (yaw_deg < - 180) {
            yaw_deg += 360;
        }
        while (yaw_deg > 180) {
            yaw_deg -= 360;
        }
        return yaw_deg;
    }



    private String buildOriginProjString(double latitude, double longitude){
        final StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("+proj=tmerc +lat_0=" + latitude + " +lon_0=" + longitude + " +k=0.9996 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs");
        return stringBuffer.toString();
    }

    private ProjCoordinate coordCartToGeo(LatLng origin, ProjCoordinate xyz){
        String projStr = buildOriginProjString(origin.latitude, origin.longitude);
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateReferenceSystem UTM = crsFactory.createFromParameters("UTM", projStr);
        CoordinateTransform utmToWgs = ctFactory.createTransform(UTM, WGS84);
        ProjCoordinate result = new ProjCoordinate();
        utmToWgs.transform(new ProjCoordinate(xyz.x, xyz.y), result);
        return result;
    }

    private ProjCoordinate coordGeoToCart(LatLng origin, ProjCoordinate llh){
        String projStr = buildOriginProjString(origin.latitude, origin.longitude);
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateReferenceSystem UTM = crsFactory.createFromParameters("UTM", projStr);
        CoordinateTransform wgsToUtm = ctFactory.createTransform(WGS84, UTM);
        ProjCoordinate result = new ProjCoordinate();
        wgsToUtm.transform(new ProjCoordinate(llh.y, llh.x), result);
        result.z = llh.z;
        return result;
    }



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);


        // When the compile and target version is higher than 22, please request the
        // following permissions at runtime to ensure the
        // SDK work well.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.VIBRATE,
                            Manifest.permission.INTERNET, Manifest.permission.ACCESS_WIFI_STATE,
                            Manifest.permission.WAKE_LOCK, Manifest.permission.ACCESS_COARSE_LOCATION,
                            Manifest.permission.ACCESS_NETWORK_STATE, Manifest.permission.ACCESS_FINE_LOCATION,
                            Manifest.permission.CHANGE_WIFI_STATE, Manifest.permission.MOUNT_UNMOUNT_FILESYSTEMS,
                            Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.SYSTEM_ALERT_WINDOW,
                            Manifest.permission.READ_PHONE_STATE,
                    }
                    , 1);

        }

        setContentView(R.layout.activity_waypoint1);

        IntentFilter filter = new IntentFilter();
        filter.addAction(DJIDemoApplication.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        initUI();

        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        addListener();

        //createIsoDroneTask
        //Task droneTask = new Task();
        //droneTask.run();
//
    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            onProductConnectionChange();
        }
    };

    private void onProductConnectionChange()
    {
        initFlightController();
        loginAccount();
    }

    private void loginAccount(){

        UserAccountManager.getInstance().logIntoDJIUserAccount(this,
                new CommonCallbacks.CompletionCallbackWith<UserAccountState>() {
                    @Override
                    public void onSuccess(final UserAccountState userAccountState) {
                        Log.e(TAG, "Login Success");
                    }
                    @Override
                    public void onFailure(DJIError error) {
                        setResultToToast("Login Error:"
                                + error.getDescription());
                    }
                });
    }

    private void initFlightController() {

        BaseProduct product = DJIDemoApplication.getProductInstance();
        if (product != null && product.isConnected()) {
            if (product instanceof Aircraft) {
                mFlightController = ((Aircraft) product).getFlightController();
            }
        }

        if (mFlightController != null) {
            mFlightController.setStateCallback(new FlightControllerState.Callback() {

                @Override
                public void onUpdate(FlightControllerState djiFlightControllerCurrentState) {
                    droneLocationLat = djiFlightControllerCurrentState.getAircraftLocation().getLatitude();
                    droneLocationLng = djiFlightControllerCurrentState.getAircraftLocation().getLongitude();
                    droneAltitude = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();
                    droneHeading = djiFlightControllerCurrentState.getAttitude().yaw;
                    updateDroneLocationData();
                }
            });
        }
    }

    //Add Listener for WaypointMissionOperator
    private void addListener() {
        if (getWaypointMissionOperator() != null){
            getWaypointMissionOperator().addListener(eventNotificationListener);
        }
    }

    private void removeListener() {
        if (getWaypointMissionOperator() != null) {
            getWaypointMissionOperator().removeListener(eventNotificationListener);
        }
    }

    private WaypointMissionOperatorListener eventNotificationListener = new WaypointMissionOperatorListener() {
        @Override
        public void onDownloadUpdate(WaypointMissionDownloadEvent downloadEvent) {

        }

        @Override
        public void onUploadUpdate(WaypointMissionUploadEvent uploadEvent) {

        }

        @Override
        public void onExecutionUpdate(WaypointMissionExecutionEvent executionEvent) {

        }

        @Override
        public void onExecutionStart() {

        }

        @Override
        public void onExecutionFinish(@Nullable final DJIError error) {

        }
    };

    public WaypointMissionOperator getWaypointMissionOperator() {
        if (instance == null) {
            if (DJISDKManager.getInstance().getMissionControl() != null){
                instance = DJISDKManager.getInstance().getMissionControl().getWaypointMissionOperator();
            }
        }
        return instance;
    }

    private void setUpMap() {
        gMap.setOnMapClickListener(this);// add the listener for click for amap object
        gMap.setMapType(GoogleMap.MAP_TYPE_SATELLITE);

    }

    @Override
    public void onMapClick(LatLng point) {
        if (isAdd == true){
            markWaypoint(point);
            Waypoint mWaypoint = new Waypoint(point.latitude, point.longitude, altitude);
            //Add Waypoints to Waypoint arraylist;
            if (waypointMissionBuilder != null) {
                waypointList.add(mWaypoint);
                //waypointMissionBuilder.waypointList(waypointList).waypointCount(waypointList.size());
                waypointMissionBuilder.waypointCount(waypointList.size());
            }else
            {
                waypointMissionBuilder = new WaypointMission.Builder();
                waypointList.add(mWaypoint);
                //waypointMissionBuilder.waypointList(waypointList).waypointCount(waypointList.size());
                waypointMissionBuilder.waypointCount(waypointList.size());
            }
        }else{
            //setResultToToast("Cannot Add Waypoint");
        }
    }

    public static boolean checkGpsCoordination(double latitude, double longitude) {
        return (latitude > -90 && latitude < 90 && longitude > -180 && longitude < 180) && (latitude != 0f && longitude != 0f);
    }

    // Update the drone location based on states from MCU.
    private void updateDroneLocationData(){


        LatLng pos = new LatLng(droneLocationLat, droneLocationLng);
        //Create MarkerOptions object
        final MarkerOptions markerOptions = new MarkerOptions();
        markerOptions.position(pos);
        if (missionUploaded) {
            ProjCoordinate currentLocation = coordGeoToCart(startLatLong, new ProjCoordinate(droneLocationLat, droneLocationLng));

            try {
                //Mark the drone on map
                mRot = Math.acos(currentLocation.x / (Math.sqrt(Math.pow(currentLocation.x, 2) + Math.pow(currentLocation.y, 2))));
                if((currentLocation.x < 0 && currentLocation.y < 0) || (currentLocation.x > 0 && currentLocation.y < 0)) {
                    mRot = 2*Math.PI - mRot;
                }
                droneRot = mRot;
                mRot = rotateUnitCircleAngleToNorthHeadingRad(mRot);
                mRot = mRot * 180 / Math.PI;
                markerOptions.rotation((float)mFlightController.getState().getAttitude().yaw).icon(BitmapDescriptorFactory.fromResource(R.drawable.aircraft));;

                //Check waiting position, when reached wait for start/resume mission
                ProjCoordinate firstPoint = coordGeoToCart(startLatLong, new ProjCoordinate(waypointSettings.get(0).geo.y, waypointSettings.get(0).geo.x));
                Double radlim = 1.0;
                Double dist = Math.sqrt(Math.pow(currentLocation.x - firstPoint.x, 2) + Math.pow(currentLocation.y - firstPoint.y, 2));
                Double anglelim = 10.0;
                Double anglediff = Math.abs(droneHeading - waypointSettings.get(0).heading);

                Button button = (Button)findViewById(R.id.pauseresume);

                if (dist < radlim && anglediff < anglelim
                        && drone.getCurrentStateName().equals("Armed")){
                    //&& button.getText().equals("Arming")
                   pauseWaypointMission();
                   button.setText("Armed");

                }

                final StringBuffer positionStatusString = new StringBuffer();
                positionStatusString.append("x=" + String.format("%.3f", currentLocation.x) + " y=" + String.format("%.3f", currentLocation.y) + " z=" + String.format("%.2f", droneAltitude));
                //positionStatusString.append("x=" + pc.x);
                //positionStatusString.append(" y=" + pc.y);
                //positionStatusString.append(" z=" + pc.z);
                positionStatus.setText(positionStatusString);




            } catch (Exception e) {

            }
        } else markerOptions.icon(BitmapDescriptorFactory.fromResource(R.drawable.aircraft));


        if(lastDroneState == "") {
            drone = new IsoDrone("10.130.22.16");
            lastDroneState = drone.getCurrentStateName();
        }
        Log.wtf("Error", "Drone is in: " + drone.getCurrentStateName());
        //SetMonr data
        //Log.wtf("Lat: ", String.valueOf(drone.getOrigin().getLatitude_deg()));
        //Log.wtf("Log: ", String.valueOf(drone.getOrigin().getLongitude_deg()));
        //Log.wtf("alt: ", String.valueOf(drone.getOrigin().getAltitude_m()));

        if (drone.getCurrentStateName().equals("Armed") || (drone.getCurrentStateName().equals("Running"))) {
            sendMonr();
        }

        if (drone.getCurrentStateName().equals("Init") && lastDroneState != "Init") {
            Log.wtf("Error", "Init");

            lastDroneState = "Init";


        } else if (drone.getCurrentStateName().equals("PreArming") && lastDroneState != "PreArming") {
            Log.wtf("Error", "PreArming");
            lastDroneState = "PreArming";
        }else if (drone.getCurrentStateName().equals("Disarmed")) {
                Log.wtf("Error", "Disarmed");
                lastDroneState = "Disarmed";
                sendMonr();


        } else if (drone.getCurrentStateName().equals("Armed") && lastDroneState != "Armed") {
            //button = (Button)findViewById(R.id.pauseresume);
            //button.setText("Arming");


            //trajectory
            runOnUiThread(new Runnable() {
                @Override
                public void run() {

                    EditText distance   = (EditText)findViewById(R.id.reduction);
                    drone.setMaxDistancePoints(Double.parseDouble(distance.getText().toString()));

                    Log.wtf("TrajName: ", drone.getTrajectoryHeader().getTrajectoryName());
                    TrajectoryWaypointVector traj =  drone.getTrajectory();
                    waypointSettings.clear();

                    //Reduce points in traj if to large (99 max amount of waypoints)
                        if(traj.size() > 9999){
                        double epsilon = 0.001;
                        do{
                            drone.reducePoints(epsilon);
                            epsilon += 0.001;
                            setResultToToast("reducing traj");
                        }while (drone.getReducedTraj().size() > 99 && epsilon < 0.06);
                        Log.wtf("newTraj", String.valueOf(drone.getReducedTraj().size()));
                        waypointSettings.clear();
                            setResultToToast("peucker Traj: " + String.valueOf(drone.getReducedTraj().size()));
                            drone.removePointsToClose();
                            setResultToToast("remove points to close: " + String.valueOf(drone.getReducedTraj().size()));
                            generateWaypointsFromTraj(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), drone.getReducedTraj()); //Use reduced
                    }else{
                        waypointSettings.clear();
                            drone.reducedTraj = drone.getTrajectory(); //Put the non douglas-peucker:ed traj in
                            setResultToToast("non reduced Traj: " + String.valueOf(drone.getReducedTraj().size()));
                        drone.removePointsToClose();
                        setResultToToast("remove points to close: " + String.valueOf(drone.getReducedTraj().size()));
                        generateWaypointsFromTraj(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), drone.reducedTraj); // use set traj
                    }

                    //generateTestCircleCoordinates(new LatLng(droneLocationLat, droneLocationLng), 10, 3, 1,19, true);

                    for(int i = 0; i < waypointSettings.size(); i ++){
                        LatLng mpoint = new LatLng(waypointSettings.get(i).geo.y, waypointSettings.get(i).geo.x);
                        markWaypoint(mpoint);
                    }
//                    deployTestCircle();
                    deployTraj();
                }
            });

            Log.wtf("Error", "Armed");
            lastDroneState = "Armed";
        } else if (drone.getCurrentStateName().equals("Disarmed") && lastDroneState != "Disarmed") {

            Log.wtf("Error", "Disarmed");
            lastDroneState = "Disarmed";
        } else if (drone.getCurrentStateName().equals("PreRunning") && lastDroneState != "PreRunning") {
            Log.wtf("Error", "PreRunning");
            lastDroneState = "PreRunning";
        } else if (drone.getCurrentStateName().equals("Running") && lastDroneState != "Running") {

            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    resumeWaypointMission();
                }});
            Log.wtf("Error", "Running");
            lastDroneState = "Running";
        } else if (drone.getCurrentStateName().equals("NormalStop") && lastDroneState != "NormalStop") {
            setResultToToast("NormalStop");
            lastDroneState = "NormalStop";
        } else if (drone.getCurrentStateName().equals("EmergencyStop") && lastDroneState != "EmergencyStop") {
            Log.wtf("Error", "EmergencyStop");
            lastDroneState = "EmergencyStop";
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    stopWaypointMission();
                }});

        }

        runOnUiThread(new Runnable() {
            @Override
            public void run() {




                if (droneMarker != null) {
                    droneMarker.remove();
                }

                if (checkGpsCoordination(droneLocationLat, droneLocationLng)) {
                    droneMarker = gMap.addMarker(markerOptions);
                }
             }
        });



    }

    private void markWaypoint(LatLng point){
        //Create MarkerOptions object
        MarkerOptions markerOptions = new MarkerOptions();
        markerOptions.position(point);
        markerOptions.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_BLUE));
        Marker marker = gMap.addMarker(markerOptions);
        mMarkers.put(mMarkers.size(), marker);
    }

    private void deployTestCircle(){

        Log.wtf("Error", "Deploying circle");

        waypointMissionBuilder = new WaypointMission.Builder();
        for (int i = 0; i < this.waypointSettings.size(); i ++)
        {
            Waypoint wp = new Waypoint();
            wp.coordinate = new LocationCoordinate2D(this.waypointSettings.get(i).geo.y, this.waypointSettings.get(i).geo.x);
            wp.altitude = (float)this.waypointSettings.get(i).geo.z + i*0.2f;
            wp.speed = (float)this.waypointSettings.get(i).speed + i*0.1f;
            try {
                wp.heading = this.waypointSettings.get(i).heading;
            } catch (Exception e){
                setResultToToast("e = "+ e.getCause() + ", " + e.getMessage());
            }
            //setResultToToast("wp.heading = "+ waypointSettings.get(i).geo.y + ", " + waypointSettings.get(i).geo.x + ", " + waypointSettings.get(i).geo.z);
            waypointList.add(wp);
            waypointMissionBuilder.waypointList(waypointList).waypointCount(waypointList.size());
        }

        Log.wtf("Error", "Number of waypoints " + this.waypointSettings.size());

        setResultToToast("Number of waypoints " + this.waypointSettings.size());
        mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
        //mSpeed = 5.0f;
        mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;
        altitude = (float)this.waypointSettings.get(0).geo.z;
        configWayPointMission();
        startLatLong = new LatLng(droneLocationLat, droneLocationLng);




    }


    private void deployTraj(){

        Log.wtf("Error", "Deploying traj");

        //Get altitude from GUI
        EditText turnRad   = (EditText)findViewById(R.id.turnRad);

        waypointMissionBuilder = new WaypointMission.Builder();
        for (int i = 0; i < this.waypointSettings.size(); i ++)
        {
            Waypoint wp = new Waypoint();
            wp.coordinate = new LocationCoordinate2D(this.waypointSettings.get(i).geo.y, this.waypointSettings.get(i).geo.x);
            wp.altitude = (float)this.waypointSettings.get(i).geo.z;
            wp.speed = (float)this.waypointSettings.get(i).speed;
            wp.cornerRadiusInMeters = Float.parseFloat(turnRad.getText().toString()); //(float)this.waypointSettings.get(i).radius;

            if (i+1 < this.waypointSettings.size()) {
                int angular_velocity = this.waypointSettings.get(i+1).heading - this.waypointSettings.get(i).heading; // Ugly approximation of velocity
                wp.turnMode = angular_velocity > 0 ? WaypointTurnMode.CLOCKWISE : WaypointTurnMode.COUNTER_CLOCKWISE;
            }

            try {
                wp.heading = this.waypointSettings.get(i).heading;
            } catch (Exception e){
                setResultToToast("e = "+ e.getCause() + ", " + e.getMessage());
            }
            //setResultToToast("wp.heading = "+ waypointSettings.get(i).geo.y + ", " + waypointSettings.get(i).geo.x + ", " + waypointSettings.get(i).geo.z);
            waypointList.add(wp);
            waypointMissionBuilder.waypointList(waypointList).waypointCount(waypointList.size());
        }

        Log.wtf("Error", "Number of waypoints " + this.waypointSettings.size());

        setResultToToast("Number of waypoints " + this.waypointSettings.size());
        mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
        //mSpeed = 5.0f;
        mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;
        altitude = (float)this.waypointSettings.get(0).geo.z;
        configWayPointMission();
        startLatLong = new LatLng(droneLocationLat, droneLocationLng);
        uploadWayPointMission();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.locate:{
                updateDroneLocationData();
                cameraUpdate(); // Locate the drone's place
                break;
            }
            case R.id.add:{
                break;
            }
            case R.id.clear: {
                setResultToToast("Clear map!");
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        gMap.clear();
                    }
                });
                waypointList.clear();
                waypointMissionBuilder.waypointCount(0);
                updateDroneLocationData();
                Button button = (Button)findViewById(R.id.pauseresume);
                button.setText("Pause");
                break;
            }
            case R.id.testcircle:{
                this.waypointSettings.clear();
                generateTestCircleCoordinates(new LatLng(droneLocationLat, droneLocationLng), 10, 3, 1,19, true);
                for(int i = 0; i < this.waypointSettings.size(); i ++){
                    LatLng mpoint = new LatLng(this.waypointSettings.get(i).geo.y, this.waypointSettings.get(i).geo.x);
                    markWaypoint(mpoint);
                }
                deployTestCircle();
                break;
            }
            case R.id.pauseresume:{
                Button button = (Button)findViewById(R.id.pauseresume);
                if(button.getText().equals("Pause")){
                    pauseWaypointMission();
                    button.setText("Resume");
                } else if(button.getText().equals("Resume")){
                    resumeWaypointMission();
                    button.setText("Pause");
                } else if(button.getText().equals("Armed")){
                    resumeWaypointMission();
                    button.setText("Running");
                }
                break;
            }
            case R.id.arm:{
                startWaypointMission();
                Button button = (Button)findViewById(R.id.pauseresume);
                button.setText("Arming");
                break;
            }
            case R.id.start:{
                resumeWaypointMission();
                break;
            }
            case R.id.stop:{
                stopWaypointMission();
                break;
            }
            default:
                break;
        }
    }

    private void cameraUpdate(){
        LatLng pos = new LatLng(droneLocationLat, droneLocationLng);
        float zoomlevel = (float) 22.0;
        CameraUpdate cu = CameraUpdateFactory.newLatLngZoom(pos, zoomlevel);
        gMap.moveCamera(cu);

    }

    private void enableDisableAdd(){
        if (isAdd == false) {
            isAdd = true;
            add.setText("Exit");
        }else{
            isAdd = false;
            add.setText("Add");
        }
    }

    private void showSettingDialog(){
        LinearLayout wayPointSettings = (LinearLayout)getLayoutInflater().inflate(R.layout.dialog_waypointsetting, null);

        final TextView wpAltitude_TV = (TextView) wayPointSettings.findViewById(R.id.altitude);
        RadioGroup speed_RG = (RadioGroup) wayPointSettings.findViewById(R.id.speed);
        RadioGroup actionAfterFinished_RG = (RadioGroup) wayPointSettings.findViewById(R.id.actionAfterFinished);
        RadioGroup heading_RG = (RadioGroup) wayPointSettings.findViewById(R.id.heading);

        speed_RG.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener(){

            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                if (checkedId == R.id.lowSpeed){
                    mSpeed = 3.0f;
                } else if (checkedId == R.id.MidSpeed){
                    mSpeed = 5.0f;
                } else if (checkedId == R.id.HighSpeed){
                    mSpeed = 10.0f;
                }
            }

        });

        actionAfterFinished_RG.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {

            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                Log.d(TAG, "Select finish action");
                if (checkedId == R.id.finishNone){
                    mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
                } else if (checkedId == R.id.finishGoHome){
                    mFinishedAction = WaypointMissionFinishedAction.GO_HOME;
                } else if (checkedId == R.id.finishAutoLanding){
                    mFinishedAction = WaypointMissionFinishedAction.AUTO_LAND;
                } else if (checkedId == R.id.finishToFirst){
                    mFinishedAction = WaypointMissionFinishedAction.GO_FIRST_WAYPOINT;
                }
            }
        });

        heading_RG.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {

            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                Log.d(TAG, "Select heading");

                if (checkedId == R.id.headingNext) {
                    mHeadingMode = WaypointMissionHeadingMode.AUTO;
                } else if (checkedId == R.id.headingInitDirec) {
                    mHeadingMode = WaypointMissionHeadingMode.USING_INITIAL_DIRECTION;
                } else if (checkedId == R.id.headingRC) {
                    mHeadingMode = WaypointMissionHeadingMode.CONTROL_BY_REMOTE_CONTROLLER;
                } else if (checkedId == R.id.headingWP) {
                    mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;
                }
            }
        });

        new AlertDialog.Builder(this)
                .setTitle("")
                .setView(wayPointSettings)
                .setPositiveButton("Finish",new DialogInterface.OnClickListener(){
                    public void onClick(DialogInterface dialog, int id) {

                        String altitudeString = wpAltitude_TV.getText().toString();
                        altitude = Integer.parseInt(nulltoIntegerDefalt(altitudeString));
                        Log.e(TAG,"altitude "+altitude);
                        Log.e(TAG,"speed "+mSpeed);
                        Log.e(TAG, "mFinishedAction "+mFinishedAction);
                        Log.e(TAG, "mHeadingMode "+mHeadingMode);
                        configWayPointMission();
                    }

                })
                .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        dialog.cancel();
                    }

                })
                .create()
                .show();
    }

    String nulltoIntegerDefalt(String value){
        if(!isIntValue(value)) value="0";
        return value;
    }

    boolean isIntValue(String val)
    {
        try {
            val=val.replace(" ","");
            Integer.parseInt(val);
        } catch (Exception e) {return false;}
        return true;
    }

    private void configWayPointMission(){

        if (waypointMissionBuilder == null){

            waypointMissionBuilder = new WaypointMission.Builder().finishedAction(mFinishedAction)
                    .headingMode(mHeadingMode)
                    .autoFlightSpeed(mSpeed)
                    .maxFlightSpeed(mSpeed)
                    .flightPathMode(WaypointMissionFlightPathMode.CURVED);

        }else
        {
            waypointMissionBuilder.finishedAction(mFinishedAction)
                    .headingMode(mHeadingMode)
                    .autoFlightSpeed(mSpeed)
                    .maxFlightSpeed(mSpeed)
                    .flightPathMode(WaypointMissionFlightPathMode.CURVED);

        }

        if (waypointMissionBuilder.getWaypointList().size() > 0){

           // for (int i=0; i< waypointMissionBuilder.getWaypointList().size(); i++){
            //    waypointMissionBuilder.getWaypointList().get(i).altitude = altitude;
           // }

           // setResultToToast("Set Waypoint altitude successfully");
        }

        DJIError error = getWaypointMissionOperator().loadMission(waypointMissionBuilder.build());
        if (error == null) {
            setResultToToast("loadWaypoint succeeded");
        } else {
            setResultToToast("loadWaypoint failed " + error.getDescription() + " " + error.toString());
        }
    }

    private void uploadWayPointMission(){

        setResultToToast("Waypoint operator state is: " + getWaypointMissionOperator().getCurrentState());

        getWaypointMissionOperator().uploadMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                if (error == null) {
                    setResultToToast("Mission upload successfully!");
                    missionUploaded = true;
                    //startWaypointMission();

                } else {
                    missionUploaded = false;
                    getWaypointMissionOperator().retryUploadMission(null);
                }
            }
        });

    }

    private void startWaypointMission(){

        getWaypointMissionOperator().startMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Mission Start: " + (error == null ? "Successfully" : error.getDescription()));
            }
        });
    }

    private void pauseWaypointMission(){

        getWaypointMissionOperator().pauseMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Mission paused: " + (error == null ? "Successfully" : error.getDescription()));
            }
        });
    }

    private void resumeWaypointMission(){

        getWaypointMissionOperator().resumeMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Mission resumed: " + (error == null ? "Successfully" : error.getDescription()));
            }
        });
    }

    private void stopWaypointMission(){

        getWaypointMissionOperator().stopMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Mission Stop: " + (error == null ? "Successfully" : error.getDescription()));
            }
        });

    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        if (gMap == null) {
            gMap = googleMap;
            setUpMap();
        }

        //LatLng shenzhen = new LatLng(22.5362, 113.9454);
        LatLng astazero = new LatLng(57.777521, 12.781442);
        gMap.addMarker(new MarkerOptions().position(astazero).title("Marker at AstaZero"));
        cameraUpdate(); // Locate the drone's place
    }

    public void sendMonr(){
        double isoLat = drone.getOrigin().getLatitude_deg();
        double isoLog = drone.getOrigin().getLongitude_deg();

        LatLng isoOrigin = new LatLng(isoLat, isoLog);

        ProjCoordinate dronePosition = new ProjCoordinate(droneLocationLat, droneLocationLng);
        ProjCoordinate result = coordGeoToCart(isoOrigin, dronePosition);

        CartesianPosition monrPos = new CartesianPosition();
        monrPos.setXCoord_m(result.x);
        monrPos.setYCoord_m(result.y);
        monrPos.setZCoord_m(droneAltitude);
        monrPos.setIsXcoordValid(true);
        monrPos.setIsYcoordValid(true);
        monrPos.setIsZcoordValid(true);
        monrPos.setIsPositionValid(true);

        monrPos.setHeading_rad(headingToYaw(mFlightController.getState().getAttitude().yaw) * Math.PI/180);
        monrPos.setIsHeadingValid(true);
        drone.setPosition(monrPos);

    }

private void setUpListener() {
        listener = new WaypointMissionOperatorListener() {
            @Override
            public void onDownloadUpdate(@NonNull WaypointMissionDownloadEvent waypointMissionDownloadEvent) {
                if (waypointMissionDownloadEvent.getProgress() != null
                        && waypointMissionDownloadEvent.getProgress().isSummaryDownloaded
                        && waypointMissionDownloadEvent.getProgress().downloadedWaypointIndex == (getWaypointMissionOperator().getLoadedMission().getWaypointCount() - 1)) {
                    setResultToToast("Mission is downloaded successfully");
                }
            }

            @Override
            public void onUploadUpdate(@NonNull WaypointMissionUploadEvent waypointMissionUploadEvent) {
                if (waypointMissionUploadEvent.getProgress() != null
                        && waypointMissionUploadEvent.getProgress().isSummaryUploaded
                        && waypointMissionUploadEvent.getProgress().uploadedWaypointIndex == (getWaypointMissionOperator().getLoadedMission().getWaypointCount() - 1)) {
                    setResultToToast("Mission is uploaded successfully");
                    startWaypointMission();
                }
            }

            @Override
            public void onExecutionUpdate(@NonNull WaypointMissionExecutionEvent waypointMissionExecutionEvent) {
                if (getWaypointMissionOperator().getCurrentState().equals(READY_TO_UPLOAD)
                        ||   getWaypointMissionOperator().getCurrentState().equals(READY_TO_RETRY_UPLOAD)){
                    uploadWayPointMission();
                }
                if(waypointMissionExecutionEvent.getCurrentState().equals(EXECUTION_PAUSED)){
                    //setResultToToast("Mission is Paused successfully");
                }
                if(waypointMissionExecutionEvent.getCurrentState().equals(EXECUTING)){
                    //setResultToToast("Mission is Running successfully");
                }
            }

            @Override
            public void onExecutionStart() {
               setResultToToast("Mission started");
            }

            @Override
            public void onExecutionFinish(@Nullable DJIError djiError) {
                setResultToToast("Execution finished: " + (djiError == null ? "Success!" : djiError.getDescription()));
                drone.setObjectState(ObjectStateID.ISO_OBJECT_STATE_DISARMED);
                
                setResultToToast("Drone is in: " + drone.getCurrentStateName());

            }
        };

        if (getWaypointMissionOperator() != null && listener != null) {
            // Example of adding listeners
            getWaypointMissionOperator().addListener(listener);
        }
    }

    private void tearDownListener() {
        if (getWaypointMissionOperator() != null && listener != null) {
            getWaypointMissionOperator().removeListener(listener);
        }
    }

    }



