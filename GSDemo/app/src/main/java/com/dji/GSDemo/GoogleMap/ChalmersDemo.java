package com.dji.GSDemo.GoogleMap;


import android.Manifest;
import android.app.ActionBar;
import android.content.IntentFilter;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;


import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import org.asta.isoObject.CartesianPosition;
import org.asta.isoObject.TrajectoryWaypointVector;
import org.locationtech.proj4j.CRSFactory;
import org.locationtech.proj4j.CoordinateReferenceSystem;
import org.locationtech.proj4j.CoordinateTransform;
import org.locationtech.proj4j.CoordinateTransformFactory;
import org.locationtech.proj4j.ProjCoordinate;

import java.util.ArrayList;
import java.net.ResponseCache;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.GPSSignalLevel;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.model.LocationCoordinate2D;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.midware.data.model.P3.B;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;

public class ChalmersDemo extends FragmentActivity implements TextureView.SurfaceTextureListener, View.OnClickListener {

    private static final String TAG = MainActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    TextureView mVideoSurface;
    Timer mGimbalTaskTimer;
    View decorView;
    private FlightController flightController;
    private FlightControllerState djiFlightControllerCurrentState;
    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;
    private String lastDroneState = "";
    private IsoDrone drone;
    private double droneLocationLat = 57.688859d, droneLocationLng = 11.978795d, droneAltitude = 0d; // Johanneberg
    private Gimbal gimbal;
    private Button btn_atos_con, btn_drone_con, btn_ip_address, btn_drone_state, clear_wps;

    private TextView text_gps, text_lat, text_lon;
    private CommonCallbacks.CompletionCallback callback;
    private float mPitch;
    private float mRoll;
    private float mYaw;
    private float mThrottle;
    private DJICodecManager mCodecManager;
    private float mGimbalPitch;
    private float mGimbalRoll;
    private float mGimbalYaw;

    private Button testcircle, config, upload, start, stop;
    private float mSpeed = 10.0f;
    private float altitude = 100.0f;
    private LatLng startLatLong;
    public CRSFactory crsFactory = new CRSFactory();
    public CoordinateReferenceSystem WGS84 = crsFactory.createFromParameters("WGS84","+proj=longlat +datum=WGS84 +no_defs");

    public static WaypointMission.Builder waypointMissionBuilder;
    private ArrayList<WaypointSetting> waypointSettings = new ArrayList<>();
    private ArrayList<Waypoint> waypointList = new ArrayList<>();

    private WaypointMissionOperator instance;
    private WaypointMissionFinishedAction mFinishedAction = WaypointMissionFinishedAction.AUTO_LAND;
    private WaypointMissionHeadingMode mHeadingMode = WaypointMissionHeadingMode.AUTO;
    private boolean missionUploaded = false;

    public double getDroneLocationLat() {
        return droneLocationLat;
    }

    public double getDroneLocationLng() {
        return droneLocationLng;
    }

    @Override
    protected void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initPreviewer();
        onProductChange();

        if (mVideoSurface == null) {
            Log.e(TAG, "mVideoSurface is null");
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onDestroy() {
//        unregisterReceiver(mReceiver);
//        removeListener();
        super.onDestroy();
    }

    @Override
    public void onStop() {
        super.onStop();
    }

    public void onReturn(View view) {
        this.finish();
    }


    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.testcircle:{
                this.waypointSettings.clear();
                generateTestCircleCoordinates(new LatLng(droneLocationLat, droneLocationLng), 1, 1.5f, 1,8, true);
                deployTraj();
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
            case R.id.clear_wps:{
                //Add land
//                Button button = (Button)findViewById(R.id.clear_wps);
                waypointList.clear();
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


    public void landDrone() {

    }

    //    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureAvailable");
        if (mCodecManager == null) {
            mCodecManager = new DJICodecManager(this, surface, width, height);
        }
    }

    //    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureSizeChanged");
    }

    //    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {

        Log.e(TAG, "onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }

        return false;
    }

    //    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
    }


    private void initUI() {

        btn_atos_con = (Button) findViewById(R.id.btn_atos_con);
        btn_drone_con = (Button) findViewById(R.id.btn_drone_con);
        btn_drone_state = (Button)  findViewById(R.id.btn_drone_state);
        btn_ip_address = (Button) findViewById(R.id.btn_ip_address);

        btn_ip_address.setText(Utils.getIPAddress(true));
        btn_drone_state.setText("State: Undefined");

        text_gps = (TextView) findViewById(R.id.text_gps);
        text_lat = (TextView) findViewById(R.id.text_lat);
        text_lon = (TextView) findViewById(R.id.text_lon);

        testcircle = (Button) findViewById(R.id.testcircle);
        config = (Button) findViewById(R.id.pauseresume);
        clear_wps = (Button) findViewById(R.id.clear_wps);
        start = (Button) findViewById(R.id.start);
        stop = (Button) findViewById(R.id.stop);

        testcircle.setOnClickListener(this);
        config.setOnClickListener(this);
        clear_wps.setOnClickListener(this);
        start.setOnClickListener(this);
        stop.setOnClickListener(this);

        mVideoSurface = (TextureView) findViewById(R.id.video_preview_surface);
        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);


        }
    }

    private void changeGimbalAngles(float pitch, float yaw, float roll) {
        if (gimbal == null) return;
        Log.wtf("GIMBAL", "Changing gimbal angle...");

        gimbal.rotate(new Rotation.Builder()
                        .pitch(pitch)
                        .yaw(yaw)
                        .roll(roll)
                        .time(0.05)
                        .mode(RotationMode.RELATIVE_ANGLE)
                        .build()
                , new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        if (djiError == null) {
                            Log.d("GIMBAL", "rotate gimbal success");
//                    showToast("rotate gimbal success");
                        } else {
                            Log.d("GIMBAL", "rotate gimbal error " + djiError.getDescription());
//                    showToast(djiError.getDescription());
                        }
                    }
                });
    }

    private void setResultToToast(final String string) {
        ChalmersDemo.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(ChalmersDemo.this, string, Toast.LENGTH_SHORT).show();
            }
        });
    }


    private void initFlightController() {

        BaseProduct product = DJIDemoApplication.getProductInstance();
        if (product != null && product.isConnected()) {
            if (product instanceof Aircraft) {
                flightController = ((Aircraft) product).getFlightController();
            }
        }

        if (flightController != null) {
            flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            flightController.setStateCallback(new FlightControllerState.Callback() {

                @Override
                public void onUpdate(FlightControllerState djiFlightControllerCurrentState) {
                    GPSSignalLevel gps = djiFlightControllerCurrentState.getGPSSignalLevel();
                    droneLocationLat = djiFlightControllerCurrentState.getAircraftLocation().getLatitude();
                    droneLocationLng = djiFlightControllerCurrentState.getAircraftLocation().getLongitude();
                    droneAltitude = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();


//                    text_gps.setText("GPS Level: " + gps.toString());
//                    text_lat.setText("LAT: " + droneLocationLat);
//                    text_lon.setText("LON: " + droneLocationLng);

                    Log.wtf("LOCATION GPS", String.valueOf(gps));
                    Log.wtf("LOCATION LAT", String.valueOf(droneLocationLat));
                    Log.wtf("LOCATION LONG", String.valueOf(droneLocationLng));
                    Log.wtf("LOCATION ALT", String.valueOf(droneAltitude));
                    updateDroneLocationData();
                }
            });
        }
    }

    private void initCameraGimbal() {
        BaseProduct product = DJIDemoApplication.getProductInstance();
        if (product != null && product.isConnected()) {
            if (product instanceof Aircraft) {
                gimbal = ((Aircraft) product).getGimbal();
            }
        }
//        Gimbal gimbal = DJIDemoApplication.getProductInstance().getGimbal();
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        decorView = getWindow().getDecorView();
        decorView.setOnSystemUiVisibilityChangeListener
                (new View.OnSystemUiVisibilityChangeListener() {
                    @Override
                    public void onSystemUiVisibilityChange(int visibility) {
                        if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
                            int uiOptions = View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION;
                            decorView.setSystemUiVisibility(uiOptions);
                        }
                    }
                });

        initFlightController();
        initCameraGimbal();

        // When the compile and target version is higher than 22, please request the
        // following permissions at runtime to ensure the
        // SDK work well.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this,
                    new String[]{android.Manifest.permission.WRITE_EXTERNAL_STORAGE, android.Manifest.permission.VIBRATE,
                            android.Manifest.permission.INTERNET, android.Manifest.permission.ACCESS_WIFI_STATE,
                            android.Manifest.permission.WAKE_LOCK, android.Manifest.permission.ACCESS_COARSE_LOCATION,
                            android.Manifest.permission.ACCESS_NETWORK_STATE, android.Manifest.permission.ACCESS_FINE_LOCATION,
                            android.Manifest.permission.CHANGE_WIFI_STATE, android.Manifest.permission.MOUNT_UNMOUNT_FILESYSTEMS,
                            android.Manifest.permission.READ_EXTERNAL_STORAGE, android.Manifest.permission.SYSTEM_ALERT_WINDOW,
                            Manifest.permission.READ_PHONE_STATE,
                    }
                    , 1);

        }

        setContentView(R.layout.activity_chalmers_demo);

        IntentFilter filter = new IntentFilter();
        filter.addAction(DJIDemoApplication.FLAG_CONNECTION_CHANGE);
//        registerReceiver(mReceiver, filter);

        initUI();

        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if (mCodecManager != null) {
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };
    }

    protected void onProductChange() {
        initPreviewer();
    }

    //For camera feed
    private void initPreviewer() {

        BaseProduct product = DJIDemoApplication.getProductInstance();

        if (product == null || !product.isConnected()) {
            btn_drone_con.setBackgroundColor(Color.RED);
            setResultToToast(getString(R.string.disconnected));
        } else {
            btn_drone_con.setBackgroundColor(Color.GREEN);
            if (null != mVideoSurface) {
                mVideoSurface.setSurfaceTextureListener(this);
            }
            if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
            }
        }
    }

    //For camera feed
    private void uninitPreviewer() {
        Camera camera = DJIDemoApplication.getProductInstance().getCamera();
        if (camera != null) {
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }



    private void updateDroneLocationData() {

        if (lastDroneState == "") {
            drone = new IsoDrone("192.168.107.229");
            lastDroneState = drone.getCurrentStateName();
        }
        Log.wtf("Error", "Drone is in: " + drone.getCurrentStateName());
        //SetMonr data
        //Log.wtf("Lat: ", String.valueOf(drone.getOrigin().getLatitude_deg()));
        //Log.wtf("Log: ", String.valueOf(drone.getOrigin().getLongitude_deg()));
        //Log.wtf("alt: ", String.valueOf(drone.getOrigin().getAltitude_m()));

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
            Log.wtf(TAG, droneStart.toString());
            ProjCoordinate pc = coordGeoToCart(droneStart, dronePosition);
            double heading = Math.acos(pc.x / (Math.sqrt(Math.pow(pc.x, 2) + Math.pow(pc.y, 2))));
            monrPos.setHeading_rad(heading);
            monrPos.setIsHeadingValid(true);

            drone.setPosition(monrPos);
        }

        if (drone.getCurrentStateName().equals("Init") && lastDroneState != "Init") {
            Log.wtf("Error", "Init");
            lastDroneState = "Init";
            updateStateButton(lastDroneState, Color.LTGRAY);
        } else if (drone.getCurrentStateName().equals("PreArming") && lastDroneState != "PreArming") {
            Log.wtf("Error", "PreArming");
            lastDroneState = "PreArming";
            updateStateButton(lastDroneState, Color.GRAY);

        } else if (drone.getCurrentStateName().equals("Armed") && lastDroneState != "Armed") {
            Log.wtf("Error", "Armed");
            lastDroneState = "Armed";
        } else if (drone.getCurrentStateName().equals("Disarmed") && lastDroneState != "Disarmed") {

            Log.wtf("Error", "Disarmed");
            lastDroneState = "Disarmed";
            updateStateButton(lastDroneState, Color.YELLOW);
        } else if (drone.getCurrentStateName().equals("PreRunning") && lastDroneState != "PreRunning") {
            Log.wtf("Error", "PreRunning");
            lastDroneState = "PreRunning";

            updateStateButton(lastDroneState, Color.DKGRAY);
        } else if (drone.getCurrentStateName().equals("Running") && lastDroneState != "Running") {
            Log.wtf("Error", "Running");
            lastDroneState = "Running";
            updateStateButton(lastDroneState, Color.RED);
        } else if (drone.getCurrentStateName().equals("NormalStop") && lastDroneState != "NormalStop") {
            setResultToToast("NormalStop");
            lastDroneState = "NormalStop";
            updateStateButton(lastDroneState, Color.CYAN);
        } else if (drone.getCurrentStateName().equals("EmergencyStop") && lastDroneState != "EmergencyStop") {
            Log.wtf("Error", "EmergencyStop");
            lastDroneState = "EmergencyStop";
            updateStateButton(lastDroneState, Color.BLACK);
        }
    }

    private void updateStateButton(String state, Integer color) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if(btn_drone_state != null) {
                    btn_drone_state.setText("STATE: " + state);
                    btn_drone_state.setBackgroundColor(color);
                }
            }
        });
    }
    class SendVirtualStickDataTask extends TimerTask {
        @Override
        public void run() {
            if (flightController != null) {
                flightController.sendVirtualStickFlightControlData(
                        new FlightControlData(
                                mPitch, mRoll, mYaw, mThrottle
                        ), new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if (djiError != null) {
                                    Log.wtf("Virtual Stick Error", (djiError.getDescription()));
                                    setResultToToast("Virtual Stick Error: " + djiError.getDescription());
                                } else {
                                    Log.wtf("Virtual Stick", "Sent Virtual stick data to flightcontroler");
                                    setResultToToast("Successfully sent virtual stick data to flightcontroller.");
                                }
                            }
                        }
                );
            }
        }
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

    private void deployTraj(){
        Log.wtf("Error", "Deploying traj");
        waypointMissionBuilder = new WaypointMission.Builder();

        for (int point = 0; point < this.waypointSettings.size(); point++){
            Waypoint wp = new Waypoint();
            wp.coordinate = new LocationCoordinate2D(this.waypointSettings.get(point).geo.y, this.waypointSettings.get(point).geo.x);
            wp.altitude = (float)this.waypointSettings.get(point).geo.z;
            wp.speed = (float)this.waypointSettings.get(point).speed;
            try {
                wp.heading = this.waypointSettings.get(point).heading;
            } catch (Exception e){
                setResultToToast("e = " + e.getCause() + ", " + e.getMessage());
            }
            waypointList.add(wp);
            waypointMissionBuilder.waypointList(waypointList).waypointCount(waypointList.size());
        }
        setResultToToast("Number of waypoints " + this.waypointSettings.size());
        mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
        mSpeed = 5.0f;
        mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;
        altitude = (float)this.waypointSettings.get(0).geo.z;
        configWayPointMission();
        startLatLong = new LatLng(droneLocationLat, droneLocationLng);
        uploadWayPointMission();
    }


    class ChangeGimbalTask extends TimerTask {
        @Override
        public void run() {
            gimbal.rotate(new Rotation.Builder()
                            .pitch(mGimbalPitch)
                            .yaw(mGimbalYaw)
                            .roll(mGimbalRoll)
                            .time(0)
                            .mode(RotationMode.RELATIVE_ANGLE)
                            .build()
                    , new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError == null) {
                                Log.d("GIMBAL", "rotate gimbal success");
                                setResultToToast("rotate gimbal success");
                            } else {
                                Log.d("GIMBAL", "rotate gimbal error " + djiError.getDescription());
                                setResultToToast(djiError.getDescription());
                            }
                        }
                    });
        }
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
            wpHeading = convertToDroneYawRangeDeg((180/Math.PI)*currentAngleRot);

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

    private int convertToDroneYawRangeDeg(double yaw) {
        int droneYaw = 0;
        if (yaw >= 0 && yaw <= 90) droneYaw = 90 - (int) (yaw);
        else if (yaw > 90 && yaw < 270) droneYaw = (90 - (int) (yaw));
        else if (yaw >= 270) droneYaw = (450 - (int) (yaw));
        return droneYaw;
    }

    public WaypointMissionOperator getWaypointMissionOperator() {
        if (instance == null) {
            if (DJISDKManager.getInstance().getMissionControl() != null){
                instance = DJISDKManager.getInstance().getMissionControl().getWaypointMissionOperator();
            }
        } return instance;
    }

    private void configWayPointMission() {
            if (waypointMissionBuilder == null) {
                waypointMissionBuilder = new WaypointMission.Builder().finishedAction(mFinishedAction)
                        .headingMode(mHeadingMode)
                        .autoFlightSpeed(mSpeed)
                        .maxFlightSpeed(mSpeed)
                        .flightPathMode(WaypointMissionFlightPathMode.CURVED);
            } else {
                waypointMissionBuilder.finishedAction(mFinishedAction)
                        .headingMode(mHeadingMode)
                        .autoFlightSpeed(mSpeed)
                        .maxFlightSpeed(mSpeed)
                        .flightPathMode(WaypointMissionFlightPathMode.CURVED);
            }


        DJIError error = getWaypointMissionOperator().loadMission(waypointMissionBuilder.build());
        if (error == null) {
            setResultToToast("loadWaypoint succeeded");
        } else {
            setResultToToast("loadWaypoint failed " + error.getDescription() + " " + error.toString());
        }
    }

    private void uploadWayPointMission(){
        getWaypointMissionOperator().uploadMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                if (error == null) {
                    setResultToToast("Mission upload successfully!");
                    missionUploaded = true;
                    startWaypointMission();

                } else {
                    missionUploaded = false;
                    setResultToToast("Mission upload failed, error: " + error.getDescription() + " retrying...");
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

//    private String buildOriginProjString(double latitude, double longitude){
//        final StringBuffer stringBuffer = new StringBuffer();
//        stringBuffer.append("+proj=tmerc +lat_0=" + latitude + " +lon_0=" + longitude + " +k=0.9996 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs");
//        return stringBuffer.toString();
//    }



}
