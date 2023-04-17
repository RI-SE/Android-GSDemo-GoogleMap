package com.dji.GSDemo.GoogleMap;

import android.Manifest;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;

import com.google.android.gms.maps.model.LatLng;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.asta.isoObject.CartesianPosition;
import org.asta.isoObject.TrajectoryWaypointVector;
import org.locationtech.proj4j.CRSFactory;
import org.locationtech.proj4j.CoordinateReferenceSystem;
import org.locationtech.proj4j.CoordinateTransform;
import org.locationtech.proj4j.CoordinateTransformFactory;
import org.locationtech.proj4j.ProjCoordinate;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.GPSSignalLevel;
import dji.common.gimbal.GimbalMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointExecutionProgress;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionDownloadEvent;
import dji.common.mission.waypoint.WaypointMissionExecuteState;
import dji.common.mission.waypoint.WaypointMissionExecutionEvent;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionState;
import dji.common.mission.waypoint.WaypointMissionUploadEvent;
import dji.common.model.LocationCoordinate2D;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.keysdk.DJIKey;
import dji.keysdk.FlightControllerKey;
import dji.midware.media.DJIVideoDataRecver;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.mission.waypoint.WaypointMissionOperatorListener;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;


public class ChalmersDemo extends FragmentActivity implements TextureView.SurfaceTextureListener, View.OnClickListener, WaypointMissionOperatorListener, CompoundButton.OnCheckedChangeListener {

    private static final int MAIN_CAMERA_INDEX = 0;
    private static final int INVAVID_INDEX = -1;
    private static final int MOVE_OFFSET = 20;
    private static final String TAG = "CHALMERS_DEMO";
    public static WaypointMission.Builder waypointMissionBuilder;
    private final DJIKey trackModeKey = FlightControllerKey.createFlightAssistantKey(FlightControllerKey.ACTIVE_TRACK_MODE);
    public CRSFactory crsFactory = new CRSFactory();
    public CoordinateReferenceSystem WGS84 = crsFactory.createFromParameters("WGS84", "+proj=longlat +datum=WGS84 +no_defs");
    private VideoFeeder.VideoDataListener mReceivedVideoDataListener;
    View decorView;
    TextureView mVideoSurface;
    float mGimbalRoll;
    float mGimbalYaw;
    float mGimbalPitch;
    private com.dji.GSDemo.GoogleMap.Utils Utils;
    private String lastDroneState = "";
    private IsoDrone drone;
    private FlightController flightController;
    private FlightControllerState djiFlightControllerCurrentState;
    private Gimbal gimbal;
    private Camera camera;
    private DJICodecManager mCodecManager;
    private WaypointMissionOperator waypointInstance;
    private WaypointMissionFinishedAction mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
    private WaypointMissionHeadingMode mHeadingMode = WaypointMissionHeadingMode.AUTO;
    private ArrayList<WaypointSetting> waypointSettings = new ArrayList<>();
    private ArrayList<Waypoint> waypointList = new ArrayList<>();
    private TextView text_gps, text_lat, text_lon, text_alt;
    private Button testcircle, config, upload, start, stop, land;
    private Button btn_mission_status, btn_ip_address, btn_drone_state, clear_wps;

    private Switch switch_dry_run;
    private double droneLocationLat = 57.688859d, droneLocationLng = 11.978795d, droneAltitude = 0d; // Johanneberg
    private float mSpeed = 10.0f;
    private float altitude = 100.0f;
    private LatLng startLatLong;
    private boolean missionUploaded = false;

    private boolean runTest = false;
    private WaypointMissionExecuteState waypointState;
    private boolean first;

    public double getDroneLocationLat() {
        return droneLocationLat;
    }

    public double getDroneLocationLng() {
        return droneLocationLng;
    }

    public double getDroneAltitude() {
        return droneAltitude;
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
        try {
            DJIVideoDataRecver.getInstance().setVideoDataListener(false, null);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (mCodecManager != null) {
            mCodecManager.destroyCodec();
        }

        super.onDestroy();
//        unregisterReceiver(mReceiver) ;
//        removeListener();
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
            case R.id.testcircle: {
                this.waypointSettings.clear();
                generateTestCircleCoordinates(new LatLng(droneLocationLat, droneLocationLng), 1, 5.0f, 1, 8, false);
                try {
                    deployTraj();
                } catch (Exception e){
                    Log.wtf("ERROR","catch at generateTestCircle" + e.getMessage());
                }
                break;
            }
            case R.id.pauseresume: {
                Button button = (Button) findViewById(R.id.pauseresume);
                if (button.getText().equals("Pause")) {
                    pauseWaypointMission();
                    button.setText("Resume");
                } else if (button.getText().equals("Resume")) {
                    resumeWaypointMission();
                    button.setText("Pause");
                } else if (button.getText().equals("Armed")) {
                    resumeWaypointMission();
                    button.setText("Running");
                }
                break;
            }
            case R.id.clear_wps: {
                waypointList.clear();
                break;
            }
            case R.id.start: {
                startWaypointMission();
                break;
            }
            case R.id.stop: {
                stopWaypointMission();
                break;
            }
            case R.id.land: {
                LandWaypointMission();
                break;
            }

            default:
                break;
        }
    }

    private void initUI() {
        btn_mission_status = (Button) findViewById(R.id.btn_mission_status);
        btn_drone_state = (Button) findViewById(R.id.btn_drone_state);
        btn_ip_address = (Button) findViewById(R.id.btn_ip_address);

        switch_dry_run = (Switch) findViewById(R.id.switch_dry_run);
        switch_dry_run.setClickable(true);

        btn_ip_address.setText(Utils.getIPAddress(true));
        btn_drone_state.setText("State: Undefined");

        text_gps = (TextView) findViewById(R.id.text_gps);
        text_lat = (TextView) findViewById(R.id.text_lat);
        text_lon = (TextView) findViewById(R.id.text_lon);
        text_alt = (TextView) findViewById(R.id.text_alt);
        mVideoSurface = (TextureView) findViewById(R.id.video_previewer_surface);


        testcircle = (Button) findViewById(R.id.testcircle);
        config = (Button) findViewById(R.id.pauseresume);
        clear_wps = (Button) findViewById(R.id.clear_wps);
        start = (Button) findViewById(R.id.start);
        stop = (Button) findViewById(R.id.stop);
        land = (Button) findViewById(R.id.land);

        testcircle.setOnClickListener(this);
        config.setOnClickListener(this);
        clear_wps.setOnClickListener(this);
        start.setOnClickListener(this);
        stop.setOnClickListener(this);
        land.setOnClickListener(this);

        switch_dry_run.setOnCheckedChangeListener(this);

        if (null != mVideoSurface) {
//            mVideoSurface.setSurfaceTextureListener(this);

        }
    }


    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
//        Log.e(TAG, "onSurfaceTextureAvailable");
//
//
        if (mCodecManager == null) {
            mCodecManager = new DJICodecManager(this, surface, width, height);
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureSizeChanged");
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {

        Log.e(TAG, "onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }

        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
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
//            flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            flightController.setHomeLocationUsingAircraftCurrentLocation(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    setResultToToast("Set drone home at take off site: " + (error == null ? "Success" : error.getDescription()));
                }
            });
            flightController.setStateCallback(new FlightControllerState.Callback() {
                @Override
                public void onUpdate(FlightControllerState djiFlightControllerCurrentState) {
                    GPSSignalLevel gps = djiFlightControllerCurrentState.getGPSSignalLevel();
                    droneLocationLat = djiFlightControllerCurrentState.getAircraftLocation().getLatitude();
                    droneLocationLng = djiFlightControllerCurrentState.getAircraftLocation().getLongitude();
                    droneAltitude = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();

                    if (getWaypointMissionOperator() != null){
                        Log.wtf("ERROR", getWaypointMissionOperator().getCurrentState().toString());
                    }

                    if (text_lon != null && text_lat != null && text_gps != null && text_alt != null) {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                text_gps.setText("GPS: " + gps.toString());
                                text_lat.setText("LAT: " + (!Double.isNaN(droneLocationLat) ? String.valueOf(droneLocationLat) : "Unknown"));
                                text_lon.setText("LONG: " + (!Double.isNaN(droneLocationLng) ? String.valueOf(droneLocationLng) : "Unknown"));
                                text_alt.setText("ALT: " + (!Double.isNaN(droneAltitude) ? String.valueOf(droneAltitude) : "Unknown"));
                            }
                        });
//
                    }
//                    Log.wtf("LOCATION GPS", String.valueOf(gps));
//                    Log.wtf("LOCATION LAT", String.valueOf(droneLocationLat));
//                    Log.wtf("LOCATION LONG", String.valueOf(droneLocationLng));
//                    Log.wtf("LOCATION ALT", String.valueOf(droneAltitude));
                    updateDroneLocationData();
                }
            });
        }
    }

    private void initCameraAndGimbal() {
        BaseProduct product = DJIDemoApplication.getProductInstance();
        if (product != null && product.isConnected()) {
            if (product instanceof Aircraft) {
                gimbal = ((Aircraft) product).getGimbal();
                camera = ((Aircraft) product).getCamera();
                gimbal.setMode(GimbalMode.FREE, new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        Log.wtf(TAG, "Set gimbal mode to FREE: " + ((djiError == null) ? "Success" : djiError.getDescription()));
                    }
                });
            }
        }
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
        initCameraAndGimbal();

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
        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                Log.e("VIDEO", "I am running the function!");
                setResultToToast("WE ARE RUNNING ON RECEIVE");
                if (mCodecManager != null) {
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };
        initUI();
        initPreviewer();
    }


    public void onExecutionUpdate(WaypointMissionExecutionEvent event) {
        // Handle execution updates here
        WaypointExecutionProgress progress = event.getProgress();
        waypointState = progress.executeState;
        Log.wtf("Error", "Pogress: " + progress.executeState);
        updateMissionButton(("Running " + progress.targetWaypointIndex + "/" + progress.totalWaypointCount),Color.CYAN);
        assert progress != null;
        if (progress.isWaypointReached) {
            Log.wtf("Error", "We reached a waypoint");
            setResultToToast("WE REACHED A WAYPOINT");

            //If we armed and went to first waypoint, stop and wait for running
//            if (progress.targetWaypointIndex == 0 && drone.getCurrentStateName() == "Armed") {
            if (first) {
                first=false;
                pauseWaypointMission();
                setResultToToast("FIRST WAYPOINT REACHED");
            }
        }
    }


    @Override
    public void onDownloadUpdate(WaypointMissionDownloadEvent event) {
        // Handle download updates here
        Log.wtf(TAG, "WaypointMission download was updated");
        updateMissionButton("Downloading...", Color.GRAY);
    }

    @Override
    public void onUploadUpdate(WaypointMissionUploadEvent event) {
        // Handle upload updates here
        Log.wtf(TAG, "WaypointMission was updated");
        if (event.getProgress() != null) {
            updateMissionButton(("Uploading " + event.getProgress().uploadedWaypointIndex + "/" + event.getProgress().totalWaypointCount), Color.CYAN);
        }
    }
    @Override
    public void onExecutionStart() {
        // Handle execution start here
        Log.wtf(TAG, "WaypointMission was started");
        updateMissionButton("Starting...", Color.GREEN);
    }

    @Override
    public void onExecutionFinish(DJIError error) {
        // Handle execution finish here
        Log.wtf(TAG, "WaypointMission has finished");
        updateMissionButton("Finished", Color.DKGRAY);

    }


    protected void onProductChange() {
        initPreviewer();
    }

    //For camera feed
    private void initPreviewer() {

        BaseProduct product = DJIDemoApplication.getProductInstance();

        if (product == null || !product.isConnected()) {
            setResultToToast(getString(R.string.disconnected));
        } else {
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
        camera = DJIDemoApplication.getProductInstance().getCamera();
        if (camera != null) {
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    private void startDroneMotors() {
        flightController.turnOnMotors(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                Log.wtf("Error", (djiError == null ? "Started drone motors." : djiError.getDescription()));
            }
        });
    }

    private void setDroneHomeLocation() {
        flightController.setHomeLocationUsingAircraftCurrentLocation(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                Log.wtf("Error", (djiError == null ? "Set home at current location." : djiError.getDescription()));
            }
        });
    }


    private void startDroneRecording() {
        camera.startRecordVideo(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                Log.wtf("Error", (djiError == null ? "Started recording." : djiError.getDescription()));
            }
        });
    }

    private void stopDroneRecording() {
        camera.stopRecordVideo(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                Log.wtf("Error", (djiError == null ? "Stopped recording." : djiError.getDescription()));
            }
        });
    }


    private void updateDroneLocationData() {
        if (lastDroneState == "") {
            drone = new IsoDrone(Utils.getIPAddress(true));
            lastDroneState = drone.getCurrentStateName();
        }
//        Log.wtf("Error", "Drone is in: " + drone.getCurrentStateName());
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
            updateStateButton(lastDroneState, Color.parseColor("#31d2f2"));
            setDroneHomeLocation();
        } else if (drone.getCurrentStateName().equals("PreArming") && lastDroneState != "PreArming") {
            Log.wtf("Error", "PreArming");
            lastDroneState = "PreArming";
            updateStateButton(lastDroneState, Color.parseColor("#ff6600"));

        } else if (drone.getCurrentStateName().equals("Armed") && lastDroneState != "Armed") {
            Log.wtf("Error", "Armed");
            updateStateButton(lastDroneState, Color.parseColor("#ff6600"));
            runOnUiThread(new Runnable() {
                @Override
                public void run() {

                    Log.wtf("TrajName: ", drone.getTrajectoryHeader().getTrajectoryName());
                    TrajectoryWaypointVector traj = drone.getTrajectory();
                    waypointSettings.clear();
                    setResultToToast("Initial size:" + traj.size());
                    //Reduce points in traj if to large (99 max amount of waypoints)
                    if (traj.size() > 99) {
                        double epsilon = 0.1;
                        do {
                            Log.wtf(TAG, "Running with epsilon: " + epsilon);
                            drone.reducePoints(epsilon);
                            epsilon += 0.1;
                            setResultToToast("reducing traj");
                        } while (drone.getReducedTraj().size() > 99 && epsilon < 10); //0.06
                        Log.wtf("newTraj", String.valueOf(drone.getReducedTraj().size()));
                        waypointSettings.clear();
                        setResultToToast("peucker Traj: " + String.valueOf(drone.getReducedTraj().size()));
                        drone.removePointsToClose();
                        setResultToToast("remove points to close: " + String.valueOf(drone.getReducedTraj().size()));
                        generateWaypointsFromTraj(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), drone.getReducedTraj()); //Use reduced
                    } else {
                        waypointSettings.clear();
                        drone.reducedTraj = drone.getTrajectory(); //Put the non douglas-peucker:ed traj in
                        setResultToToast("non reduced Traj: " + String.valueOf(drone.getReducedTraj().size()));
                        drone.removePointsToClose();
                        setResultToToast("remove points to close: " + String.valueOf(drone.getReducedTraj().size()));
                        generateWaypointsFromTraj(new LatLng(drone.getOrigin().getLatitude_deg(), drone.getOrigin().getLongitude_deg()), drone.reducedTraj); // use set traj
                    }
                    //generateTestCircleCoordinates(new LatLng(droneLocationLat, droneLocationLng), 10, 3, 1,19, true);
                    try {
                        deployTraj();
                    } catch (Exception e) {
                        Log.wtf("ERROR","catch at updateDroneLocation:" + e.getMessage());
                    }

                }
            });

            Log.wtf("Error", "Armed");
            lastDroneState = "Armed";
//            startDroneMotors();

        } else if (drone.getCurrentStateName().equals("Disarmed")) {
            Log.wtf("Error", "Disarmed");
            lastDroneState = "Disarmed";
            sendMonr();
            updateStateButton(lastDroneState, Color.parseColor("#ffca2c"));

        } else if (drone.getCurrentStateName().equals("PreRunning") && lastDroneState != "PreRunning") {
            Log.wtf("Error", "PreRunning");
            lastDroneState = "PreRunning";
            updateStateButton(lastDroneState, Color.DKGRAY);

        } else if (drone.getCurrentStateName().equals("Running") && lastDroneState != "Running") {
            if (waypointState == WaypointMissionExecuteState.PAUSED) {
                resumeWaypointMission();
            } else {
                setResultToToast("WaypointMissionState is not PAUSED");
            }

            Log.wtf("Error", "Running");
            lastDroneState = "Running";
            updateStateButton(lastDroneState, Color.parseColor("#157347"));
//            startDroneRecording();

        } else if (drone.getCurrentStateName().equals("NormalStop") && lastDroneState != "NormalStop") {
            setResultToToast("NormalStop");
            pauseWaypointMission();
            lastDroneState = "NormalStop";
            updateStateButton(lastDroneState, Color.parseColor("#bb2d3b;"));
//            stopDroneRecording();

        } else if (drone.getCurrentStateName().equals("EmergencyStop") && lastDroneState != "EmergencyStop") {
            Log.wtf("Error", "EmergencyStop");
            lastDroneState = "EmergencyStop";
            stopWaypointMission();
            updateStateButton(lastDroneState, Color.parseColor("#bb2d3b;"));
//            stopDroneRecording();
        }
    }

    public void sendMonr() {
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

        monrPos.setHeading_rad(headingToYaw(flightController.getState().getAttitude().yaw) * Math.PI / 180);
        monrPos.setIsHeadingValid(true);
        drone.setPosition(monrPos);

    }

    private void updateStateButton(String state, Integer color) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (btn_drone_state != null) {
                    btn_drone_state.setText("STATE: " + state);
                    btn_drone_state.setBackgroundColor(color);
                }
            }
        });
    }

    private void updateMissionButton(String state, Integer color) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (btn_mission_status != null) {
                    btn_mission_status.setText("MISSION: " + state);
                    btn_mission_status.setBackgroundColor(color);
                }
            }
        });
    }

    private String buildOriginProjString(double latitude, double longitude) {
        final StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("+proj=tmerc +lat_0=" + latitude + " +lon_0=" + longitude + " +k=0.9996 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs");
        return stringBuffer.toString();
    }


    private ProjCoordinate coordCartToGeo(LatLng origin, ProjCoordinate xyz) {
        String projStr = buildOriginProjString(origin.latitude, origin.longitude);
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateReferenceSystem UTM = crsFactory.createFromParameters("UTM", projStr);
        CoordinateTransform utmToWgs = ctFactory.createTransform(UTM, WGS84);
        ProjCoordinate result = new ProjCoordinate();
        utmToWgs.transform(new ProjCoordinate(xyz.x, xyz.y), result);
        return result;
    }


    private void generateWaypointsFromTraj(LatLng origin, TrajectoryWaypointVector trajectory) {

        //Add extra point before first point to correct heading
        //add trajPoints
        RealMatrix A = MatrixUtils.createRealMatrix(trajectory.size(), trajectory.size());
        ArrayRealVector b = new ArrayRealVector(trajectory.size());
        for (int i = 0; i < trajectory.size(); ++i) {
            A.setEntry(i, i, 1);
            if (i + 1 < trajectory.size()) {
                A.setEntry(i, i + 1, 1);
                Double dist = Math.sqrt(
                        Math.pow(trajectory.get(i).getPos().getXCoord_m() - trajectory.get(i + 1).getPos().getXCoord_m(), 2)
                                + Math.pow(trajectory.get(i).getPos().getYCoord_m() - trajectory.get(i + 1).getPos().getYCoord_m(), 2));
                b.setEntry(i, dist);
            } else {
                b.setEntry(i, 0.2);
            }
        }
        DecompositionSolver solver = new LUDecomposition(A).getSolver();
        RealVector radii = solver.solve(b);

        for (int i = 0; i < trajectory.size(); i++) {
            WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(trajectory.get(i).getPos().getXCoord_m(), trajectory.get(i).getPos().getYCoord_m(), trajectory.get(i).getPos().getZCoord_m())), new ProjCoordinate());
            this.waypointSettings.add(wps);
            this.waypointSettings.get(i).heading = (int) yawToHeading((180 / Math.PI) * trajectory.get(i).getPos().getHeading_rad());
//            this.waypointSettings.get(i).geo.z = trajectory.get(i).getPos().getZCoord_m();
            this.waypointSettings.get(i).geo.z = 8;
            this.waypointSettings.get(i).speed = (float) trajectory.get(i).getSpd().getLongitudinal_m_s(); //Possible lossy conversion?
            this.waypointSettings.get(i).radius = radii.getEntry(i) - 0.01;
        }

        //add landing point
        // WaypointSetting wps =  54new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(0, 0)), new ProjCoordinate());
        // wps.heading = 0;
        // wps.speed = 15;
        // wps.geo.z = 6;
        // this.waypointSettings.add(wps);
    }


    private void deployTraj() {
        Log.wtf("Error", "Deploying traj");
        waypointMissionBuilder = new WaypointMission.Builder();

//        for (int point = 0; point < 10; point++) {
        for (int point = 0; point < this.waypointSettings.size(); point++) {
            Waypoint wp = new Waypoint();
            wp.coordinate = new LocationCoordinate2D(this.waypointSettings.get(point).geo.y, this.waypointSettings.get(point).geo.x);
            wp.altitude = (float) this.waypointSettings.get(point).geo.z;
            wp.speed = (float) this.waypointSettings.get(point).speed;
            try {
                wp.heading = this.waypointSettings.get(point).heading;
            } catch (Exception e) {
                setResultToToast("e = " + e.getCause() + ", " + e.getMessage());
            }
            waypointList.add(wp);
            Log.wtf("Waypoint", "i: " + point + " Coords: " + wp.coordinate.toString());
            waypointMissionBuilder.waypointList(waypointList).waypointCount(waypointList.size());
        }
        setResultToToast("Number of waypoints " + this.waypointSettings.size());
        mFinishedAction = WaypointMissionFinishedAction.GO_HOME;
        mSpeed = 5.0f;
        mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;
        altitude = (float) this.waypointSettings.get(0).geo.z;
        configWayPointMission();
        //TODO Remove?
        startLatLong = new LatLng(droneLocationLat, droneLocationLng);
        uploadWayPointMission();

        while(getWaypointMissionOperator().getCurrentState() != WaypointMissionState.READY_TO_EXECUTE) {
            try {
                Thread.sleep(1000); // It's ugly, I know.
            }
            catch (InterruptedException e) {
                Log.wtf("ERROR", "WAITING for WaypointMissionState Ready_TO_EXECUTE");
            }

        }
        first = true;
        startWaypointMission();
    }


    private double headingToYaw(double heading_deg) {
        return wrapAngle360(90 - heading_deg);
    }

    private double yawToHeading(double yaw_deg) {
        return wrapAngle180(90 - yaw_deg);
    }

    private double wrapAngle360(double yaw_deg) {
        while (yaw_deg < 0) {
            yaw_deg += 360;
        }
        while (yaw_deg > 360) {
            yaw_deg -= 360;
        }
        return yaw_deg;
    }

    private double wrapAngle180(double yaw_deg) {
        while (yaw_deg < -180) {
            yaw_deg += 360;
        }
        while (yaw_deg > 180) {
            yaw_deg -= 360;
        }
        return yaw_deg;
    }

    private void generateTestCircleCoordinates(LatLng origin, double radius, double altitude, float speed, int nofPoints, boolean headingTowardsCenter) {

        double angularStep = 2 * Math.PI / nofPoints;
        double currentAngle = 0;
        double currentAngleRot = 0;
        int wpHeading = 0;
        for (int i = 0; i <= nofPoints; i++) {

            if (headingTowardsCenter) {
                currentAngleRot = rotateUnitCircleAngleToDroneYawRad(currentAngle, Math.PI);
            } else currentAngleRot = currentAngle;
            wpHeading = convertToDroneYawRangeDeg((180 / Math.PI) * currentAngleRot);

            WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(radius * Math.cos(currentAngle), radius * Math.sin(currentAngle), 0)), new ProjCoordinate());
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

    private Double rotateUnitCircleAngleToNorthHeadingRad(double yaw) {
        Double yawRot = 0.0;
        if (yaw >= 0 && yaw <= Math.PI / 2) yawRot = Math.PI / 2 - yaw;
        else if (yaw > Math.PI / 2 && yaw <= 2 * Math.PI) yawRot = Math.PI / 2 + 2 * Math.PI - yaw;
        return yawRot;
    }

    private Double rotateUnitCircleAngleToDroneYawRad(double yaw, double rot) {
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
        if (waypointInstance == null) {
            if (DJISDKManager.getInstance().getMissionControl() != null) {
                waypointInstance = DJISDKManager.getInstance().getMissionControl().getWaypointMissionOperator();
                waypointInstance.addListener(this);
            }
        }
        return waypointInstance;
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

    private void uploadWayPointMission() {
        getWaypointMissionOperator().uploadMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                if (error == null) {
                    setResultToToast("Mission begun upload successfully!");
                    missionUploaded = true;

                } else {
                    missionUploaded = false;
                    setResultToToast("Mission upload failed, error: " + error.getDescription() + " retrying...");
                    getWaypointMissionOperator().retryUploadMission(null);
                }
            }
        });

    }

    private void startWaypointMission() {
     if (!runTest) {
         setResultToToast("Dry run is enabled, will not run");
         return;
     }

    if (flightController != null)  {
        getWaypointMissionOperator().startMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Mission Start: " + (error == null ? "Successfully" : error.getDescription()));
                switch_dry_run.setClickable(false);
            }
        });
    }
    }

    private void pauseWaypointMission() {
            getWaypointMissionOperator().pauseMission(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    setResultToToast("Mission paused: " + (error == null ? "Successfully" : error.getDescription()));
                }
            });
    }
    private void resumeWaypointMission() {
        getWaypointMissionOperator().resumeMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Mission resumed: " + (error == null ? "Successfully" : error.getDescription()));
            }
        });
    }

    private void stopWaypointMission() {
        getWaypointMissionOperator().stopMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Mission Stop: " + (error == null ? "Successfully" : error.getDescription()));
                switch_dry_run.setClickable(true);
            }
        });

    }

    private void LandWaypointMission() {
        if (getWaypointMissionOperator().getCurrentState() == WaypointMissionState.EXECUTING || getWaypointMissionOperator().getCurrentState() == WaypointMissionState.EXECUTION_PAUSED) {
            stopWaypointMission();
        }
        flightController.startLanding(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast("Initializing landing: " + (error == null ? "Success" : error.getDescription()));
            }
        });
        if (flightController.getState().isLandingConfirmationNeeded()) {
            flightController.confirmLanding(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    setResultToToast("Confirming landing: " + (error == null ? "Success" : error.getDescription()));
                }
            });

        }
    }

    private ProjCoordinate coordGeoToCart(LatLng origin, ProjCoordinate llh) {
        String projStr = buildOriginProjString(origin.latitude, origin.longitude);
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateReferenceSystem UTM = crsFactory.createFromParameters("UTM", projStr);
        CoordinateTransform wgsToUtm = ctFactory.createTransform(WGS84, UTM);
        ProjCoordinate result = new ProjCoordinate();
        wgsToUtm.transform(new ProjCoordinate(llh.y, llh.x), result);
        result.z = llh.z;
        return result;
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


    @Override
    public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
        if (!b) {
            runTest = true;
            setResultToToast("Dry run is switched OFF");
        } else {
            setResultToToast("Dry run is switched ON");
        }
    }
}
