package com.dji.GSDemo.GoogleMap;


import android.Manifest;
import android.content.IntentFilter;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;


import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import org.asta.isoObject.CartesianPosition;
import org.asta.isoObject.TrajectoryWaypointVector;
import org.locationtech.proj4j.CRSFactory;
import org.locationtech.proj4j.CoordinateReferenceSystem;
import org.locationtech.proj4j.CoordinateTransform;
import org.locationtech.proj4j.CoordinateTransformFactory;
import org.locationtech.proj4j.ProjCoordinate;

import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;


public class ChalmersDemo extends FragmentActivity implements TextureView.SurfaceTextureListener, View.OnClickListener, View.OnTouchListener {

    private static final String TAG = MainActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    TextureView mVideoSurface;
    Timer mGimbalTaskTimer;
    private FlightController flightController;
    private FlightControllerState djiFlightControllerCurrentState;
    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;
    private String lastDroneState = "";
    private IsoDrone drone;
    private double droneLocationLat = 57.688859d, droneLocationLng = 11.978795d, droneAltitude = 0d; // Johanneberg
    private Gimbal gimbal;
    private Button take_off, land, enable_virtual_sticks, disable_virtual_sticks, stop, gimbal_up, gimbal_down, gimbal_left, gimbal_right;
    private CommonCallbacks.CompletionCallback callback;
    private float mPitch;
    private float mRoll;
    private float mYaw;
    private float mThrottle;
    private DJICodecManager mCodecManager;
    private float mGimbalPitch;
    private float mGimbalRoll;
    private float mGimbalYaw;
    public CRSFactory crsFactory = new CRSFactory();
    public CoordinateReferenceSystem WGS84 = crsFactory.createFromParameters("WGS84","+proj=longlat +datum=WGS84 +no_defs");


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
        stop = (Button) findViewById(R.id.btn_stop);

        enable_virtual_sticks = (Button) findViewById(R.id.btn_enable_virtual_sticks);
        disable_virtual_sticks = (Button) findViewById(R.id.btn_disable_virtual_sticks);
        take_off = (Button) findViewById(R.id.btn_take_off);
        land = (Button) findViewById(R.id.btn_land);

        gimbal_up = (Button) findViewById(R.id.btn_gimbal_up);
        gimbal_down = (Button) findViewById(R.id.btn_gimbal_down);
        gimbal_left = (Button) findViewById(R.id.btn_gimbal_left);
        gimbal_right = (Button) findViewById(R.id.btn_gimbal_right);

        mVideoSurface = (TextureView) findViewById(R.id.video_preview_surface);
        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);
        }

        stop.setOnClickListener(this);

        enable_virtual_sticks.setOnClickListener(this);
        disable_virtual_sticks.setOnClickListener(this);

        take_off.setOnClickListener(this);
        land.setOnClickListener(this);

        gimbal_up.setOnTouchListener(this);
        gimbal_down.setOnTouchListener(this);
        gimbal_left.setOnTouchListener(this);
        gimbal_right.setOnTouchListener(this);
    }

    @Override
    public void onClick(View v) {
        FlightControllerState flightControllerState = flightController.getState();
        switch (v.getId()) {
            case R.id.btn_enable_virtual_sticks: {
                if (flightController != null) {
                    flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                Log.wtf("Virtual Sticks Error", djiError.getDescription());
                                setResultToToast("Enable Virtual Sticks Error, check Log");
                            } else {
                                setResultToToast("Enable Virtual Sticks Success");
                            }
                        }
                    });
                }
                break;
            }
            case R.id.btn_disable_virtual_sticks: {
                if (flightController != null) {
                    flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                Log.wtf("Virtual Sticks Error", djiError.getDescription());
                                setResultToToast("Disable Virtual Sticks Error, check Log");
                            } else {
                                setResultToToast("Disable Virtual Sticks Success");
                            }
                        }
                    });
                }
                break;
            }

            case R.id.btn_take_off: {
                if (flightController != null) {
                    flightController.startTakeoff(
                            new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if (djiError != null) {
                                        setResultToToast(djiError.getDescription());
                                    } else {
                                        setResultToToast("Take off Success");
                                    }
                                }
                            }
                    );
                }

                LocationCoordinate3D coords = flightControllerState.getAircraftLocation();

                Log.wtf("COORDS", coords.toString());
                break;
            }

            case R.id.btn_stop: {
                setResultToToast("STOP MOTHERFUCKER!");

                flightController.turnOffMotors(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(@Nullable final DJIError djiError) {
                        if (djiError != null) {
                            Log.wtf("CHALMERS_ERROR_UP", djiError.getDescription());
                        } else {
                            setResultToToast("Execution finished:");
                        }
                    }

                });
                break;
            }

            case R.id.btn_land: {
                if (flightController != null) {
                    flightController.startLanding(
                            new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if (djiError != null) {
                                        setResultToToast(djiError.getDescription());
                                    } else {
                                        setResultToToast("Start Landing");
                                    }
                                }
                            }
                    );
                }


                break;
            }

            case R.id.btn_gimbal_up: {
                changeGimbalAngles(24, Rotation.NO_ROTATION, Rotation.NO_ROTATION);
                Log.wtf("GIMBAL up", "gimbal up");
                break;
            }

            default:
                break;
        }

    }


    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        Log.wtf(TAG, "We in OnTouchMethod");
        int action = motionEvent.getAction();
        switch (view.getId()) {
            case R.id.btn_gimbal_up: {
                Log.wtf(TAG, "Go up, pitch:" + mGimbalPitch + " roll:" + mGimbalRoll + "yaw: " + mGimbalYaw);
                mGimbalPitch = 1;
                mGimbalRoll = 0;
                mGimbalYaw = 0;
//                changeGimbalAngles(mGimbalPitch, mGimbalRoll, mGimbalYaw);
                break;
            }
            case R.id.btn_gimbal_down: {
                Log.wtf(TAG, "Go down, pitch:" + mGimbalPitch + " roll:" + mGimbalRoll + "yaw: " + mGimbalYaw);
                mGimbalPitch = -1;
                mGimbalRoll = 0;
                mGimbalYaw = 0;

//                changeGimbalAngles(mGimbalPitch, mGimbalRoll, mGimbalYaw);
                break;
            }
            case R.id.btn_gimbal_right: {
                Log.wtf(TAG, "Go right, pitch:" + mGimbalPitch + " roll:" + mGimbalRoll + "yaw: " + mGimbalYaw);
                mGimbalPitch = 0;
                mGimbalRoll = 0;
                mGimbalYaw = -1;

//                changeGimbalAngles(mGimbalPitch, mGimbalRoll, mGimbalYaw);
                break;
            }
            case R.id.btn_gimbal_left: {
                Log.wtf(TAG, "Go left, pitch:" + mGimbalPitch + " roll:" + mGimbalRoll + "yaw: " + mGimbalYaw);
                mGimbalPitch = 0;
                mGimbalRoll = 0;
                mGimbalYaw = 1;
//                changeGimbalAngles(mGimbalPitch, mGimbalRoll, mGimbalYaw);
                break;
            }
        }

        if (action == MotionEvent.ACTION_DOWN) {
            setResultToToast("Touching down..");
            if (null == mGimbalTaskTimer) {
                ChangeGimbalTask mGimbalTask = new ChangeGimbalTask();
                mGimbalTaskTimer = new Timer();
                mGimbalTaskTimer.schedule(mGimbalTask, 25, 50);
            }

        } else if (action == MotionEvent.ACTION_UP) {
            setResultToToast("Touching up..");
            if (mGimbalTaskTimer != null) {
                mGimbalTaskTimer.cancel();
                mGimbalTaskTimer = null;
            }
        }
        return true;
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
                    droneLocationLat = djiFlightControllerCurrentState.getAircraftLocation().getLatitude();
                    droneLocationLng = djiFlightControllerCurrentState.getAircraftLocation().getLongitude();
                    droneAltitude = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();
                    updateDroneLocationData();
//                    Log.wtf("LOCATION LAT", String.valueOf(droneLocationLat));
//                    Log.wtf("LOCATION LONG", String.valueOf(droneLocationLng));
//                    Log.wtf("LOCATION ALT", String.valueOf(droneAltitude));
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
        Camera camera = DJIDemoApplication.getProductInstance().getCamera();
        if (camera != null) {
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    private void updateDroneLocationData() {

        if (lastDroneState == "") {
            drone = new IsoDrone("192.168.213.129");
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
        } else if (drone.getCurrentStateName().equals("PreArming") && lastDroneState != "PreArming") {
            Log.wtf("Error", "PreArming");
            lastDroneState = "PreArming";

        } else if (drone.getCurrentStateName().equals("Armed") && lastDroneState != "Armed") {
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
            setResultToToast("NormalStop");
            lastDroneState = "NormalStop";
        } else if (drone.getCurrentStateName().equals("EmergencyStop") && lastDroneState != "EmergencyStop") {
            Log.wtf("Error", "EmergencyStop");
            lastDroneState = "EmergencyStop";


        }

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

    private String buildOriginProjString(double latitude, double longitude){
        final StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("+proj=tmerc +lat_0=" + latitude + " +lon_0=" + longitude + " +k=0.9996 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs");
        return stringBuffer.toString();
    }

}
