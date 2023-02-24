package com.dji.GSDemo.GoogleMap;

import static com.dji.GSDemo.GoogleMap.Tools.showToast;

import android.Manifest;
import android.content.IntentFilter;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;

import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
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
import dji.sdk.products.Aircraft;


public class ChalmersDemo extends FragmentActivity implements TextureView.SurfaceTextureListener, View.OnClickListener, View.OnTouchListener {

    private static final String TAG = MainActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    TextureView mVideoSurface;
    Timer mGimbalTaskTimer;
    private FlightController flightController;
    private FlightControllerState djiFlightControllerCurrentState;

    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;
    private double droneLocationLat = 57.688859d, droneLocationLng = 11.978795d, droneAltitude = 0d; // Johanneberg
    private Gimbal gimbal;
    private Button take_off, land, enable_virtual_sticks, disable_virtual_sticks,stop, gimbal_up, gimbal_down, gimbal_left, gimbal_right;
    private CommonCallbacks.CompletionCallback callback;
    private float mPitch;
    private float mRoll;
    private float mYaw;
    private float mThrottle;
    public double getDroneLocationLat(){return droneLocationLat;}
    public double getDroneLocationLng(){return droneLocationLng;}
    private static final String TAG = MainActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    TextureView mVideoSurface;
    private DJICodecManager mCodecManager;
    private float mGimbalPitch;
    private float mGimbalRoll;
    private float mGimbalYaw;

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

        up.setOnClickListener(this);
        down.setOnClickListener(this);
        stop.setOnClickListener(this);

//        gimbal_up.setOnClickListener(this);
//        gimbal_down.setOnClickListener(this);
//        gimbal_left.setOnClickListener(this);
//        gimbal_right.setOnClickListener(this);

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
                if (flightController != null){
                    flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null){
                                Log.wtf("Virtual Sticks Error", djiError.getDescription());
                                setResultToToast("Enable Virtual Sticks Error, check Log");
                            }
                            else{
                                setResultToToast("Enable Virtual Sticks Success");
                            }
                        }
                    });
                }
                break;
            }
            case R.id.btn_disable_virtual_sticks: {
                if (flightController != null){
                    flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null){
                                Log.wtf("Virtual Sticks Error", djiError.getDescription());
                                setResultToToast("Disable Virtual Sticks Error, check Log");
                            }
                            else{
                                setResultToToast("Disable Virtual Sticks Success");
                            }
                        }
                    });
                }
                break;
            }

            case R.id.btn_take_off: {
                if (flightController != null){
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

                    }
                });
                break;
            }
            case R.id.btn_land: {
                if (flightController != null){
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
                    );}



                break;}

            case R.id.btn_gimbal_up: {
                changeGimbalAngles(24, Rotation.NO_ROTATION, Rotation.NO_ROTATION);
                Log.wtf("GIMBAL up", "gimbal up");
                break;
            }

            case R.id.btn_gimbal_down: {
                changeGimbalAngles(-90, Rotation.NO_ROTATION, Rotation.NO_ROTATION);
                Log.wtf("GIMBAL down", "gimbal down");
                break;
            }

            case R.id.btn_gimbal_left: {
                changeGimbalAngles(Rotation.NO_ROTATION, 40, Rotation.NO_ROTATION);
                Log.wtf("GIMBAL left", "gimbal left");
                break;
            }

            case R.id.btn_gimbal_right: {
                changeGimbalAngles(Rotation.NO_ROTATION, -40, Rotation.NO_ROTATION);
                Log.wtf("GIMBAL right", "gimbal right");
                break;
            }

            default:
                break;
        }

    }


    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        int action = motionEvent.getAction();
        switch (view.getId()) {
            case R.id.btn_gimbal_up: {
                mGimbalPitch = 5;
                mGimbalRoll = 0;
                mGimbalYaw = 0;
                break;
            }
            case R.id.btn_gimbal_down: {
                mGimbalPitch = -5;
                mGimbalRoll = 0;
                mGimbalYaw = 0;
                break;
            }
            case R.id.btn_gimbal_right: {
                mGimbalPitch = 0;
                mGimbalRoll = 5;
                mGimbalYaw = 0;
                break;
            }
            case R.id.btn_gimbal_left: {
                mGimbalPitch = 0;
                mGimbalRoll = -5;
                mGimbalYaw = 0;
                break;
            }
        }

        if (action == MotionEvent.ACTION_DOWN) {
            if (null == mGimbalTaskTimer) {
                ChangeGimbalTask mGimbalTask = new ChangeGimbalTask();
                mGimbalTaskTimer = new Timer();
                mGimbalTaskTimer.schedule(mGimbalTask, 50, 100);
            }

        } else if (action == MotionEvent.ACTION_UP) {
            if (mGimbalTaskTimer != null) {
                mGimbalTaskTimer.cancel();
                mGimbalTaskTimer.purge();
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
                        .time(1)
                        .mode(RotationMode.ABSOLUTE_ANGLE)
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
                                } else {
                                    Log.wtf("Virtual Stick","Sent Virtual stick data to flightcontroler");
                                }
                            }
                        }
                );
            }
        }
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
//                    updateDroneLocationData();
                    Log.wtf("LOCATION LAT", String.valueOf(droneLocationLat));
                    Log.wtf("LOCATION LONG", String.valueOf(droneLocationLng));
                    Log.wtf("LOCATION ALT", String.valueOf(droneAltitude));
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

    private void uninitPreviewer() {
        Camera camera = DJIDemoApplication.getProductInstance().getCamera();
        if (camera != null){
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    class ChangeGimbalTask extends TimerTask {
        @Override
        public void run() {
            gimbal.rotate(new Rotation.Builder()
                            .pitch(mGimbalPitch)
                            .yaw(mGimbalYaw)
                            .roll(mGimbalRoll)
                            .time(1)
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
    }


}
