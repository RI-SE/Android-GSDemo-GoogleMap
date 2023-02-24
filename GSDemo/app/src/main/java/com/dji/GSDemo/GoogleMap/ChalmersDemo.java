package com.dji.GSDemo.GoogleMap;

import static com.dji.GSDemo.GoogleMap.Tools.showToast;

import android.Manifest;
import android.content.IntentFilter;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;

import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;

import org.jboss.netty.util.Timer;

import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.gimbal.CapabilityKey;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.util.CommonCallbacks;
import dji.common.util.DJIParamMinMaxCapability;
import dji.sdk.base.BaseProduct;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;


public class ChalmersDemo extends FragmentActivity implements View.OnClickListener {

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

    @Override
    protected void onResume() {
        super.onResume();
//        initFlightController();
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


        stop.setOnClickListener(this);

        enable_virtual_sticks.setOnClickListener(this);
        disable_virtual_sticks.setOnClickListener(this);

        take_off.setOnClickListener(this);
        land.setOnClickListener(this);

        gimbal_up.setOnClickListener(this);
        gimbal_down.setOnClickListener(this);
        gimbal_left.setOnClickListener(this);
        gimbal_right.setOnClickListener(this);
    }


    @Override
    public void onClick(View v) {
//        FlightControllerState flightControllerState = flightController.getState();
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
                    );

                }

                break;
            }

            case R.id.btn_stop: {
                setResultToToast("STOP MOTHERFUCKER!");
                break;
            }

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

    private void changeGimbalAngles(float pitch, float yaw, float roll) {
        if (gimbal == null) return;
        Log.wtf("GIMBAL", "VAFAN JA VILL");

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

    }

}
