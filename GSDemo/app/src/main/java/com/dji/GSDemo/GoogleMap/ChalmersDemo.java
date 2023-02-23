package com.dji.GSDemo.GoogleMap;

import android.Manifest;
import android.content.IntentFilter;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
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
    private Gimbal gimbal;
    private Button up, down, stop, gimbal_up, gimbal_down, gimbal_left, gimbal_right;
    private CommonCallbacks.CompletionCallback callback;


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
        up = (Button) findViewById(R.id.btn_up);
        down = (Button) findViewById(R.id.btn_down);
        stop = (Button) findViewById(R.id.btn_stop);

        gimbal_up = (Button) findViewById(R.id.btn_gimbal_up);
        gimbal_down = (Button) findViewById(R.id.btn_gimbal_down);
        gimbal_left = (Button) findViewById(R.id.btn_gimbal_left);
        gimbal_right = (Button) findViewById(R.id.btn_gimbal_right);

        up.setOnClickListener(this);
        down.setOnClickListener(this);
        stop.setOnClickListener(this);

        gimbal_up.setOnClickListener(this);
        gimbal_down.setOnClickListener(this);
        gimbal_left.setOnClickListener(this);
        gimbal_right.setOnClickListener(this);

    }


    @Override
    public void onClick(View v) {
//        FlightControllerState flightControllerState = flightController.getState();
        switch (v.getId()) {
            case R.id.btn_up: {
                setResultToToast("GOING UP!!!!!");


//                if (!flightControllerState.areMotorsOn()) {
//                    flightController.turnOnMotors(new CommonCallbacks.CompletionCallback() {
//                        @Override
//                        public void onResult(@Nullable final DJIError djiError) {
//                            if (djiError != null) {
//                                Log.wtf("CHALMERS_ERROR_UP", djiError.getDescription());
//                            } else {
//                                setResultToToast("Execution finished:");
//                            }
//                        }
//                    });
//                }

//                LocationCoordinate3D coords = flightControllerState.getAircraftLocation();

//                Log.wtf("COORDS", coords.toString());
                break;
            }
            case R.id.btn_down: {
                setResultToToast("GOING DOWN!!!!!");

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

//        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
//                .findFragmentById(R.id.map);
//        mapFragment.getMapAsync(this);

//        addListener();

        //createIsoDroneTask
        //Task droneTask = new Task();
        //droneTask.run();
//
    }

}
