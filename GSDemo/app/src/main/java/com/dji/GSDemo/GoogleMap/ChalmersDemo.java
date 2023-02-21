package com.dji.GSDemo.GoogleMap;

import android.Manifest;
import android.content.IntentFilter;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import androidx.core.app.ActivityCompat;
import androidx.fragment.app.FragmentActivity;

import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;


public class ChalmersDemo extends FragmentActivity implements View.OnClickListener{

    private FlightController flightController;
    private Button up, down, stop;
    private CommonCallbacks.CompletionCallback callback;

    @Override
    protected void onResume(){
        super.onResume();
//        initFlightController();
    }

    @Override
    protected void onPause(){
        super.onPause();
    }

    @Override
    protected void onDestroy(){
//        unregisterReceiver(mReceiver);
//        removeListener();
        super.onDestroy();
    }

    private void initUI() {
        up = (Button) findViewById(R.id.btn_up);
        down = (Button) findViewById(R.id.btn_down);
        stop = (Button) findViewById(R.id.btn_stop);

        up.setOnClickListener(this);
        down.setOnClickListener(this);
        stop.setOnClickListener(this);
    }


    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.btn_up:{
                setResultToToast("GOING UP!!!!!");

                flightController.turnOnMotors(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        setResultToToast(djiError.getDescription());
                    }
                });

                break;
            }
            case R.id.btn_down:{
                setResultToToast("GOING DOWN!!!!!");

                flightController.turnOffMotors(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        setResultToToast(djiError.getDescription());
                    }
                });

                break;
            }
            case R.id.btn_stop:{
                setResultToToast("STOP MOTHERFUCKER!");
                break;
            }

            default:
                break;
        }
    }

    private void setResultToToast(final String string){
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

//        if (flightController != null) {
//            flightController.setStateCallback(new FlightControllerState.Callback() {
//
//                @Override
//                public void onUpdate(FlightControllerState djiFlightControllerCurrentState) {
//                    droneLocationLat = djiFlightControllerCurrentState.getAircraftLocation().getLatitude();
//                    droneLocationLng = djiFlightControllerCurrentState.getAircraftLocation().getLongitude();
//                    droneAltitude = djiFlightControllerCurrentState.getAircraftLocation().getAltitude();
//                    updateDroneLocationData();
//                }
//            });
//        }
    }


    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        initFlightController();

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
