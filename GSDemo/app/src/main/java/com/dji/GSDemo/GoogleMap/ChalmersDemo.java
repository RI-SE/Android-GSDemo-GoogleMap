package com.dji.GSDemo.GoogleMap;

import android.Manifest;
import android.content.IntentFilter;
import android.graphics.Color;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.SlidingDrawer;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
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
import java.util.Iterator;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ConcurrentHashMap;

import dji.common.camera.SettingsDefinitions;
import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.GPSSignalLevel;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.activetrack.ActiveTrackMission;
import dji.common.mission.activetrack.ActiveTrackMissionEvent;
import dji.common.mission.activetrack.ActiveTrackMode;
import dji.common.mission.activetrack.ActiveTrackState;
import dji.common.mission.activetrack.ActiveTrackTargetState;
import dji.common.mission.activetrack.ActiveTrackTrackingState;
import dji.common.mission.activetrack.QuickShotMode;
import dji.common.mission.activetrack.SubjectSensingState;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionState;
import dji.common.model.LocationCoordinate2D;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.keysdk.CameraKey;
import dji.keysdk.DJIKey;
import dji.keysdk.FlightControllerKey;
import dji.keysdk.KeyManager;
import dji.keysdk.callback.ActionCallback;
import dji.keysdk.callback.SetCallback;
import dji.midware.data.model.P3.B;
import dji.midware.media.DJIVideoDataRecver;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.MissionControl;
import dji.sdk.mission.activetrack.ActiveTrackMissionOperatorListener;
import dji.sdk.mission.activetrack.ActiveTrackOperator;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;


import org.apache.commons.math3.*;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.DiagonalMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;


public class ChalmersDemo extends FragmentActivity implements TextureView.SurfaceTextureListener, View.OnClickListener, View.OnTouchListener, CompoundButton.OnCheckedChangeListener, ActiveTrackMissionOperatorListener {

    private static final int MAIN_CAMERA_INDEX = 0;
    private static final int INVAVID_INDEX = -1;
    private static final int MOVE_OFFSET = 20;

    private RelativeLayout.LayoutParams layoutParams;
    private Switch mAutoSensingSw;
    private Switch mQuickShotSw;
    private ImageButton mPushDrawerIb;
    private SlidingDrawer mPushInfoSd;
    private ImageButton mStopBtn;
    private ImageView mTrackingImage;
    private RelativeLayout mBgLayout;
    private TextView mPushInfoTv;
    private Switch mPushBackSw;
    private Switch mGestureModeSw;
    private ImageView mSendRectIV;
    private Button mConfigBtn;
    private Button mConfirmBtn;
    private Button mRejectBtn;

    private ActiveTrackOperator mActiveTrackOperator;
    private ActiveTrackMission mActiveTrackMission;
    private final DJIKey trackModeKey = FlightControllerKey.createFlightAssistantKey(FlightControllerKey.ACTIVE_TRACK_MODE);
    private ConcurrentHashMap<Integer, MultiTrackingView> targetViewHashMap = new ConcurrentHashMap<>();
    private int trackingIndex = INVAVID_INDEX;
    private boolean isAutoSensingSupported = false;
    private ActiveTrackMode startMode = ActiveTrackMode.TRACE; //Ändrare .TRACE till .SPOTLIGHT för att vi bara vill att kameran ska röra sig
    // UPDATE: Detta funkade inte så just nu vinklar sig drönaren men man manuellt styr drönaren vart den ska åka
    private QuickShotMode quickShotMode = QuickShotMode.UNKNOWN;

    private boolean isDrawingRect = false;
    float downX;
    float downY;

    private static final String TAG = MainActivity.class.getName();
    private String lastDroneState = "";
    private IsoDrone drone;
    private FlightController flightController;
    private FlightControllerState djiFlightControllerCurrentState;
    private Gimbal gimbal;
    private Camera camera;
    private DJICodecManager mCodecManager;
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;


    private WaypointMissionOperator instance;
    private WaypointMissionFinishedAction mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
    private WaypointMissionHeadingMode mHeadingMode = WaypointMissionHeadingMode.AUTO;
    private ArrayList<WaypointSetting> waypointSettings = new ArrayList<>();
    private ArrayList<Waypoint> waypointList = new ArrayList<>();
    public static WaypointMission.Builder waypointMissionBuilder;


    public CRSFactory crsFactory = new CRSFactory();
    public CoordinateReferenceSystem WGS84 = crsFactory.createFromParameters("WGS84", "+proj=longlat +datum=WGS84 +no_defs");


    View decorView;
    TextureView mVideoSurface;
    private TextView text_gps, text_lat, text_lon, text_alt;
    private Button testcircle, config, upload, start, stop, land;
    private Button btn_atos_con, btn_drone_con, btn_ip_address, btn_drone_state, clear_wps;


    private double droneLocationLat = 57.688859d, droneLocationLng = 11.978795d, droneAltitude = 0d; // Johanneberg
    private float mSpeed = 10.0f;
    private float altitude = 100.0f;
    private LatLng startLatLong;
    private boolean missionUploaded = false;

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
        isAutoSensingSupported = false;
        try {
            DJIVideoDataRecver.getInstance().setVideoDataListener(false, null);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (mActiveTrackOperator != null) {
            mActiveTrackOperator.removeListener(this);
        }

        if (mCodecManager != null) {
            mCodecManager.destroyCodec();
        }

        super.onDestroy();
//        unregisterReceiver(mReceiver);
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
                generateTestCircleCoordinates(new LatLng(droneLocationLat, droneLocationLng), 1, 1.5f, 1, 8, true);
                deployTraj();
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
            case R.id.recommended_configuration_btn:
                Log.wtf("recommended_configuration_btn: ", "Start the tracking operator");
                mActiveTrackOperator.setRecommendedConfiguration(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError error) {
                        setResultToToast("Set Recommended Config " + (error == null ? "Success" : error.getDescription()));
                    }
                });
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mConfigBtn.setVisibility(View.GONE);
                    }
                });
                break;

            case R.id.confirm_btn:
                boolean isAutoTracking =
                        isAutoSensingSupported &&
                                (mActiveTrackOperator.isAutoSensingEnabled() ||
                                        mActiveTrackOperator.isAutoSensingForQuickShotEnabled());
                if (isAutoTracking) {
                    startAutoSensingMission();
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mStopBtn.setVisibility(View.VISIBLE);
                            mRejectBtn.setVisibility(View.VISIBLE);
                            mConfirmBtn.setVisibility(View.INVISIBLE);
                        }
                    });
                } else {
                    trackingIndex = INVAVID_INDEX;
                    mActiveTrackOperator.acceptConfirmation(new CommonCallbacks.CompletionCallback() {

                        @Override
                        public void onResult(DJIError error) {
                            setResultToToast(error == null ? "Accept Confirm Success!" : error.getDescription());
                        }
                    });

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mStopBtn.setVisibility(View.VISIBLE);
                            mRejectBtn.setVisibility(View.VISIBLE);
                            mConfirmBtn.setVisibility(View.INVISIBLE);
                        }
                    });

                }
                break;

            case R.id.tracking_stop_btn:
                trackingIndex = INVAVID_INDEX;
                mActiveTrackOperator.stopTracking(new CommonCallbacks.CompletionCallback() {

                    @Override
                    public void onResult(DJIError error) {
                        setResultToToast(error == null ? "Stop track Success!" : error.getDescription());
                    }
                });

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if (mTrackingImage != null) {
                            mTrackingImage.setVisibility(View.INVISIBLE);
                            mSendRectIV.setVisibility(View.INVISIBLE);
                            mStopBtn.setVisibility(View.INVISIBLE);
                            mRejectBtn.setVisibility(View.INVISIBLE);
                            mConfirmBtn.setVisibility(View.VISIBLE);
                        }
                    }
                });
                break;

            case R.id.reject_btn:
                trackingIndex = INVAVID_INDEX;
                mActiveTrackOperator.rejectConfirmation(new CommonCallbacks.CompletionCallback() {

                    @Override
                    public void onResult(DJIError error) {

                        setResultToToast(error == null ? "Reject Confirm Success!" : error.getDescription());
                    }
                });
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mStopBtn.setVisibility(View.VISIBLE);
                        mRejectBtn.setVisibility(View.VISIBLE);
                        mConfirmBtn.setVisibility(View.INVISIBLE);
                    }
                });
                break;

            case R.id.tracking_drawer_control_ib:
                if (mPushInfoSd.isOpened()) {
                    mPushInfoSd.animateClose();
                } else {
                    mPushInfoSd.animateOpen();
                }
                break;

            default:
                break;
        }
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                isDrawingRect = false;
                downX = event.getX();
                downY = event.getY();
                break;

            case MotionEvent.ACTION_MOVE:
                Log.wtf("Action_Move", "Currently drawing rectangle");
                //setResultToToast("Action_Move");
                if (calcManhattanDistance(downX, downY, event.getX(), event.getY()) < MOVE_OFFSET && !isDrawingRect) {
                    trackingIndex = getTrackingIndex(downX, downY, targetViewHashMap);
                    if (targetViewHashMap.get(trackingIndex) != null) {
                        targetViewHashMap.get(trackingIndex).setBackgroundColor(Color.RED);
                    }
                    return true;
                }

                isDrawingRect = true;
                mSendRectIV.setVisibility(View.VISIBLE);
                int l = (int) (downX < event.getX() ? downX : event.getX());
                int t = (int) (downY < event.getY() ? downY : event.getY());
                int r = (int) (downX >= event.getX() ? downX : event.getX());
                int b = (int) (downY >= event.getY() ? downY : event.getY());
                mSendRectIV.setX(l);
                mSendRectIV.setY(t);
                mSendRectIV.getLayoutParams().width = r - l;
                mSendRectIV.getLayoutParams().height = b - t;
                mSendRectIV.requestLayout();
                break;

            case MotionEvent.ACTION_UP:
                if (mGestureModeSw.isChecked()) {
                    setResultToToast("Please try to start Gesture Mode!");
                } else if (!isDrawingRect) {
                    if (targetViewHashMap.get(trackingIndex) != null) {
                        setResultToToast("Selected Index: " + trackingIndex + ",Please Confirm it!");
                        targetViewHashMap.get(trackingIndex).setBackgroundColor(Color.TRANSPARENT);
                    }
                } else {
                    RectF rectF = getActiveTrackRect(mSendRectIV);

                    mActiveTrackMission = new ActiveTrackMission(rectF, startMode);
                    if (startMode == ActiveTrackMode.QUICK_SHOT) {
                        Log.wtf("StartMode", "ActiveTrackMode is set to QUICK_SHOT");
                        mActiveTrackMission.setQuickShotMode(quickShotMode);
                        //checkStorageStates();
                    }
                    mActiveTrackOperator.startTracking(mActiveTrackMission, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError error) {
                            if (error == null) {
                                isDrawingRect = false;
                            }
                            setResultToToast("Start Tracking: " + (error == null
                                    ? "Success"
                                    : error.getDescription()));
                        }
                    });
                    mSendRectIV.setVisibility(View.INVISIBLE);
                    clearCurrentView();
                }
                break;

            default:
                break;
        }

        return true;
    }

    @Override
    public void onCheckedChanged(CompoundButton compoundButton, final boolean isChecked) {
        Log.wtf("onCheckedChanged: ", "Changed some of the configurations");
        if (mActiveTrackOperator == null) {
            return;
        }
        switch (compoundButton.getId()) {
            case R.id.set_multitracking_enabled:
                startMode = ActiveTrackMode.TRACE;  //Ändrare .TRACE till .SPOTLIGHT för att vi bara vill att kameran ska röra sig.
                // UPDATE: Detta funkade inte så just nu vinklar sig drönaren men man manuellt styr drönaren vart den ska åka
                quickShotMode = QuickShotMode.UNKNOWN;
                setAutoSensingEnabled(isChecked);
                break;
            case R.id.set_multiquickshot_enabled:
                startMode = ActiveTrackMode.QUICK_SHOT;
                quickShotMode = QuickShotMode.CIRCLE;
                //checkStorageStates();
                setAutoSensingForQuickShotEnabled(isChecked);
                break;
            case R.id.tracking_pull_back_tb:
                mActiveTrackOperator.setRetreatEnabled(isChecked, new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError error) {
                        if (error != null) {
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    mPushBackSw.setChecked(!isChecked);
                                }
                            });
                        }
                        setResultToToast("Set Retreat Enabled: " + (error == null
                                ? "Success"
                                : error.getDescription()));
                    }
                });
                break;
            case R.id.tracking_in_gesture_mode:
                mActiveTrackOperator.setGestureModeEnabled(isChecked, new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError error) {
                        if (error != null) {
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    mGestureModeSw.setChecked(!isChecked);
                                }
                            });
                        }
                        setResultToToast("Set GestureMode Enabled: " + (error == null
                                ? "Success"
                                : error.getDescription()));
                    }
                });
                break;
            default:
                break;
        }
    }

    private void initUI() {

        layoutParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT,
                RelativeLayout.LayoutParams.MATCH_PARENT);
        mPushDrawerIb = (ImageButton) findViewById(R.id.tracking_drawer_control_ib);
        mPushInfoSd = (SlidingDrawer) findViewById(R.id.tracking_drawer_sd);
        mPushInfoTv = (TextView) findViewById(R.id.tracking_push_tv);
        mBgLayout = (RelativeLayout) findViewById(R.id.tracking_bg_layout);
        mSendRectIV = (ImageView) findViewById(R.id.tracking_send_rect_iv);
        mTrackingImage = (ImageView) findViewById(R.id.tracking_rst_rect_iv);
        mConfirmBtn = (Button) findViewById(R.id.confirm_btn);
        mStopBtn = (ImageButton) findViewById(R.id.tracking_stop_btn);
        mRejectBtn = (Button) findViewById(R.id.reject_btn);
        mConfigBtn = (Button) findViewById(R.id.recommended_configuration_btn);
        mAutoSensingSw = (Switch) findViewById(R.id.set_multitracking_enabled);
        mQuickShotSw = (Switch) findViewById(R.id.set_multiquickshot_enabled);
        mPushBackSw = (Switch) findViewById(R.id.tracking_pull_back_tb);
        mGestureModeSw = (Switch) findViewById(R.id.tracking_in_gesture_mode);

        mAutoSensingSw.setChecked(false);
        mGestureModeSw.setChecked(false);
        mQuickShotSw.setChecked(false);
        mPushBackSw.setChecked(false);

        mAutoSensingSw.setOnCheckedChangeListener(this);
        mQuickShotSw.setOnCheckedChangeListener(this);
        mPushBackSw.setOnCheckedChangeListener(this);
        mGestureModeSw.setOnCheckedChangeListener(this);

        mBgLayout.setOnTouchListener(this);
        mConfirmBtn.setOnClickListener(this);
        mStopBtn.setOnClickListener(this);
        mRejectBtn.setOnClickListener(this);
        mConfigBtn.setOnClickListener(this);
        mPushDrawerIb.setOnClickListener(this);


        btn_atos_con = (Button) findViewById(R.id.btn_atos_con);
        btn_drone_con = (Button) findViewById(R.id.btn_drone_con);
        btn_drone_state = (Button) findViewById(R.id.btn_drone_state);
        btn_ip_address = (Button) findViewById(R.id.btn_ip_address);

        btn_ip_address.setText(Utils.getIPAddress(true));
        btn_drone_state.setText("State: Undefined");

        text_gps = (TextView) findViewById(R.id.text_gps);
        text_lat = (TextView) findViewById(R.id.text_lat);
        text_lon = (TextView) findViewById(R.id.text_lon);
        text_alt = (TextView) findViewById(R.id.text_alt);

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

        mVideoSurface = (TextureView) findViewById(R.id.video_preview_surface);
        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);

        }
    }

    private void initMissionManager() {
        mActiveTrackOperator = MissionControl.getInstance().getActiveTrackOperator();
        if (mActiveTrackOperator == null) {
            return;
        }

        mActiveTrackOperator.addListener(this);
        mAutoSensingSw.setChecked(mActiveTrackOperator.isAutoSensingEnabled());
        mQuickShotSw.setChecked(mActiveTrackOperator.isAutoSensingForQuickShotEnabled());
        mGestureModeSw.setChecked(mActiveTrackOperator.isGestureModeEnabled());
        mActiveTrackOperator.getRetreatEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
            @Override
            public void onSuccess(final Boolean aBoolean) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mPushBackSw.setChecked(aBoolean);
                    }
                });
            }

            @Override
            public void onFailure(DJIError error) {
                setResultToToast("can't get retreat enable state " + error.getDescription());
            }
        });
    }
    private double calcManhattanDistance(double point1X, double point1Y, double point2X,
                                         double point2Y) {
        Log.wtf("calcManhattanDistance: ","");
        return Math.abs(point1X - point2X) + Math.abs(point1Y - point2Y);
    }

    /**
     * @return
     */
    private int getTrackingIndex(final float x, final float y,
                                 final ConcurrentHashMap<Integer, MultiTrackingView> multiTrackinghMap) {
        Log.wtf(TAG, "getTrackingIndex");
        setResultToToast("getTrackingIndex");
        //Log.wtf("MultiTrackinghMap: ", multiTrackinghMap.toString());
        if (multiTrackinghMap == null || multiTrackinghMap.isEmpty()) {
            Log.wtf("MultiTrackinghMap: ", "MultiTrackingMap is Null or Empty");
            //setResultToToast("MultiTrackinghMap: " + multiTrackinghMap.toString());
            return INVAVID_INDEX;
        }

        float l, t, r, b;
        for (Map.Entry<Integer, MultiTrackingView> vo : multiTrackinghMap.entrySet()) {
            int key = vo.getKey().intValue();
            MultiTrackingView view = vo.getValue();
            l = view.getX();
            t = view.getY();
            r = (view.getX() + (view.getWidth() / 2));
            b = (view.getY() + (view.getHeight() / 2));

            if (x >= l && y >= t && x <= r && y <= b) {
                return key;
            }
        }
        return INVAVID_INDEX;
    }

    /**
     * Get ActiveTrack RectF
     *
     * @param iv
     * @return
     */
    private RectF getActiveTrackRect(View iv) {
        Log.wtf(TAG, "getActiveTrackRect");
        setResultToToast("getActiveTrackRect");
        View parent = (View) iv.getParent();
        return new RectF(
                ((float) iv.getLeft() + iv.getX()) / (float) parent.getWidth(),
                ((float) iv.getTop() + iv.getY()) / (float) parent.getHeight(),
                ((float) iv.getRight() + iv.getX()) / (float) parent.getWidth(),
                ((float) iv.getBottom() + iv.getY()) / (float) parent.getHeight());
    }

    /**
     * Post Result RectF
     *
     * @param iv
     * @param rectF
     * @param targetState
     */
    private void postResultRect(final ImageView iv, final RectF rectF,
                                final ActiveTrackTargetState targetState) {

        View parent = (View) iv.getParent();
        RectF trackingRect = rectF;

        final int l = (int) ((trackingRect.centerX() - trackingRect.width() / 2) * parent.getWidth());
        final int t = (int) ((trackingRect.centerY() - trackingRect.height() / 2) * parent.getHeight());
        final int r = (int) ((trackingRect.centerX() + trackingRect.width() / 2) * parent.getWidth());
        final int b = (int) ((trackingRect.centerY() + trackingRect.height() / 2) * parent.getHeight());
        ChalmersDemo.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Log.wtf("postResultRect: ", targetState.toString());
                mTrackingImage.setVisibility(View.VISIBLE);
                if ((targetState == ActiveTrackTargetState.CANNOT_CONFIRM)
                        || (targetState == ActiveTrackTargetState.UNKNOWN)) {
                    iv.setImageResource(R.drawable.visual_track_cannotconfirm);
                } else if (targetState == ActiveTrackTargetState.WAITING_FOR_CONFIRMATION) {
                    iv.setImageResource(R.drawable.visual_track_needconfirm);
                } else if (targetState == ActiveTrackTargetState.TRACKING_WITH_LOW_CONFIDENCE) {
                    iv.setImageResource(R.drawable.visual_track_lowconfidence);
                } else if (targetState == ActiveTrackTargetState.TRACKING_WITH_HIGH_CONFIDENCE) {
                    iv.setImageResource(R.drawable.visual_track_highconfidence);
                }
                iv.setX(l);
                iv.setY(t);
                iv.getLayoutParams().width = r - l;
                iv.getLayoutParams().height = b - t;
                iv.requestLayout();
            }
        });
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


    private void setResultToToast(final String string) {
        ChalmersDemo.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(ChalmersDemo.this, string, Toast.LENGTH_SHORT).show();
            }
        });
    }

    /**
     * Confim Mission by Index
     */
    private void startAutoSensingMission() {
        Log.wtf("startAutoSensingMission:", "Check if trackIndex is valid...");
        if (trackingIndex != INVAVID_INDEX) {
            Log.wtf("startAutoSensingMission:", "trackIndex valid!!!");
            ActiveTrackMission mission = new ActiveTrackMission(null, startMode);
            mission.setQuickShotMode(quickShotMode);
            mission.setTargetIndex(trackingIndex);
            mActiveTrackOperator.startAutoSensingMission(mission, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    if (error == null) {
                        setResultToToast("Accept Confim index: " + trackingIndex + " Success!");
                        trackingIndex = INVAVID_INDEX;
                    } else {
                        setResultToToast(error.getDescription());
                    }
                }
            });
        }
    }


    private void switchStorageLocation(final SettingsDefinitions.StorageLocation storageLocation) {
        KeyManager keyManager = KeyManager.getInstance();
        DJIKey storageLoactionkey = CameraKey.create(CameraKey.CAMERA_STORAGE_LOCATION, MAIN_CAMERA_INDEX);

        if (storageLocation == SettingsDefinitions.StorageLocation.INTERNAL_STORAGE) {
            keyManager.setValue(storageLoactionkey, SettingsDefinitions.StorageLocation.SDCARD, new SetCallback() {
                @Override
                public void onSuccess() {
                    setResultToToast("Change to SD card Success!");
                }

                @Override
                public void onFailure(@NonNull DJIError error) {
                    setResultToToast(error.getDescription());
                }
            });
        } else {
            keyManager.setValue(storageLoactionkey, SettingsDefinitions.StorageLocation.INTERNAL_STORAGE, new SetCallback() {
                @Override
                public void onSuccess() {
                    setResultToToast("Change to Interal Storage Success!");
                }

                @Override
                public void onFailure(@NonNull DJIError error) {
                    setResultToToast(error.getDescription());
                }
            });
        }
    }
    /**
     * Clear MultiTracking View
     */

    private void clearCurrentView() {
        if (targetViewHashMap != null && !targetViewHashMap.isEmpty()) {
            Iterator<Map.Entry<Integer, MultiTrackingView>> it = targetViewHashMap.entrySet().iterator();
            while (it.hasNext()) {
                Map.Entry<Integer, MultiTrackingView> entry = it.next();
                final MultiTrackingView view = entry.getValue();
                it.remove();
                ChalmersDemo.this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mBgLayout.removeView(view);
                    }
                });
            }
        }
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
                    Log.wtf("LOCATION GPS", String.valueOf(gps));
                    Log.wtf("LOCATION LAT", String.valueOf(droneLocationLat));
                    Log.wtf("LOCATION LONG", String.valueOf(droneLocationLng));
                    Log.wtf("LOCATION ALT", String.valueOf(droneAltitude));
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


    @Override
    public void onUpdate(ActiveTrackMissionEvent event) {
        StringBuffer sb = new StringBuffer();
        String errorInformation = (event.getError() == null ? "null" : event.getError().getDescription()) + "\n";
        String currentState = event.getCurrentState() == null ? "null" : event.getCurrentState().getName();
        String previousState = event.getPreviousState() == null ? "null" : event.getPreviousState().getName();

        ActiveTrackTargetState targetState = ActiveTrackTargetState.UNKNOWN;
        if (event.getTrackingState() != null) {
            Log.wtf("onUpdate: ", "Tracking state is confirmed!!");
            targetState = event.getTrackingState().getState();
        }
        Utils.addLineToSB(sb, "CurrentState: ", currentState);
        Utils.addLineToSB(sb, "PreviousState: ", previousState);
        Utils.addLineToSB(sb, "TargetState: ", targetState);
        Utils.addLineToSB(sb, "Error:", errorInformation);

        Object value = KeyManager.getInstance().getValue(trackModeKey);
        if (value instanceof ActiveTrackMode) {
            Utils.addLineToSB(sb, "TrackingMode:", value.toString());
        }

        ActiveTrackTrackingState trackingState = event.getTrackingState();
        if (trackingState != null) {
            final SubjectSensingState[] targetSensingInformations = trackingState.getAutoSensedSubjects();
            if (targetSensingInformations != null) {
                for (SubjectSensingState subjectSensingState : targetSensingInformations) {
                    RectF trackingRect = subjectSensingState.getTargetRect();
                    if (trackingRect != null) {
                        Utils.addLineToSB(sb, "Rect center x: ", trackingRect.centerX());
                        Utils.addLineToSB(sb, "Rect center y: ", trackingRect.centerY());
                        Utils.addLineToSB(sb, "Rect Width: ", trackingRect.width());
                        Utils.addLineToSB(sb, "Rect Height: ", trackingRect.height());
                        Utils.addLineToSB(sb, "Reason", trackingState.getReason().name());
                        Log.wtf("Get Reason", trackingState.getReason().name());
                        Utils.addLineToSB(sb, "Target Index: ", subjectSensingState.getIndex());
                        Utils.addLineToSB(sb, "Target Type", subjectSensingState.getTargetType().name());
                        Log.wtf("Target Type", subjectSensingState.getTargetType().name());
                        Utils.addLineToSB(sb, "Target State", subjectSensingState.getState().name());
                        isAutoSensingSupported = true;
                    }
                }
            } else {
                RectF trackingRect = trackingState.getTargetRect();
                if (trackingRect != null) {
                    Utils.addLineToSB(sb, "Rect center x: ", trackingRect.centerX());
                    Utils.addLineToSB(sb, "Rect center y: ", trackingRect.centerY());
                    Utils.addLineToSB(sb, "Rect Width: ", trackingRect.width());
                    Utils.addLineToSB(sb, "Rect Height: ", trackingRect.height());
                    Utils.addLineToSB(sb, "Reason", trackingState.getReason().name());
                    Log.wtf("Get Reason", trackingState.getReason().name());
                    Utils.addLineToSB(sb, "Target Index: ", trackingState.getTargetIndex());
                    Utils.addLineToSB(sb, "Target Type", trackingState.getType().name());
                    Log.wtf("Target Type", trackingState.getType().name());
                    Utils.addLineToSB(sb, "Target State", trackingState.getState().name());
                    isAutoSensingSupported = false;
                }
                clearCurrentView();
            }
        }

        updateActiveTrackRect(mTrackingImage, event);
        updateButtonVisibility(event);
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
            updateStateButton(lastDroneState, Color.LTGRAY);
            setDroneHomeLocation();
        } else if (drone.getCurrentStateName().equals("PreArming") && lastDroneState != "PreArming") {
            Log.wtf("Error", "PreArming");
            lastDroneState = "PreArming";
            updateStateButton(lastDroneState, Color.GRAY);

        } else if (drone.getCurrentStateName().equals("Armed") && lastDroneState != "Armed") {
            Log.wtf("Error", "Armed");
            runOnUiThread(new Runnable() {
                @Override
                public void run() {

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
                    deployTraj();
                }
            });

            Log.wtf("Error", "Armed");
            lastDroneState = "Armed";
            startDroneMotors();
        } else if (drone.getCurrentStateName().equals("Disarmed")) {

            Log.wtf("Error", "Disarmed");
            lastDroneState = "Disarmed";
            sendMonr();
            updateStateButton(lastDroneState, Color.YELLOW);
        } else if (drone.getCurrentStateName().equals("PreRunning") && lastDroneState != "PreRunning") {
            Log.wtf("Error", "PreRunning");
            lastDroneState = "PreRunning";

            updateStateButton(lastDroneState, Color.DKGRAY);
        } else if (drone.getCurrentStateName().equals("Running") && lastDroneState != "Running") {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    resumeWaypointMission();
                }});
            Log.wtf("Error", "Running");
            lastDroneState = "Running";
            updateStateButton(lastDroneState, Color.RED);
            startDroneRecording();

        } else if (drone.getCurrentStateName().equals("NormalStop") && lastDroneState != "NormalStop") {
            setResultToToast("NormalStop");
            lastDroneState = "NormalStop";
            updateStateButton(lastDroneState, Color.CYAN);
            stopDroneRecording();
        } else if (drone.getCurrentStateName().equals("EmergencyStop") && lastDroneState != "EmergencyStop") {
            Log.wtf("Error", "EmergencyStop");
            lastDroneState = "EmergencyStop";
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    stopWaypointMission();
                }});
            updateStateButton(lastDroneState, Color.BLACK);
            stopDroneRecording();
        }
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

        monrPos.setHeading_rad(headingToYaw(flightController.getState().getAttitude().yaw) * Math.PI/180);
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
            this.waypointSettings.get(i).geo.z = trajectory.get(i).getPos().getZCoord_m();
            this.waypointSettings.get(i).speed = (float) trajectory.get(i).getSpd().getLongitudinal_m_s(); //Possible lossy conversion?
            this.waypointSettings.get(i).radius = radii.getEntry(i) - 0.01;
        }

        //add landing point
        // WaypointSetting wps = new WaypointSetting(coordCartToGeo(origin, new ProjCoordinate(0, 0)), new ProjCoordinate());
        // wps.heading = 0;
        // wps.speed = 15;
        // wps.geo.z = 6;
        // this.waypointSettings.add(wps);
    }


    private void deployTraj() {
        Log.wtf("Error", "Deploying traj");
        waypointMissionBuilder = new WaypointMission.Builder();

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
            waypointMissionBuilder.waypointList(waypointList).waypointCount(waypointList.size());
        }
        setResultToToast("Number of waypoints " + this.waypointSettings.size());
        mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
        mSpeed = 5.0f;
        mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;
        altitude = (float) this.waypointSettings.get(0).geo.z;
        configWayPointMission();
        startLatLong = new LatLng(droneLocationLat, droneLocationLng);
        uploadWayPointMission();
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
        if (instance == null) {
            if (DJISDKManager.getInstance().getMissionControl() != null) {
                instance = DJISDKManager.getInstance().getMissionControl().getWaypointMissionOperator();
            }
        }
        return instance;
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
                    setResultToToast("Mission upload successfully!");
                    missionUploaded = true;
//                    startWaypointMission();

                } else {
                    missionUploaded = false;
                    setResultToToast("Mission upload failed, error: " + error.getDescription() + " retrying...");
                    getWaypointMissionOperator().retryUploadMission(null);
                }
            }
        });

    }

    private void startWaypointMission() {
        if (flightController != null) {
            getWaypointMissionOperator().startMission(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    setResultToToast("Mission Start: " + (error == null ? "Successfully" : error.getDescription()));
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

        if (djiFlightControllerCurrentState.isLandingConfirmationNeeded()) {
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

    /**
     * Update ActiveTrack Rect
     *
     * @param iv
     * @param event
     */
    private void updateActiveTrackRect(final ImageView iv, final ActiveTrackMissionEvent event) {
        if (iv == null || event == null) {
            return;
        }

        ActiveTrackTrackingState trackingState = event.getTrackingState();
        if (trackingState != null) {
            if (trackingState.getAutoSensedSubjects() != null) {
                final SubjectSensingState[] targetSensingInformations = trackingState.getAutoSensedSubjects();
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        updateMultiTrackingView(targetSensingInformations);
                    }
                });
            } else {
                RectF trackingRect = trackingState.getTargetRect();
                ActiveTrackTargetState trackTargetState = trackingState.getState();
                postResultRect(iv, trackingRect, trackTargetState);
            }
        }

    }

    private void updateButtonVisibility(final ActiveTrackMissionEvent event) {
        ActiveTrackState state = event.getCurrentState();
        Log.wtf("updateButtonVisibilty: ", state.toString());
        if (state == ActiveTrackState.AUTO_SENSING ||
                state == ActiveTrackState.AUTO_SENSING_FOR_QUICK_SHOT ||
                state == ActiveTrackState.WAITING_FOR_CONFIRMATION) {
            TrackingTestActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mStopBtn.setVisibility(View.VISIBLE);
                    mStopBtn.setClickable(true);
                    mConfirmBtn.setVisibility(View.VISIBLE);
                    mConfirmBtn.setClickable(true);
                    mRejectBtn.setVisibility(View.VISIBLE);
                    mRejectBtn.setClickable(true);
                    mConfigBtn.setVisibility(View.GONE);
                }
            });
        } else if (state == ActiveTrackState.AIRCRAFT_FOLLOWING ||
                state == ActiveTrackState.ONLY_CAMERA_FOLLOWING ||
                state == ActiveTrackState.FINDING_TRACKED_TARGET ||
                state == ActiveTrackState.CANNOT_CONFIRM ||
                state == ActiveTrackState.PERFORMING_QUICK_SHOT) {
            TrackingTestActivity.this.runOnUiThread(new Runnable() {

                @Override
                public void run() {
                    mStopBtn.setVisibility(View.VISIBLE);
                    mStopBtn.setClickable(true);
                    mConfirmBtn.setVisibility(View.INVISIBLE);
                    mConfirmBtn.setClickable(false);
                    mRejectBtn.setVisibility(View.VISIBLE);
                    mRejectBtn.setClickable(true);
                    mConfigBtn.setVisibility(View.GONE);
                }
            });
        } else {
            TrackingTestActivity.this.runOnUiThread(new Runnable() {

                @Override
                public void run() {
                    mStopBtn.setVisibility(View.INVISIBLE);
                    mStopBtn.setClickable(false);
                    mConfirmBtn.setVisibility(View.INVISIBLE);
                    mConfirmBtn.setClickable(false);
                    mRejectBtn.setVisibility(View.INVISIBLE);
                    mRejectBtn.setClickable(false);
                    mTrackingImage.setVisibility(View.INVISIBLE);
                }
            });
        }
    }

    /**
     * PostMultiResult
     *
     * @param iv
     * @param rectF
     * @param information
     */
    private void postMultiResultRect(final MultiTrackingView iv, final RectF rectF,
                                     final SubjectSensingState information) {
        View parent = (View) iv.getParent();
        RectF trackingRect = rectF;

        final int l = (int) ((trackingRect.centerX() - trackingRect.width() / 2) * parent.getWidth());
        final int t = (int) ((trackingRect.centerY() - trackingRect.height() / 2) * parent.getHeight());
        final int r = (int) ((trackingRect.centerX() + trackingRect.width() / 2) * parent.getWidth());
        final int b = (int) ((trackingRect.centerY() + trackingRect.height() / 2) * parent.getHeight());

        TrackingTestActivity.this.runOnUiThread(new Runnable() {

            @Override
            public void run() {
                mTrackingImage.setVisibility(View.INVISIBLE);
                iv.setX(l);
                iv.setY(t);
                iv.getLayoutParams().width = r - l;
                iv.getLayoutParams().height = b - t;
                iv.requestLayout();
                iv.updateView(information);
            }
        });
    }

    /**
     * Update MultiTrackingView
     *
     * @param targetSensingInformations
     */
    private void updateMultiTrackingView(final SubjectSensingState[] targetSensingInformations) {
        Log.wtf("updateMultiTrackingView: ", "Currently running multiple events");
        ArrayList<Integer> indexs = new ArrayList<>();
        for (SubjectSensingState target : targetSensingInformations) {
            indexs.add(target.getIndex());
            if (targetViewHashMap.containsKey(target.getIndex())) {

                MultiTrackingView targetView = targetViewHashMap.get(target.getIndex());
                postMultiResultRect(targetView, target.getTargetRect(), target);
            } else {
                MultiTrackingView trackingView = new MultiTrackingView(TrackingTestActivity.this);
                mBgLayout.addView(trackingView, layoutParams);
                targetViewHashMap.put(target.getIndex(), trackingView);
            }
        }

        ArrayList<Integer> missingIndexs = new ArrayList<>();
        for (Integer key : targetViewHashMap.keySet()) {
            boolean isDisappeared = true;
            for (Integer index : indexs) {
                if (index.equals(key)) {
                    isDisappeared = false;
                    break;
                }
            }

            if (isDisappeared) {
                missingIndexs.add(key);
            }
        }

        for (Integer i : missingIndexs) {
            MultiTrackingView view = targetViewHashMap.remove(i);
            mBgLayout.removeView(view);
        }
    }


    /**
     * Enable MultiTracking
     *
     * @param isChecked
     */
    private void setAutoSensingEnabled(final boolean isChecked) {
        if (mActiveTrackOperator != null) {
            if (isChecked) {
                startMode = ActiveTrackMode.TRACE; //Ändrare .TRACE till .SPOTLIGHT för att vi bara vill att kameran ska röra sig
                // UPDATE: Detta funkade inte så just nu vinklar sig drönaren men man manuellt styr drönaren vart den ska åka
                mActiveTrackOperator.enableAutoSensing(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError error) {
                        if (error != null) {
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    mAutoSensingSw.setChecked(!isChecked);
                                }
                            });
                        }
                        setResultToToast("Set AutoSensing Enabled " + (error == null ? "Success!" : error.getDescription()));
                    }
                });
            } else {
                disableAutoSensing();
            }
        }
    }

    /**
     * Enable QuickShotMode
     *
     * @param isChecked
     */
    private void setAutoSensingForQuickShotEnabled(final boolean isChecked) {
        if (mActiveTrackOperator != null) {
            if (isChecked) {
                mActiveTrackOperator.enableAutoSensingForQuickShot(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError error) {
                        if (error != null) {
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    mQuickShotSw.setChecked(!isChecked);
                                }
                            });
                        }
                        setResultToToast("Set QuickShot Enabled " + (error == null ? "Success!" : error.getDescription()));
                    }
                });

            } else {
                disableAutoSensing();
            }

        }
    }

    /**
     * Disable AutoSensing
     */
    private void disableAutoSensing() {
        if (mActiveTrackOperator != null) {
            mActiveTrackOperator.disableAutoSensing(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    if (error == null) {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                mConfirmBtn.setVisibility(View.INVISIBLE);
                                mStopBtn.setVisibility(View.INVISIBLE);
                                mRejectBtn.setVisibility(View.INVISIBLE);
                                mConfigBtn.setVisibility(View.VISIBLE);
                                isAutoSensingSupported = false;
                            }
                        });
                        clearCurrentView();
                    }
                    setResultToToast(error == null ? "Disable Auto Sensing Success!" : error.getDescription());
                }
            });
        }
    }

    private boolean isSDCardReady(int index) {
        Log.wtf(TAG, "isSDCardReady");
        KeyManager keyManager = KeyManager.getInstance();

        return ((Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_INSERTED, index))
                && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_INITIALIZING, index))
                && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_READ_ONLY, index))
                && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_HAS_ERROR, index))
                && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_FULL, index))
                && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_BUSY, index))
                && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_FORMATTING, index))
                && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_INVALID_FORMAT, index))
                && (Boolean) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_IS_VERIFIED, index))
                && (Long) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_AVAILABLE_CAPTURE_COUNT, index)) > 0L
                && (Integer) keyManager.getValue(CameraKey.create(CameraKey.SDCARD_AVAILABLE_RECORDING_TIME_IN_SECONDS, index)) > 0);
    }



    /**
     * determine Interal Storage is or not Ready
     *
     * @param index
     * @return
     */


    private boolean isInteralStorageReady(int index) {
        Log.wtf(TAG, "isInteralStorageReady");
        KeyManager keyManager = KeyManager.getInstance();

        boolean isInternalSupported = (boolean)
                keyManager.getValue(CameraKey.create(CameraKey.IS_INTERNAL_STORAGE_SUPPORTED, index));
        if (isInternalSupported) {
            return ((Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_INSERTED, index))
                    && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_INITIALIZING, index))
                    && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_READ_ONLY, index))
                    && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_HAS_ERROR, index))
                    && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_FULL, index))
                    && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_BUSY, index))
                    && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_FORMATTING, index))
                    && !(Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_INVALID_FORMAT, index))
                    && (Boolean) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_IS_VERIFIED, index))
                    && (Long) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_AVAILABLE_CAPTURE_COUNT, index)) > 0L
                    && (Integer) keyManager.getValue(CameraKey.create(CameraKey.INNERSTORAGE_AVAILABLE_RECORDING_TIME_IN_SECONDS, index)) > 0);
        }
        return false;
    }



    /**
     * Check Storage States
     */


    private void checkStorageStates() {
        Log.wtf(TAG, "checkStorageStates");
        KeyManager keyManager = KeyManager.getInstance();
        DJIKey storageLocationkey = CameraKey.create(CameraKey.CAMERA_STORAGE_LOCATION, MAIN_CAMERA_INDEX);
        Object storageLocationObj = keyManager.getValue(storageLocationkey);
        SettingsDefinitions.StorageLocation storageLocation = SettingsDefinitions.StorageLocation.INTERNAL_STORAGE;

        if (storageLocationObj instanceof SettingsDefinitions.StorageLocation){
            storageLocation = (SettingsDefinitions.StorageLocation) storageLocationObj;
        }

        if (storageLocation == SettingsDefinitions.StorageLocation.INTERNAL_STORAGE) {
            if (!isInteralStorageReady(MAIN_CAMERA_INDEX) && isSDCardReady(MAIN_CAMERA_INDEX)) {
                switchStorageLocation(SettingsDefinitions.StorageLocation.SDCARD);
            }
        }

        if (storageLocation == SettingsDefinitions.StorageLocation.SDCARD) {
            if (!isSDCardReady(MAIN_CAMERA_INDEX) && isInteralStorageReady(MAIN_CAMERA_INDEX)) {
                switchStorageLocation(SettingsDefinitions.StorageLocation.INTERNAL_STORAGE);
            }
        }

        DJIKey isRecordingKey = CameraKey.create(CameraKey.IS_RECORDING, MAIN_CAMERA_INDEX);
        Object isRecording = keyManager.getValue(isRecordingKey);
        if (isRecording instanceof Boolean) {
            if (((Boolean) isRecording).booleanValue()) {
                keyManager.performAction(CameraKey.create(CameraKey.STOP_RECORD_VIDEO, MAIN_CAMERA_INDEX), new ActionCallback() {
                    @Override
                    public void onSuccess() {
                        setResultToToast("Stop Recording Success!");
                    }

                    @Override
                    public void onFailure(@NonNull DJIError error) {
                        setResultToToast("Stop Recording Fail，Error " + error.getDescription());
                    }
                });
            }
        }
    }
}
