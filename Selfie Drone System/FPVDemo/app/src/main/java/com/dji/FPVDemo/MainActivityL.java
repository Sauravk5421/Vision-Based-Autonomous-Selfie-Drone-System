package com.dji.FPVDemo;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.media.MediaScannerConnection;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import android.view.TextureView;
import android.view.TextureView.SurfaceTextureListener;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.Chronometer;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import androidx.annotation.NonNull;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.camera.SettingsDefinitions;
import dji.common.camera.SystemState;
import dji.common.error.DJIError;
import dji.common.flightcontroller.simulator.SimulatorState;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.CapabilityKey;
import dji.common.gimbal.GimbalMode;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.common.util.DJIParamCapability;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;

import static org.opencv.imgproc.Imgproc.MORPH_RECT;

public class MainActivityL extends Activity implements SurfaceTextureListener,OnClickListener{

    private static final String TAG = MainActivityL.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;

    // Codec for video live view
    protected DJICodecManager mCodecManager = null;
    protected TextureView mVideoSurface = null;
    protected ImageView mImageSurface;
    private Button mCaptureBtn, mShootPhotoModeBtn, mRecordVideoModeBtn;
    private ToggleButton mRecordBtn;
    private TextView recordingTime;

    private FlightController mFlightController;
    protected TextView mConnectStatusTextView;
    private Button mBtnEnableVirtualStick;
    private Button mBtnDisableVirtualStick;
    private Button mBtnMoveup;
    private Button mBtnMovedown;
    private Button mBtnset;
    private Button mBtnvalue;
    private Button mBtnstart;
    private Button mBtnTakeOff;
    private Button mBtnLand;

    private OnScreenJoystick mScreenJoystickRight;
    private OnScreenJoystick mScreenJoystickLeft;

    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;

    private float mPitch;
    private float mRoll;
    private float mYaw;
    private float mThrottle;

    double pX;
    double pY;
    double distancex1,distancey1,distancez1;
    double distancex2,distancey2,distancez2;

    private Chronometer chronometer;
    private boolean running;


    protected final static int DISABLE = 0;
    private Button yawMinBtn;
    private Button yawMaxBtn;
    private Button rollMinBtn;
    private Button rollMaxBtn;
    private TextView mTextView;

    private Handler handler;
    private Gimbal gimbal = null;
    private int currentGimbalId = 0;

    double [] arr_rxr;
    double [] arr_ryr;
    double [] arr_rxb;
    double [] arr_ryb;
    double [] arr_rxg;
    double [] arr_ryg;
    double [] arr_rxbl;
    double [] arr_rybl;
    double px_r=0,py_r=0,px_bl=0,py_bl=0,px_g=0,py_g=0,px_b=0,py_b=0;
    double per_px_r=0,per_py_r=0,per_px_bl=0,per_py_bl=0,per_px_g=0,per_py_g=0,per_px_b=0,per_py_b=0;
    double tx, ty,tz,tpitch,troll,tyaw;
    double rtx,rty,rtz,rtroll,rtpitch,rtyaw;
    double rxr,ryr,rxb,ryb,rxg,ryg,rxbl,rybl;
    double rxrf,ryrf,rxbf,rybf,rxgf,rygf,rxblf,ryblf;
    double final_pitch,final_roll,final_yaw;
    double initial_pitch,initial_roll,initial_yaw;
    double movepitch,moveroll,moveyaw,distancex,distancey,distancez;
    double moveyaw2;
    int counter;
    int w,h;
    double ttyaw,timeyaw,time;
    protected StringBuffer stringBuffer;
    private float act_pitch,act_roll,act_yaw;
    Bitmap sourceBitmap;
    Bitmap displayBitmap;
    FileWriter writer;
    int kk = 0;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    break;
                }
                default: {
                    super.onManagerConnected(status);
                    break;
                }
            }
        }
    };

    static {
        if (OpenCVLoader.initDebug()){
            Log.d(TAG,"OpenCV loaded successfully");
        }else {
            Log.d(TAG,"OpenCV not loaded");

        }
    }

    private void initFlightController() {

        Aircraft aircraft = DJISimulatorApplication.getAircraftInstance();
        if (aircraft == null || !aircraft.isConnected()) {
            showToast("Disconnected");
            mFlightController = null;
            return;
        } else {
            mFlightController = aircraft.getFlightController();
            mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            mFlightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            mFlightController.getSimulator().setStateCallback(new SimulatorState.Callback() {
                @Override
                public void onUpdate(final SimulatorState stateData) {
                    new Handler(Looper.getMainLooper()).post(new Runnable() {
                        @Override
                        public void run() {
                            String yaw = String.format("%.2f", stateData.getYaw());
                            String pitch = String.format("%.2f", stateData.getPitch());
                            String roll = String.format("%.2f", stateData.getRoll());
                            String positionX = String.format("%.2f", stateData.getPositionX());
                            String positionY = String.format("%.2f", stateData.getPositionY());
                            String positionZ = String.format("%.2f", stateData.getPositionZ());
                            mTextView.setText("Yaw : " + yaw + ", Pitch : " + pitch + ", Roll : " + roll + "\n" + ", PosX : " + positionX +
                                    ", PosY : " + positionY +
                                    ", PosZ : " + positionZ);
                        }
                    });
                }
            });
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main2);
        chronometer = findViewById(R.id.chronometer);
        Intent intent = getIntent();
        tx = intent.getDoubleExtra(ColorDetection.EXTRA_NUMBER1,0);
        ty = intent.getDoubleExtra(ColorDetection.EXTRA_NUMBER2,0);
        tz = intent.getDoubleExtra(ColorDetection.EXTRA_NUMBER3,0);
        tpitch = intent.getDoubleExtra(ColorDetection.EXTRA_NUMBER4,0);
        tyaw = intent.getDoubleExtra(ColorDetection.EXTRA_NUMBER5,0);
        troll = intent.getDoubleExtra(ColorDetection.EXTRA_NUMBER6,0);


// template values
        distancex1 = tx;
        distancey1 = ty;
        distancez1 = tz;
        final_pitch = tpitch;
        final_yaw = tyaw;
        final_roll = troll;

        TextView theTextView1 = (TextView) findViewById(R.id.textView1);
        TextView theTextView2 = (TextView) findViewById(R.id.textView2);
        TextView theTextView3 = (TextView) findViewById(R.id.textView3);
        TextView theTextView4 = (TextView) findViewById(R.id.textView4);
        TextView theTextView5 = (TextView) findViewById(R.id.textView5);
        TextView theTextView6 = (TextView) findViewById(R.id.textView6);

// Template 6D

        theTextView1.setText("Roll: " + String.format("%.2f",troll));
        theTextView1.setTextColor(Color.RED);

        theTextView2.setText("Pitch: " + String.format("%.2f",tpitch));
        theTextView2.setTextColor(Color.RED);

        theTextView3.setText("Yaw: " + String.format("%.2f",tyaw));
        theTextView3.setTextColor(Color.RED);

        theTextView4.setText("X: " + String.format("%.2f",distancex1));
        theTextView4.setTextColor(Color.RED);

        theTextView5.setText("Y: " + String.format("%.2f",distancey1));
        theTextView5.setTextColor(Color.RED);

        theTextView6.setText("Z: " + String.format("%.2f",distancez1));
        theTextView6.setTextColor(Color.RED);

        handler = new Handler();
        initUI();
        dronestart();
        IntentFilter filter = new IntentFilter();
        filter.addAction(DJISimulatorApplication.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        mBtnset.setOnClickListener(new OnClickListener() {

            @Override
            public void onClick(View v) {
                setzero();
                resetgimbal();
            }
        });

        mBtnstart.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                Handler handler = new Handler();
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        SolvePNP();
                    }
                },(long)(1*1000 ) );
            }
        });

        mBtnMoveup.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                moveup();
                Handler handler = new Handler();
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        setzero();
                        saveImageToExternalStorage(sourceBitmap);
                    }
                },(long)2800 );
               // rotatecamera_reverse(timeyaw);
               //straight(distancex1,distancex2,distancey1,distancey2,distancez1,distancez2);
            }
        });


        mBtnMovedown.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                movedown();
            }
        });

        mBtnvalue.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {


 // Clock start
                if (!running) {
                    chronometer.setBase(SystemClock.elapsedRealtime());
                    chronometer.start();
                    running = true;
                }

// real time
                distancex2 = rtx + 0.20;    // a pattern error is added
                distancey2 = rty + 0.20;    // a pattern error is added
                distancez2 = rtz;
                initial_pitch = rtpitch;
                initial_yaw = rtyaw;
                initial_roll = rtroll;

// Pitch Calculation

                if(final_pitch > 0 && initial_pitch > 0) {
                    movepitch = initial_pitch - final_pitch;
                }

                if(final_pitch < 0 && initial_pitch < 0) {
                    movepitch = - (final_pitch - initial_pitch);
                }

                if(final_pitch < 0 && initial_pitch > 0) {
                    final_pitch = 180 - Math.abs(final_pitch);
                    initial_pitch = 180 - initial_pitch;
                    movepitch = (initial_pitch + final_pitch);
                    if(movepitch > 29)
                    {
                        movepitch = 30;
                    }

                }
                if(final_pitch > 0 && initial_pitch < 0) {
                    initial_pitch = 180 - Math.abs(initial_pitch);
                    final_pitch = 180 - final_pitch;
                    movepitch =  (initial_pitch + final_pitch);

                }
                moveroll= initial_roll - final_roll;
                rotatePitch((act_pitch - movepitch)/1.2);
                straight(distancex1,distancex2,distancey1,distancey2,distancez1,distancez2);

// //Show Rotate
//
//                TextView theTextView7 = (TextView) findViewById(R.id.textView7);
//                TextView theTextView8 = (TextView) findViewById(R.id.textView8);
//
//
//                theTextView7.setText(", " + String.format("%.2f",moveroll));
//                theTextView7.setTextColor(Color.RED);
//
//                theTextView8.setText(", " + String.format("%.2f",movepitch));
//                theTextView8.setTextColor(Color.RED);
//
////Show Speed
//
//                TextView theTextView24 = (TextView) findViewById(R.id.textView24);
//                TextView theTextView25 = (TextView) findViewById(R.id.textView25);
//
//                theTextView24.setText(" Pitch =  " + String.valueOf(0));
//                theTextView24.setTextColor(Color.RED);
//
//                theTextView25.setText(" Roll =  " + String.valueOf(0));
//                theTextView25.setTextColor(Color.RED);


            }
        });

        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if (mCodecManager != null) {
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };

        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            camera.setSystemStateCallback(new SystemState.Callback() {
                @Override
                public void onUpdate(SystemState cameraSystemState) {
                    if (null != cameraSystemState) {
                        int recordTime = cameraSystemState.getCurrentVideoRecordingTimeInSeconds();
                        int minutes = (recordTime % 3600) / 60;
                        int seconds = recordTime % 60;
                        final String timeString = String.format("%02d:%02d", minutes, seconds);
                        final boolean isVideoRecording = cameraSystemState.isRecording();
                        MainActivityL.this.runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                recordingTime.setText(timeString);
                                if (isVideoRecording){
                                    recordingTime.setVisibility(View.VISIBLE);
                                }else
                                {
                                    recordingTime.setVisibility(View.INVISIBLE);
                                }
                            }
                        });
                    }
                }
            });
        }
    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            updateTitleBar();
        }
    };

    private void updateTitleBar() {
        if(mConnectStatusTextView == null) return;
        boolean ret = false;
        BaseProduct product = DJISimulatorApplication.getProductInstance();
        if (product != null) {
            if(product.isConnected()) {
                //The product is connected
                mConnectStatusTextView.setText(DJISimulatorApplication.getProductInstance().getModel() + " Connected");
                ret = true;
            } else {
                if(product instanceof Aircraft) {
                    Aircraft aircraft = (Aircraft)product;
                    if(aircraft.getRemoteController() != null && aircraft.getRemoteController().isConnected()) {
                        // The product is not connected, but the remote controller is connected
                        mConnectStatusTextView.setText("only RC Connected");
                        ret = true;
                    }
                }
            }
        }
        if(!ret) {
            // The product or the remote controller are not connected.
            mConnectStatusTextView.setText("Disconnected");
        }
    }

    double incrementY(double pYI)
    {
        while ( pY<=1 )
        {
            pYI += 0.05;
            break;
        }
        return pYI;
    }

    double decrementY(double pYD)
    {
        while ( pY>=-1 )
        {
            pYD -= 0.05;
            break;
        }
        return pYD;
    }

    double incrementX(double pXI)
    {
        while ( pX<=1 )
        {
            pXI += 0.05;
            break;
        }
        return pXI;
    }

    double decrementX(double pXD)
    {
        while ( pX>=-1 )
        {
            pXD -= 0.05;
            break;
        }
        return pXD;
    }

// Drone x, y, z control using parameteric equation

    double straight(double distancex1,double distancex2,double distancey1,double distancey2,double distancez1,double distancez2) {
        float MaxSpeed = (float)0.75;
        float distance = (float)Math.sqrt((distancex2-distancex1)*(distancex2-distancex1)+(distancey2-distancey1)*(distancey2-distancey1)+ (distancez2-distancez1)*(distancez2-distancez1));
        time = distance/MaxSpeed;
        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        TextView theTextView21  = (TextView) findViewById(R.id.textView21);
        TextView theTextView22  = (TextView) findViewById(R.id.textView22);
        mRoll =  (float)(MaxSpeed*(distancez2-distancez1)/distance);
        mPitch = - (float)((MaxSpeed*(distancex2-distancex1)/distance));
        mThrottle = - (float)(MaxSpeed*(distancey2-distancey1)/distance);
        theTextView21.setText(" L/R = " + String.format("%.2f",mPitch));
        theTextView21.setTextColor(Color.RED);
        theTextView22.setText(" F/B = " + String.format("%.2f",mRoll));
        theTextView22.setTextColor(Color.RED);
        theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);

//        TextView theTextView10 = (TextView) findViewById(R.id.textView10);
//        TextView theTextView11 = (TextView) findViewById(R.id.textView11);
//        TextView theTextView12 = (TextView) findViewById(R.id.textView12);
//
//        theTextView10.setText(" , " + String.format("%.2f",distancex2 - distancex1));
//        theTextView10.setTextColor(Color.RED);
//
//        theTextView11.setText(" , " + String.format("%.2f",distancey2 - distancey1));
//        theTextView11.setTextColor(Color.RED);
//
//        theTextView12.setText(" , " + String.format("%.2f",distancez2 - distancez1));
//        theTextView12.setTextColor(Color.RED);

        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                setzero();
                double final_yaw2, initial_yaw2;
                final_yaw2 = final_yaw;
                initial_yaw2 = initial_yaw;
                rotatecamera(final_yaw, initial_yaw);
            }
        },(long)( time * 1000));

        return 0;
    }


//Drone yaw rotation

    double rotatecamera(double oyaw, double eyaw) {
        moveyaw = (eyaw - oyaw)/1.1;
        ttyaw=(20.5 * moveyaw) / 360;
        timeyaw = Math.abs(ttyaw);
//        TextView theTextView9 = (TextView) findViewById(R.id.textView9);
//        theTextView9.setText(", " + String.format("%.2f",moveyaw));
//        theTextView9.setTextColor(Color.RED);
        if(Math.abs(moveyaw) > 7) {
            if (oyaw > eyaw) {
                RotateLeft();
                counter=1;
                //showToast("Rotating left");
                Handler handler = new Handler();
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        setzeroyaw();
                        Handler handler = new Handler();
                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                //captureAction();
                                saveImageToExternalStorage(sourceBitmap);

                            }
                        }, (1500));
                    }
                }, (long) (timeyaw * 1000));

            } else if (oyaw < eyaw) {
                counter = 2;
                RotateRight();
                //showToast("Rotating Right");
                Handler handler = new Handler();
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        setzeroyaw();
                        Handler handler = new Handler();
                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                //captureAction();
                                saveImageToExternalStorage(sourceBitmap);

                            }
                        }, (1500));
                    }
                }, (long) (timeyaw * 1000));
            }
        }
        else if(Math.abs(moveyaw) < 4) {
            counter =3;
            saveImageToExternalStorage(sourceBitmap);
        }

        return 0;
    }


// for return to home

    double straight_reverse(double distancex1,double distancex2,double distancey1,double distancey2,double distancez1,double distancez2) {
        float MaxSpeed = (float)0.75;
        float distance = (float)Math.sqrt((distancex2-distancex1)*(distancex2-distancex1)+(distancey2-distancey1)*(distancey2-distancey1)+ (distancez2-distancez1)*(distancez2-distancez1));
        time = distance/MaxSpeed;
        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        TextView theTextView21  = (TextView) findViewById(R.id.textView21);
        TextView theTextView22  = (TextView) findViewById(R.id.textView22);
        mRoll =  -(float)(MaxSpeed*(distancez2-distancez1)/distance);
        mPitch =  (float)((MaxSpeed*(distancex2-distancex1)/distance));
        mThrottle =  (float)(MaxSpeed*(distancey2-distancey1)/distance);
        theTextView21.setText(" L/R = " + String.format("%.2f",mPitch));
        theTextView21.setTextColor(Color.RED);
        theTextView22.setText(" F/B = " + String.format("%.2f",mRoll));
        theTextView22.setTextColor(Color.RED);
        theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);

//        TextView theTextView10 = (TextView) findViewById(R.id.textView10);
//        TextView theTextView11 = (TextView) findViewById(R.id.textView11);
//        TextView theTextView12 = (TextView) findViewById(R.id.textView12);
//
//        theTextView10.setText(" , " + String.format("%.2f",distancex2 - distancex1));
//        theTextView10.setTextColor(Color.RED);
//
//        theTextView11.setText(" , " + String.format("%.2f",distancey2 - distancey1));
//        theTextView11.setTextColor(Color.RED);
//
//        theTextView12.setText(" , " + String.format("%.2f",distancez2 - distancez1));
//        theTextView12.setTextColor(Color.RED);
        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                setzeroyaw();
                Handler handler = new Handler();
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        //captureAction();
                        saveImageToExternalStorage(sourceBitmap);

                    }
                }, (1500));
            }
        }, (long) (time * 1000));


        return 0;
    }

// for return to home

    double rotatecamera_reverse(double timeyaw) {

        if(counter == 1) {
            RotateRight();
            //showToast("Rotating left");
            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    setzeroyaw();
                    Handler handler = new Handler();
                    handler.postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            //captureAction();
                            straight_reverse(distancex1,distancex2,distancey1,distancey2,distancez1,distancez2);

                        }
                    }, (1500));
                }
            }, (long) (timeyaw * 1000));
        }
         if (counter == 2){
             RotateLeft();
                //showToast("Rotating Right");
             Handler handler = new Handler();
             handler.postDelayed(new Runnable() {
             @Override
             public void run() {
             setzeroyaw();
             Handler handler = new Handler();
             handler.postDelayed(new Runnable() {
                @Override
                 public void run() {
                    //captureAction();
                    straight_reverse(distancex1,distancex2,distancey1,distancey2,distancez1,distancez2);

                      }
                        }, (1500));
                    }
                }, (long) (timeyaw * 1000));
            }

        if(counter==3) {
            straight_reverse(distancex1,distancex2,distancey1,distancey2,distancez1,distancez2);
        }


        return 0;
    }

// for capturing selfie function

    public void cap() {
        if (time > timeyaw)
        {
            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    captureAction();

                }
            },(long)(time * 1000));
        }
        else if (time < timeyaw)
        {
            Handler handler = new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    captureAction();
                }
            },(long)(timeyaw * 1000) );
        }
    }


    public  void  resetgimbal(){
        if(act_pitch != 0.0) {
            Gimbal gimbal = getGimbalInstance();
            if (gimbal != null) {
                gimbal.reset(null);
            } else {
                showToast("The gimbal is disconnected.");
            }
        }

    }


// set all value zero

    public void setzero(){

        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        TextView theTextView21  = (TextView) findViewById(R.id.textView21);
        TextView theTextView22  = (TextView) findViewById(R.id.textView22);

        //mYaw = (float)(yawJoyControlMaxSpeed * pX);
        mPitch = (float)0.0;
        mRoll = (float) 0.0;
        mThrottle = (float)0.0;

        if (mFlightController != null){
            mFlightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError != null){
                        showToast(djiError.getDescription());
                    }else
                    {
                        showToast("Enable Virtual Stick Success");
                    }
                }
            });
        }
        theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);
        theTextView21.setText(" L/R = " + String.format("%.2f",mPitch));
        theTextView21.setTextColor(Color.RED);
        theTextView22.setText(" F/B = " + String.format("%.2f", mRoll));
        theTextView22.setTextColor(Color.RED);
    }

    public void setzeroyaw(){

        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        TextView theTextView21  = (TextView) findViewById(R.id.textView21);
        TextView theTextView22  = (TextView) findViewById(R.id.textView22);
        TextView theTextView23  = (TextView) findViewById(R.id.textView23);

        mYaw = (float)(0.0);

        theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);
        theTextView21.setText(" L/R = " + String.format("%.2f",mPitch));
        theTextView21.setTextColor(Color.RED);
        theTextView22.setText(" F/B = " + String.format("%.2f", mRoll));
        theTextView22.setTextColor(Color.RED);
        theTextView23.setText(" RL =" + String.format("%.2f",mYaw));
        theTextView23.setTextColor(Color.RED);
    }


    public void moveup() {
        double Y = 0;
        double pYI = incrementY(Y);
        pY+= pYI;
        if(Math.abs(pY) < 0.02 ){
            pY = 0;
        }
        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        //mThrottle = (float)(verticalJoyControlMaxSpeed *pY);
        mThrottle = (float).50;
        theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);
    }

    public void movedown() {
        double Y1 = 0;
        double pYD = decrementY(Y1);
        pY+= pYD;
        if(Math.abs(pY) < 0.02 ){
            pY = 0;
        }
        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        //mThrottle = (float)(verticalJoyControlMaxSpeed *pY);
        mThrottle = (float)(-0.50);
        theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);
    }

    public void RotateRight() {
        double X = 0;
        double pXI = incrementX(X);
        pX+= pXI;
        if(Math.abs(pX) < 0.02 ){
            pX = 0;
        }
        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        TextView theTextView23  = (TextView) findViewById(R.id.textView23);
        mYaw = (float)(18);
        theTextView20.setText(" U/D= " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);
        theTextView23.setText(" RR = " + String.format("%.2f",mYaw));
        theTextView23.setTextColor(Color.RED);
    }

    public void RotateLeft() {
        double X = 0;
        double pXI = decrementX(X);
        pX+= pXI;
        if(Math.abs(pX) < 0.02 ){
            pX = 0;
        }
        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        TextView theTextView23  = (TextView) findViewById(R.id.textView23);
        mYaw = (float)(-18);
        theTextView20.setText(" U/D= " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);
        theTextView23.setText(" RL = " + String.format("%.2f",mYaw));
        theTextView23.setTextColor(Color.RED);
    }



    protected void onProductChange() {
        initPreviewer();
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initPreviewer();
        initFlightController();
        onProductChange();
        enablePitchExtensionIfPossible();
        super.onResume();
        initPreviewer();
        onProductChange();
        enablePitchExtensionIfPossible();
        if (getGimbalInstance() != null) {
            getGimbalInstance().setMode(GimbalMode.YAW_FOLLOW, new CallbackHandlers.CallbackToastHandler());
        }
        if (ModuleVerificationUtil.isGimbalModuleAvailable()) {

            DJISampleApplication.getProductInstance().getGimbal().setStateCallback(new GimbalState.Callback() {
                @Override
                public void onUpdate(@NonNull GimbalState gimbalState) {
                    stringBuffer = new StringBuffer();
                    stringBuffer.delete(0, stringBuffer.length());

                    stringBuffer.append("PitchInDegrees: ").
                            append(gimbalState.getAttitudeInDegrees().getPitch()).append("\n");
                    stringBuffer.append("RollInDegrees: ").
                            append(gimbalState.getAttitudeInDegrees().getRoll()).append("\n");
                    stringBuffer.append("YawInDegrees: ").
                            append(gimbalState.getAttitudeInDegrees().getYaw()).append("\n");
                    act_pitch = gimbalState.getAttitudeInDegrees().getPitch();
                    act_roll = gimbalState.getAttitudeInDegrees().getRoll();
                    act_yaw = gimbalState.getAttitudeInDegrees().getYaw();
                }
            });
        }

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        if (mVideoSurface == null) {
            Log.e(TAG, "mVideoSurface is null");
        }
    }

    private Gimbal getGimbalInstance() {
        if (gimbal == null) {
            initGimbal();
        }
        return gimbal;
    }

    private void initGimbal() {
        if (DJISDKManager.getInstance() != null) {
            BaseProduct product = DJISDKManager.getInstance().getProduct();
            if (product != null) {
                if (product instanceof Aircraft) {
                    gimbal = ((Aircraft) product).getGimbals().get(currentGimbalId);
                } else {
                    gimbal = product.getGimbal();
                }
            }
        }
    }


    private boolean isFeatureSupported(CapabilityKey key) {
        Gimbal gimbal = getGimbalInstance();
        if (gimbal == null) {
            return false;
        }
        DJIParamCapability capability = null;
        if (gimbal.getCapabilities() != null) {
            capability = gimbal.getCapabilities().get(key);
        }
        if (capability != null) {
            return capability.isSupported();
        }
        return false;
    }
    private void sendRotateGimbalCommand(Rotation rotation) {
        Gimbal gimbal = getGimbalInstance();
        if (gimbal == null) {
            return;
        }
        gimbal.rotate(rotation, new CallbackHandlers.CallbackToastHandler());
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        uninitPreviewer();
        super.onPause();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    public void onReturn(View view){
        Log.e(TAG, "onReturn");
        this.finish();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        uninitPreviewer();
        if (null != mSendVirtualStickDataTimer) {
            mSendVirtualStickDataTask.cancel();
            mSendVirtualStickDataTask = null;
            mSendVirtualStickDataTimer.cancel();
            mSendVirtualStickDataTimer.purge();
            mSendVirtualStickDataTimer = null;
        }
        super.onDestroy();
    }

    private void initUI() {
        // init mVideoSurface
        mVideoSurface = (TextureView)findViewById(R.id.video_previewer_surface);
        mImageSurface = (ImageView) findViewById(R.id.flight_image_previewer_surface);
        recordingTime = (TextView) findViewById(R.id.timer);
        mCaptureBtn = (Button) findViewById(R.id.btn_capture);
        mRecordBtn = (ToggleButton) findViewById(R.id.btn_record);
        mShootPhotoModeBtn = (Button) findViewById(R.id.btn_shoot_photo_mode);
        mRecordVideoModeBtn = (Button) findViewById(R.id.btn_record_video_mode);
//        yawMinBtn = (Button) findViewById(R.id.btn_pitchMin);
//        yawMaxBtn = (Button) findViewById(R.id.btn_pitchMax);
        mBtnEnableVirtualStick = (Button) findViewById(R.id.btn_enable_virtual_stick);
        mBtnDisableVirtualStick = (Button) findViewById(R.id.btn_disable_virtual_stick);
        mBtnMovedown = (Button) findViewById(R.id.btn_movedown);
        mBtnMoveup = (Button) findViewById(R.id.btn_moveup);
        mBtnTakeOff = (Button) findViewById(R.id.btn_take_off);
        mBtnLand = (Button) findViewById(R.id.btn_land);
        mBtnset = (Button) findViewById(R.id.btn_set);
        mBtnstart = (Button) findViewById(R.id.btn_start);
        mBtnvalue = (Button) findViewById(R.id.btn_value1);
        //mConnectStatusTextView = (TextView) findViewById(R.id.ConnectStatusTextView);
        mScreenJoystickRight = (OnScreenJoystick)findViewById(R.id.directionJoystickRight);
        mScreenJoystickLeft = (OnScreenJoystick)findViewById(R.id.directionJoystickLeft);
        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);
        }
        mCaptureBtn.setOnClickListener(this);
        mRecordBtn.setOnClickListener(this);
        mShootPhotoModeBtn.setOnClickListener(this);
        mRecordVideoModeBtn.setOnClickListener(this);
        mBtnEnableVirtualStick.setOnClickListener(this);
        mBtnDisableVirtualStick.setOnClickListener(this);
        mBtnMovedown.setOnClickListener(this);
        mBtnset.setOnClickListener(this);
        mBtnstart.setOnClickListener(this);
        mBtnvalue.setOnClickListener(this);
        mBtnMoveup.setOnClickListener(this);
        mBtnTakeOff.setOnClickListener(this);
        mBtnLand.setOnClickListener(this);
//        yawMinBtn.setOnClickListener(this);
//        yawMaxBtn.setOnClickListener(this);
        recordingTime.setVisibility(View.INVISIBLE);
        mRecordBtn.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    //startRecord();
                } else {
                    //stopRecord();
                }
            }
        });

        mScreenJoystickRight.setJoystickListener(new OnScreenJoystickListener(){
            @Override
            public void onTouch(OnScreenJoystick joystick, float pX, float pY) {
                if(Math.abs(pX) < 0.02 ){
                    pX = 0;
                }

                if(Math.abs(pY) < 0.02 ){
                    pY = 0;
                }
                float pitchJoyControlMaxSpeed = 10;
                float rollJoyControlMaxSpeed = 10;

                mPitch = (float)(pitchJoyControlMaxSpeed * pX);
                mRoll = (float)(rollJoyControlMaxSpeed * pY);

                TextView theTextView21  = (TextView) findViewById(R.id.textView21);
                TextView theTextView22  = (TextView) findViewById(R.id.textView22);
                theTextView21.setText(" L/R = " + String.format("%.2f",mPitch));
                theTextView21.setTextColor(Color.RED);
                theTextView22.setText(" F/B = " + String.format("%.2f",mRoll));
                theTextView22.setTextColor(Color.RED);

                if (null == mSendVirtualStickDataTimer) {
                    mSendVirtualStickDataTask = new SendVirtualStickDataTask();
                    mSendVirtualStickDataTimer = new Timer();
                    mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 100, 200);
                }

            }

        });

        mScreenJoystickLeft.setJoystickListener(new OnScreenJoystickListener() {
            @Override
            public void onTouch(OnScreenJoystick joystick, float pX, float pY) {
                if(Math.abs(pX) < 0.02 ){
                    pX = 0;
                }
                if(Math.abs(pY) < 0.02 ){
                    pY = 0;
                }
                float verticalJoyControlMaxSpeed = 2;
                float yawJoyControlMaxSpeed = 30;

                mYaw = (float)(yawJoyControlMaxSpeed * pX);
                mThrottle = (float)(verticalJoyControlMaxSpeed * pY);

                TextView theTextView23  = (TextView) findViewById(R.id.textView23);
                TextView theTextView20  = (TextView) findViewById(R.id.textView20);
                theTextView23.setText(" Yaw = " + String.format("%.2f",mYaw));
                theTextView23.setTextColor(Color.RED);
                theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
                theTextView20.setTextColor(Color.RED);

                if (null == mSendVirtualStickDataTimer) {
                    mSendVirtualStickDataTask = new SendVirtualStickDataTask();
                    mSendVirtualStickDataTimer = new Timer();
                    mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, 200);
                }
            }
        });
    }

//for activiting the drone commands

    private void dronestart() {
        pY = 0;
        float verticalJoyControlMaxSpeed = 1;
        mThrottle = (float)(verticalJoyControlMaxSpeed * pY);
        TextView theTextView20  = (TextView) findViewById(R.id.textView20);
        TextView theTextView23  = (TextView) findViewById(R.id.textView23);
        theTextView20.setText(" U/D = " + String.format("%.2f",mThrottle));
        theTextView20.setTextColor(Color.RED);
        theTextView23.setText(" Yaw = " + String.format("%.2f",mYaw));
        theTextView23.setTextColor(Color.RED);
        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                setzero();
            }
        },(long)(3*1000 ) );

        if (null == mSendVirtualStickDataTimer) {
            mSendVirtualStickDataTask = new SendVirtualStickDataTask();
            mSendVirtualStickDataTimer = new Timer();
            mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, 200);
        }
    }

    private void rotatePitch(double valpitch) {
        Gimbal gimbal = getGimbalInstance();
        if (gimbal == null) {
            return;
        }
        Number minValue = valpitch;
        Rotation.Builder builder = new Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).time(2);
        builder.pitch(minValue.floatValue());
        sendRotateGimbalCommand(builder.build());
    }

    private void enablePitchExtensionIfPossible() {

        Gimbal gimbal = getGimbalInstance();
        if (gimbal == null) {
            return;
        }
        boolean ifPossible = isFeatureSupported(CapabilityKey.PITCH_RANGE_EXTENSION);
        if (ifPossible) {
            gimbal.setPitchRangeExtensionEnabled(true, new CallbackHandlers.CallbackToastHandler());
        }
    }

    private void initPreviewer() {

        BaseProduct product = FPVDemoApplication.getProductInstance();

        if (product == null || !product.isConnected()) {
            showToast(getString(R.string.disconnected));
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
        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null){
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureAvailable");
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
        Log.e(TAG,"onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }
        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        trackHeatSignatures();
    }

    public void showToast(final String msg) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(MainActivityL.this, msg, Toast.LENGTH_SHORT).show();
            }
        });
    }

// real time keypoints detection

    private void trackHeatSignatures() {
        TextView theTextView5 = (TextView) findViewById(R.id.textView5);
        TextView theTextView6 = (TextView) findViewById(R.id.textView6);
        showToast("detecting");
        sourceBitmap = Bitmap.createScaledBitmap(mVideoSurface.getBitmap(),1200,605, false);
        Mat droneImage = new Mat();
        Utils.bitmapToMat(sourceBitmap, droneImage);
        Mat img = displayAlteredImage(droneImage);
        Bitmap bmpImageSurface = Bitmap.createBitmap(img.cols(),img.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(img, bmpImageSurface);
        displayBitmap = Bitmap.createScaledBitmap(bmpImageSurface, 1200,605, false);
//        w = mVideoSurface.getWidth();
//        h = mVideoSurface.getHeight();
        w = 1200;
        h = 605;


        TextView theTextView60 = (TextView) findViewById(R.id.textView60);
        TextView theTextView70 = (TextView) findViewById(R.id.textView70);
        TextView theTextView80 = (TextView) findViewById(R.id.textView80);
        TextView theTextView26 = (TextView) findViewById(R.id.textView26);
        TextView theTextView27 = (TextView) findViewById(R.id.textView27);
        TextView theTextView28 = (TextView) findViewById(R.id.textView28);

        mImageSurface.setImageBitmap(null);
        mImageSurface.setImageBitmap(displayBitmap);


        theTextView26.setText("G_R:" + String.valueOf(act_roll));
        theTextView26.setTextColor(Color.RED);

        theTextView27.setText("G_P:" + String.valueOf(act_pitch));
        theTextView27.setTextColor(Color.RED);

        theTextView28.setText("G_Y:" + String.valueOf(act_yaw));
        theTextView28.setTextColor(Color.RED);

//        theTextView60.setText("W:" + String.valueOf(bmpImageSurface.getWidth()));
//        theTextView60.setTextColor(Color.RED);
//
//        theTextView70.setText("H:" + String.valueOf(bmpImageSurface.getHeight()));
//        theTextView70.setTextColor(Color.RED);
    }

    public void takeoff1() {
        if (mFlightController != null){
            mFlightController.startTakeoff(
                    new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                showToast(djiError.getDescription());
                            } else {
                                showToast("Take off Success");
                            }
                        }
                    }
            );
        }
    }


    @Override
    public void onClick(View v) {

        switch (v.getId()) {
            case R.id.btn_capture:{
                captureAction();
                saveImageToExternalStorage(sourceBitmap);
                break;
            }
            case R.id.btn_shoot_photo_mode:{
                switchCameraMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO);
                break;
            }
            case R.id.btn_record_video_mode:{
                switchCameraMode(SettingsDefinitions.CameraMode.RECORD_VIDEO);
                break;
            }
            case R.id.btn_enable_virtual_stick:
                if (mFlightController != null){

                    mFlightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null){
                                showToast(djiError.getDescription());
                            }else
                            {
                                showToast("Enable Virtual Stick Success");
                            }
                        }
                    });
                }
                break;

            case R.id.btn_disable_virtual_stick:
                if (mFlightController != null){
                    mFlightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                showToast(djiError.getDescription());
                            } else {
                                showToast("Disable Virtual Stick Success");
                            }
                        }
                    });
                }
                break;

            case R.id.btn_take_off:
                takeoff1();
                Handler handler = new Handler();
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        //captureAction();
                        saveImageToExternalStorage(sourceBitmap);

                    }
                }, (5000));
                break;

            case R.id.btn_land:

// Clock stop
                if (running) {
                    chronometer.stop();
                    running = true;
                }


                if (mFlightController != null){
                    mFlightController.startLanding(
                            new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if (djiError != null) {
                                        showToast(djiError.getDescription());
                                    } else {
                                        showToast("Start Landing");
                                    }
                                }
                            }
                    );
                }
                break;

            default:
                break;
        }
    }

// Save image to android phone gallery

    private void saveImageToExternalStorage(Bitmap finalBitmap) {
        String root = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES).toString();
        File myDir = new File(root + "/Selfie_Drone");
        myDir.mkdirs();
        Random generator = new Random();
        int n = 10000;
        n = generator.nextInt(n);
        String fname = "Image-" + n + ".jpg";
        File file = new File(myDir, fname);
        if (file.exists())
            file.delete();
        try {
            FileOutputStream out = new FileOutputStream(file);
            finalBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
            out.flush();
            out.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        MediaScannerConnection.scanFile(this, new String[] { file.toString() }, null,
                new MediaScannerConnection.OnScanCompletedListener() {
                    public void onScanCompleted(String path, Uri uri) {
                        Log.i("ExternalStorage", "Scanned " + path + ":");
                        Log.i("ExternalStorage", "-> uri=" + uri);
                    }
                });
    }


    Mat displayAlteredImage(Mat img) {
        Mat hsvImage = new Mat();
        Mat color_range_red = img.clone();
        Mat color_range_black = new Mat();
        Mat color_range_green = new Mat();
        Mat color_range_blue = new Mat();
        Mat canny = new Mat();
        Mat copy = img.clone();
        Mat blur = new Mat();
        Mat dil = new Mat();

        TextView theTextView1 = (TextView) findViewById(R.id.textView1);
        TextView theTextView2 = (TextView) findViewById(R.id.textView2);
        TextView theTextView3 = (TextView) findViewById(R.id.textView3);
        TextView theTextView4 = (TextView) findViewById(R.id.textView4);

        arr_rxr = new double[10];
        arr_ryr = new double[10];
        arr_rxb = new double[10];
        arr_ryb = new double[10];
        arr_rxg = new double[10];
        arr_ryg = new double[10];
        arr_rxbl = new double[10];
        arr_rybl = new double[10];

        Point point1 = new Point(410, 110);
        Point point2 = new Point(790, 430);
        Imgproc.rectangle (copy, point1, point2, new Scalar(255,255,255), 1);
        Imgproc.GaussianBlur(img, blur, new Size(5, 5), 0);
        Imgproc.cvtColor(blur, hsvImage, Imgproc.COLOR_BGR2HSV);


        Core.inRange(hsvImage, new Scalar(70, 70, 80), new Scalar(210, 255, 255), color_range_red);
        Core.inRange(hsvImage, new Scalar(0, 0, 0), new Scalar(80, 130, 150), color_range_black);
        Core.inRange(hsvImage, new Scalar(20, 40, 72), new Scalar(70, 245, 245), color_range_green);
        Core.inRange(hsvImage, new Scalar(0, 110, 140), new Scalar(40, 255, 255), color_range_blue);


// start

 //Red
        Mat out1 = new Mat();
        Mat tmp1 = new Mat();

        Mat Kernel1 = new Mat(new Size(3, 3), CvType.CV_8U, new Scalar(255));
        Imgproc.morphologyEx(color_range_red, tmp1, Imgproc.MORPH_OPEN, Kernel1);
        Imgproc.morphologyEx(tmp1, out1, Imgproc.MORPH_CLOSE, Kernel1);
        Imgproc.Canny(out1, canny, 50, 100);
        Mat k1 = Imgproc.getStructuringElement(MORPH_RECT, new Size((2*2) + 1, (2*2)+1));
        Imgproc.dilate(canny, dil, k1);
        List<MatOfPoint> contoursred = new ArrayList<MatOfPoint>();
        Imgproc.findContours(dil, contoursred, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint cnt : contoursred) {
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint approxf1 = new MatOfPoint();

            double peri = Imgproc.arcLength( cnt2f, true);
            double area = Imgproc.contourArea(cnt);

            Imgproc.approxPolyDP(cnt2f, approx, 0.02 * peri, true);
            approx.convertTo(approxf1, CvType.CV_32S);
            List<MatOfPoint> contourTemp = new ArrayList<>();
            contourTemp.add(approxf1);
            int count= (int) approx.total();

            if (Math.abs(area) > 100 && Math.abs(area) < 4000)
            {
                if(count >=6  && count <= 10) {
                    float[] radius = new float[1];
                    Point center = new Point();
                    Imgproc.minEnclosingCircle(approx, center, radius);
                    if((center.x > point1.x) && (center.x < point2.x) && (center.y > point1.y) && (center.y < point2.y) ) {
                        int r = (int) (radius[0]);
                        rxr = (int) center.x;
                        ryr = (int) center.y;
                        Imgproc.circle(copy, center, 1, new Scalar(255, 255, 255), 3);
                        Imgproc.circle(copy, center, r, new Scalar(255, 255, 255), 2);
                        Imgproc.drawContours(copy, contourTemp, -1, new Scalar(0, 255, 255), 2);
                    }
                }
            }
       }

//Black
        Mat out2 = new Mat();
        Mat tmp2 = new Mat();

        Mat Kernel2 = new Mat(new Size(3, 3), CvType.CV_8U, new Scalar(255));
        Imgproc.morphologyEx(color_range_black, tmp2, Imgproc.MORPH_OPEN, Kernel2);
        Imgproc.morphologyEx(tmp2, out2, Imgproc.MORPH_CLOSE, Kernel2);
        Imgproc.Canny(out2, canny, 50, 100);
        Mat k2 = Imgproc.getStructuringElement(MORPH_RECT, new Size((2*2) + 1, (2*2)+1));
        Imgproc.dilate(canny, dil, k2);
        List<MatOfPoint> contoursblack = new ArrayList<MatOfPoint>();
        Imgproc.findContours(dil, contoursblack, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint cnt : contoursblack) {
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint approxf1 = new MatOfPoint();

            double peri = Imgproc.arcLength( cnt2f, true);
            double area = Imgproc.contourArea(cnt);

            Imgproc.approxPolyDP(cnt2f, approx, 0.02 * peri, true);
            approx.convertTo(approxf1, CvType.CV_32S);
            List<MatOfPoint> contourTemp = new ArrayList<>();
            contourTemp.add(approxf1);
            int count= (int) approx.total();

            if (Math.abs(area) > 100 && Math.abs(area) < 4000)
            {
                if(count >=6  && count <= 10) {
                    float[] radius = new float[1];
                    Point center = new Point();
                    Imgproc.minEnclosingCircle(cnt2f, center, radius);
                    if((center.x > point1.x) && (center.x < point2.x) && (center.y > point1.y) && (center.y < point2.y) ) {
                        int r = (int) (radius[0]);
                        rxbl= (int) center.x;
                        rybl = (int) center.y;
                        Imgproc.circle(copy, center, 1, new Scalar(255, 255, 255), 3);
                        Imgproc.circle(copy, center, r, new Scalar(255, 255, 255), 2);
                        Imgproc.drawContours(copy, contourTemp, -1, new Scalar(0, 255, 255), 2);
                    }
                }
            }
        }

//Green

        Mat out3 = new Mat();
        Mat tmp3 = new Mat();

        Mat Kernel3 = new Mat(new Size(3, 3), CvType.CV_8U, new Scalar(255));
        Imgproc.morphologyEx(color_range_green, tmp3, Imgproc.MORPH_OPEN, Kernel3);
        Imgproc.morphologyEx(tmp3, out3, Imgproc.MORPH_CLOSE, Kernel3);
        Imgproc.Canny(out3, canny, 50, 100);
        Mat k3 = Imgproc.getStructuringElement(MORPH_RECT, new Size((2*2) + 1, (2*2)+1));
        Imgproc.dilate(canny, dil, k3);
        List<MatOfPoint> contoursgreen = new ArrayList<MatOfPoint>();
        Imgproc.findContours(dil, contoursgreen, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint cnt : contoursgreen) {
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint approxf1 = new MatOfPoint();

            double peri = Imgproc.arcLength( cnt2f, true);
            double area = Imgproc.contourArea(cnt);

            Imgproc.approxPolyDP(cnt2f, approx, 0.02 * peri, true);
            approx.convertTo(approxf1, CvType.CV_32S);
            List<MatOfPoint> contourTemp = new ArrayList<>();
            contourTemp.add(approxf1);
            int count= (int) approx.total();

            if (Math.abs(area) > 100 && Math.abs(area) < 4000)
            {
                if(count >=6  && count <= 10) {
                    float[] radius = new float[1];
                    Point center = new Point();
                    Imgproc.minEnclosingCircle(cnt2f, center, radius);
                    if((center.x > point1.x) && (center.x < point2.x) && (center.y > point1.y) && (center.y < point2.y) ) {
                        int r = (int) (radius[0]);
                        rxg = (int) center.x;
                        ryg = (int) center.y;
                        Imgproc.circle(copy, center, 1, new Scalar(255, 255, 255), 3);
                        Imgproc.circle(copy, center, r, new Scalar(255, 255, 255), 2);
                        Imgproc.drawContours(copy, contourTemp, -1, new Scalar(0, 255, 255), 2);
                    }
                }
            }
        }

//blue
        Mat out4 = new Mat();
        Mat tmp4 = new Mat();

        Mat Kernel4 = new Mat(new Size(3, 3), CvType.CV_8U, new Scalar(255));
        Imgproc.morphologyEx(color_range_blue, tmp4, Imgproc.MORPH_OPEN, Kernel4);
        Imgproc.morphologyEx(tmp4, out4, Imgproc.MORPH_CLOSE, Kernel4);
        Imgproc.Canny(out4, canny, 50, 100);
        Mat k4 = Imgproc.getStructuringElement(MORPH_RECT, new Size((2*2) + 1, (2*2)+1));
        Imgproc.dilate(canny, dil, k4);
        List<MatOfPoint> contoursblue = new ArrayList<MatOfPoint>();
        Imgproc.findContours(dil, contoursblue, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint cnt : contoursblue) {
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint approxf1 = new MatOfPoint();

            double peri = Imgproc.arcLength( cnt2f, true);
            double area = Imgproc.contourArea(cnt);

            Imgproc.approxPolyDP(cnt2f, approx, 0.02 * peri, true);
            approx.convertTo(approxf1, CvType.CV_32S);
            List<MatOfPoint> contourTemp = new ArrayList<>();
            contourTemp.add(approxf1);
            int count= (int) approx.total();

            if (Math.abs(area) > 100 && Math.abs(area) < 4000)
            {
                if(count >=6  && count <= 10) {
                    float[] radius = new float[1];
                    Point center = new Point();
                    Imgproc.minEnclosingCircle(cnt2f, center, radius);
                    if((center.x > point1.x) && (center.x < point2.x) && (center.y > point1.y) && (center.y < point2.y) ) {
                        int r = (int) (radius[0]);
                        rxb = (int) center.x;
                        ryb = (int) center.y;
                        Imgproc.circle(copy, center, 1, new Scalar(255, 255, 255), 3);
                        Imgproc.circle(copy, center, r, new Scalar(255, 255, 255), 2);
                        Imgproc.drawContours(copy, contourTemp, -1, new Scalar(0, 255, 255), 2);
                    }
                }
            }
        }
        return copy;
    }

// Save PnP value to excel sheet

    private void writeCsvHeader(String h1, String h2, String h3, String h4, String h5, String h6) throws IOException {
        String line = String.format("%s,%s,%s,%s,%s,%s\n", h1,h2,h3,h4,h5,h6);
        writer.write(line);
    }

    private void writeCsvData(float d, float e, float f, float g, float h, float i) throws IOException {
        String line = String.format("%f,%f,%f,%f,%f,%f\n", d, e, f, g, h, i);
        writer.write(line);
    }

// SolvePnP function

    private void SolvePNP() {
        TextView theTextView1 = (TextView) findViewById(R.id.textView1);
        TextView theTextView2 = (TextView) findViewById(R.id.textView2);
        TextView theTextView3 = (TextView) findViewById(R.id.textView3);
        TextView theTextView4 = (TextView) findViewById(R.id.textView4);
        TextView theTextView5 = (TextView) findViewById(R.id.textView5);
        TextView theTextView6 = (TextView) findViewById(R.id.textView6);
        TextView theTextView60 = (TextView) findViewById(R.id.textView60);
        TextView theTextView70 = (TextView) findViewById(R.id.textView70);
        TextView theTextView80 = (TextView) findViewById(R.id.textView80);
        TextView theTextView90 = (TextView) findViewById(R.id.textView90);

        Mat rotationMatrix = new Mat();
        Mat rvec = new MatOfDouble(Math.PI, 0, 0);
        Mat tvec = new MatOfDouble(-24, 0, 60);

// Camera Matrix 2

        Mat cameraMatrix = Mat.zeros(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, 2766.93281); //fx
        cameraMatrix.put(1, 1, 2811.88175); //fy
        cameraMatrix.put(0, 2, 1942.48783); //cx
        cameraMatrix.put(1, 2, 936.313862); //cy
        cameraMatrix.put(2, 2, 1);

// Distorstion coefficients

        Mat distCoeffs = Mat.zeros(5, 1, CvType.CV_64F);
        distCoeffs.put(0,0, 0.38628748);
        distCoeffs.put(1,0,-0.81343147);
        distCoeffs.put(2,0,-0.03587456);
        distCoeffs.put(3,0, -0.01739278);
        distCoeffs.put(4,0, 1.03496557);

//Image Points

        per_px_r = (100*rxr)/w;
        per_py_r = (100*ryr)/h;

        per_px_bl = (100*rxbl)/w;
        per_py_bl = (100*rybl)/h;

        per_px_g = (100*rxg)/w;
        per_py_g = (100*ryg)/h;

        per_px_b = (100*rxb)/w;
        per_py_b = (100*ryb)/h;

        rxrf = (per_px_r*4000)/100;
        ryrf = (per_py_r*2250)/100;

        rxblf = (per_px_bl*4000)/100;
        ryblf = (per_py_bl*2250)/100;

        rxgf = (per_px_g*4000)/100;
        rygf = (per_py_g*2250)/100;

        rxbf = (per_px_b*4000)/100;
        rybf = (per_py_b*2250)/100;

        MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(rxrf, ryrf),
                new Point(rxblf, ryblf), new Point(rxgf, rygf), new Point(rxbf, rybf));

//Objects Points

        MatOfPoint3f objPoints = new MatOfPoint3f(new Point3(-128, 148, 0), new Point3(128, 148, 0),
                new Point3(-128, -148, 0),new Point3(128, -148, 0));

//SolvePNP Function

        Calib3d.solvePnP(objPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

// Eular Angles and Camera Position

        Calib3d.Rodrigues(rvec, rotationMatrix);
        Mat projectionMatrix = new Mat(4, 4, CvType.CV_64F);
        projectionMatrix.put(0, 0,
                rotationMatrix.get(0, 0)[0], rotationMatrix.get(0, 1)[0], rotationMatrix.get(0, 2)[0], tvec.get(0, 0)[0],
                rotationMatrix.get(1, 0)[0], rotationMatrix.get(1, 1)[0], rotationMatrix.get(1, 2)[0], tvec.get(1, 0)[0],
                rotationMatrix.get(2, 0)[0], rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0], tvec.get(2, 0)[0],
                0,0,0,1
        );

        Mat b1 = new Mat(3, 4, CvType.CV_64F);
        Mat b = projectionMatrix.inv();
        b1.put(0, 0,
                b.get(0, 0)[0], b.get(0, 1)[0], b.get(0, 2)[0], b.get(0, 3)[0],
                b.get(1, 0)[0], b.get(1, 1)[0], b.get(1, 2)[0], b.get(1, 3)[0],
                b.get(2, 0)[0], b.get(2, 1)[0], b.get(2, 2)[0], b.get(2, 3)[0]
        );

        Mat cameraMatrix1 = new Mat();
        Mat rotMatrix = new Mat();
        Mat transVect = new Mat();
        Mat rotMatrixX = new Mat();
        Mat rotMatrixY = new Mat();
        Mat rotMatrixZ = new Mat();
        Mat eulerAngles = new Mat();

        Calib3d.decomposeProjectionMatrix(b1, cameraMatrix1, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);

        rtx = - Math.abs(((b.get(0, 3)[0])/1000));
        rty = - Math.abs(((b.get(1, 3)[0])/1000));
        rtz = ((b.get(2, 3)[0])/1000);
        rtroll = (eulerAngles.get(2,0)[0]);
        rtpitch = - Math.abs(eulerAngles.get(0,0)[0]);
        rtyaw = (eulerAngles.get(1,0)[0]);

        File root = Environment.getExternalStorageDirectory();
        File gpxfile = new File(root, "PNPData.csv");

        try {
            writer = new FileWriter(gpxfile, true);
            for (kk = kk; kk < 1; kk++) {
                writeCsvHeader("X","Y","Z","Roll","Pitch","Yaw");
            }
            writeCsvData((float)rtx,(float)rty,(float)rtz,(float)rtroll,(float)rtpitch, (float)rtyaw);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

// Show Print Point

        theTextView60.setText("P1:" + String.format("%.2f",rxrf) + "," + String.format("%.2f",ryrf));
        theTextView60.setTextColor(Color.RED);

        theTextView70.setText("P2:" + String.format("%.2f",rxblf) + "," + String.format("%.2f",ryblf));
        theTextView70.setTextColor(Color.RED);

        theTextView80.setText("P3:" + String.format("%.2f",rxgf) + "," + String.format("%.2f",rygf));
        theTextView80.setTextColor(Color.RED);

        theTextView90.setText("P4:" + String.format("%.2f",rxbf) + "," + String.format("%.2f",rybf));
        theTextView90.setTextColor(Color.RED);

// Show Real time 6D
        theTextView1.setText("Roll: " + String.format("%.2f",troll) + " , " + String.format("%.2f",rtroll) );
        theTextView1.setTextColor(Color.RED);

        theTextView2.setText( "Pitch: " + String.format("%.2f",tpitch) + " , " + String.format("%.2f",rtpitch));
        theTextView2.setTextColor(Color.RED);

        theTextView3.setText("Yaw: " + String.format("%.2f",tyaw) + " , " + String.format("%.2f",rtyaw));
        theTextView3.setTextColor(Color.RED);

        theTextView4.setText("X: " + String.format("%.2f",tx) + " , " +String.format("%.2f",rtx));
        theTextView4.setTextColor(Color.RED);

        theTextView5.setText("Y: " + String.format("%.2f",ty) + " , " + String.format("%.2f",rty));
        theTextView5.setTextColor(Color.RED);

        theTextView6.setText("Z: " + String.format("%.2f",tz) + " , " + String.format("%.2f",rtz));
        theTextView6.setTextColor(Color.RED);
    }

    private void switchCameraMode(SettingsDefinitions.CameraMode cameraMode){
        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            camera.setMode(cameraMode, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError error) {

                    if (error == null) {
                        showToast("Switch Camera Mode Succeeded");
                    } else {
                        showToast(error.getDescription());
                    }
                }
            });
        }
    }

 // Method for taking photo
    private void captureAction(){
        final Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            SettingsDefinitions.ShootPhotoMode photoMode = SettingsDefinitions.ShootPhotoMode.SINGLE; // Set the camera capture mode as Single mode
            camera.setShootPhotoMode(photoMode, new CommonCallbacks.CompletionCallback(){
                @Override
                public void onResult(DJIError djiError) {
                    if (null == djiError) {
                        handler.postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                camera.startShootPhoto(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        if (djiError == null) {
                                            showToast("take photo: success");
                                        } else {
                                            showToast(djiError.getDescription());
                                        }
                                    }
                                });
                            }
                        }, 1000);
                    }
                }
            });
        }
    }

    class SendVirtualStickDataTask extends TimerTask {
        @Override
        public void run() {
            if (mFlightController != null) {
                mFlightController.sendVirtualStickFlightControlData(
                        new FlightControlData(
                                mPitch, mRoll, mYaw, mThrottle
                        ), new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {

                            }
                        }
                );
            }
        }
    }
}
