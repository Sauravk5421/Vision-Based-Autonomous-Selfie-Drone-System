package com.dji.FPVDemo;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;

public class ColorDetection extends AppCompatActivity {

    public static final String EXTRA_NUMBER1 = "com.dji.FPVDemo.EXTRA_NUMBER1";
    public static final String EXTRA_NUMBER2 = "com.dji.FPVDemo.EXTRA_NUMBER2";
    public static final String EXTRA_NUMBER3 = "com.dji.FPVDemo.EXTRA_NUMBER3";
    public static final String EXTRA_NUMBER4 = "com.dji.FPVDemo.EXTRA_NUMBER4";
    public static final String EXTRA_NUMBER5 = "com.dji.FPVDemo.EXTRA_NUMBER5";
    public static final String EXTRA_NUMBER6 = "com.dji.FPVDemo.EXTRA_NUMBER6";
    private static final String TAG = "ColorDetection" ;
    ImageView imgView;
    TextView text;
    Button processBtn;
    Mat mRgba;
    int w,h;
    double tp11,tp12,tp21,tp22,tp31,tp32,tp41,tp42;
    double per_px_r=0,per_py_r=0,per_px_bl=0,per_py_bl=0,per_px_g=0,per_py_g=0,per_px_b=0,per_py_b=0;
    double tp11f,tp12f,tp21f,tp22f,tp31f,tp32f,tp41f,tp42f;

    private BaseLoaderCallback _baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mRgba=new Mat();
                }
                break;
                default: {
                    super.onManagerConnected(status);
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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_color_detection);
        ActivityCompat.requestPermissions(ColorDetection.this,
                new String[]{Manifest.permission.CAMERA, Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE},
                1);
        processBtn = (Button) findViewById(R.id.processImgBtn);
        imgView = (ImageView) findViewById(R.id.img);
        text = (TextView) findViewById(R.id.test1);
        getSupportActionBar().hide();
        getSupportActionBar().setTitle("Full Screen Image");
        Intent i  = getIntent();
        int position = i.getExtras().getInt("id");
        ImageAdapter imageAdapter = new ImageAdapter(this);
        imgView.setImageResource(imageAdapter.mThumbIds[position]);
        text.setText(String.valueOf(position+1));


        if ( String.valueOf(position) == String.valueOf(0))   // Template 1
        {

            tp11 = 344;
            tp12 = 325;
            tp21 = 376;
            tp22 = 325;
            tp31 = 341;
            tp32 = 358;
            tp41 = 376;
            tp42 = 359;
        }


        if ( String.valueOf(position) == String.valueOf(1))   //Template 2
        {

            tp11 = 876;
            tp12 = 254;
            tp21 = 910;
            tp22 = 253;
            tp31 = 877;
            tp32 = 289;
            tp41 = 909;
            tp42 = 288;
        }

        if ( String.valueOf(position) == String.valueOf(2))  //Template 3
        {

            tp11 = 649;
            tp12 = 235;
            tp21 = 677;
            tp22 = 236;
            tp31 = 649;
            tp32 = 265;
            tp41 = 676;
            tp42 = 266;
        }


        if ( String.valueOf(position) == String.valueOf(3))  //Template 4
        {

            tp11 = 626;
            tp12 = 228;
            tp21 = 656;
            tp22 = 226;
            tp31 = 629;
            tp32 = 260;
            tp41 = 657;
            tp42 = 257;
        }

        if ( String.valueOf(position) == String.valueOf(4))  //Template 5
        {

            tp11 = 635;
            tp12 = 209;
            tp21 = 666;
            tp22 = 208;
            tp31 = 636;
            tp32 = 240;
            tp41 = 665;
            tp42 = 240;
        }

        if ( String.valueOf(position) == String.valueOf(5))  // Template 6
        {

            tp11 = 310;
            tp12 = 311;
            tp21 = 349;
            tp22 = 308;
            tp31 = 311;
            tp32 = 347;
            tp41 = 350;
            tp42 = 347;
        }


        if ( String.valueOf(position) == String.valueOf(6))  // Template7
        {

            tp11 = 820;
            tp12 = 295;
            tp21 = 857;
            tp22 = 295;
            tp31 = 824;
            tp32 = 330;
            tp41 = 856;
            tp42 = 329;
        }

        if ( String.valueOf(position) == String.valueOf(7))  //Template 8
        {
            tp11 = 531;
            tp12 = 130;
            tp21 = 590;
            tp22 = 132;
            tp31 = 532;
            tp32 = 198;
            tp41 = 591;
            tp42 = 200;

        }


        if ( String.valueOf(position) == String.valueOf(8))  // Template 9
        {

            tp11 = 573;
            tp12 = 285;
            tp21 = 602;
            tp22 = 284;
            tp31 = 576;
            tp32 = 315;
            tp41 = 602;
            tp42 = 313;
        }


        if ( String.valueOf(position) == String.valueOf(9))  //Template 10
        {

            tp11 = 548;
            tp12 = 293;
            tp21 = 586;
            tp22 = 292;
            tp31 = 550;
            tp32 = 332;
            tp41 = 586;
            tp42 = 331;
        }


//        if ( String.valueOf(position) == String.valueOf(5))  //6
//        {
//
//            tp11 = 924;
//            tp12 = 277;
//            tp21 = 975;
//            tp22 = 277;
//            tp31 = 925;
//            tp32 = 328;
//            tp41 = 976;
//            tp42 = 328;
//        }
//
//
//        if ( String.valueOf(position) == String.valueOf(6))  //7
//        {
//            tp11 = 598;
//            tp12 = 217;
//            tp21 = 662;
//            tp22 = 217;
//            tp31 = 598;
//            tp32 = 281;
//            tp41 = 662;
//            tp42 = 281;
//        }
//
//        if ( String.valueOf(position) == String.valueOf(7))  //8
//        {
//
//            tp11 = 532;
//            tp12 = 132;
//            tp21 = 593;
//            tp22 = 131;
//            tp31 = 539;
//            tp32 = 197;
//            tp41 = 590;
//            tp42 = 198;
//        }
//
//
//        if ( String.valueOf(position) == String.valueOf(8))  //9
//        {
//
//            tp11 = 609;
//            tp12 = 135;
//            tp21 = 666;
//            tp22 = 136;
//            tp31 = 611;
//            tp32 = 203;
//            tp41 = 667;
//            tp42 = 202;
//        }
//
//        if ( String.valueOf(position) == String.valueOf(9)) //10
//        {
//
//            tp11 = 585;
//            tp12 = 136;
//            tp21 = 660;
//            tp22 = 138;
//            tp31 = 588;
//            tp32 = 221;
//            tp41 = 660;
//            tp42 = 220;
//        }
//
//        if ( String.valueOf(position) == String.valueOf(10)) //11
//        {
////
////            tp11 = 501;
////            tp12 = 215;
////            tp21 = 574;
////            tp22 = 215;
////            tp31 = 496;
////            tp32 = 300;
////            tp41 = 566;
////            tp42 = 308;
//            tp11 = 547;
//            tp12 = 204;
//            tp21 = 626;
//            tp22 = 206;
//            tp31 = 542;
//            tp32 = 288;
//            tp41 = 618;
//            tp42 = 296;
//        }
//
//        if ( String.valueOf(position) == String.valueOf(11)) //12
//        {
//
//            tp11 = 601;
//            tp12 = 144;
//            tp21 = 660;
//            tp22 = 145;
//            tp31 = 599;
//            tp32 = 206;
//            tp41 = 656;
//            tp42 = 207;
//        }


        processBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                processImage();
            }

        });
    }

    private void processImage(){

//        processBtn.setOnClickListener(new View.OnClickListener() {
//            @Override
//            public void onClick(View v) {
//                //openMainActivity();
//            }
//        });

        Intent i  = getIntent();
        int position = i.getExtras().getInt("id");
        ImageAdapter imageAdapter = new ImageAdapter(this);
        imgView.setImageResource(imageAdapter.mThumbIds[position]);
        imgView = (ImageView) findViewById(R.id.img);
        try {
            mRgba= Utils.loadResource(this,imageAdapter.mThumbIds[position]);
        } catch (IOException e) {
            e.printStackTrace();
        }
        Mat oImg = detectColor(mRgba);
        Bitmap bm = Bitmap.createBitmap(oImg.cols(),oImg.rows(),Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(oImg,bm);;
        imgView.setImageBitmap(bm);
    }

    // for image processing, if the input image is not stored in the application

    Mat detectColor(Mat srcImg) {
        Mat blurImg = new Mat();
        Mat srcImg1 = srcImg.clone();
        Mat hsvImage = new Mat();
        Mat color_range_red = new Mat();
        Mat color_range_black = new Mat();
        Mat color_range_green = new Mat();
        Mat color_range_blue = new Mat();
        Mat circlesred = new Mat();
        Mat circlesblack = new Mat();
        Mat circlesgreen = new Mat();
        Mat circlesblue = new Mat();

        TextView theTextView1  = (TextView) findViewById(R.id.textView1);
        TextView theTextView2  = (TextView) findViewById(R.id.textView2);
        TextView theTextView3  = (TextView) findViewById(R.id.textView3);
        TextView theTextView4  = (TextView) findViewById(R.id.textView4);

        Imgproc.GaussianBlur(srcImg, blurImg, new Size(5,5),0);
        Imgproc.cvtColor(blurImg, hsvImage, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsvImage, new Scalar(70,100,50), new Scalar(200,255,255), color_range_red);
        Core.inRange(hsvImage, new Scalar(0, 0, 0), new Scalar(120, 205, 100), color_range_black);
        Core.inRange(hsvImage, new Scalar(20, 40, 72), new Scalar(70, 245, 245), color_range_green);
        Core.inRange(hsvImage, new Scalar(0, 90, 140), new Scalar(40, 255, 255), color_range_blue);

        Imgproc.HoughCircles(color_range_red, circlesred, Imgproc.HOUGH_GRADIENT, 2,
                80, // change this value to detect circles with different distances to each other
                160, 5.0, 50, 200);

        Imgproc.HoughCircles(color_range_black, circlesblack, Imgproc.HOUGH_GRADIENT, 2,
                80, // change this value to detect circles with different distances to each other
                160.0, 5.0, 50, 200);

        Imgproc.HoughCircles(color_range_green, circlesgreen, Imgproc.HOUGH_GRADIENT, 2,
                80, // change this value to detect circles with different distances to each other
                160.0, 5.0, 50, 200);

        Imgproc.HoughCircles(color_range_blue, circlesblue, Imgproc.HOUGH_GRADIENT, 2,
                80, // change this value to detect circles with different distances to each other
                160.0, 5.0, 50, 200);

// Red
        if (circlesred.cols() > 0) {
            for (int x = 0; x < Math.min(circlesred.cols(), 1); x++) {
                double circleVec[] = circlesred.get(0, x);
                if (circleVec == null) {
                    break;
                }
                Point center = new Point((int) circleVec[0], (int) circleVec[1]);
//                tp11 = (int) circleVec[0];
//                tp12 = (int) circleVec[1];
                int radius = (int) circleVec[2];
                Imgproc.circle(srcImg, center, 3, new Scalar(255, 255, 255), 4);
                Imgproc.circle(srcImg, center, radius, new Scalar(255, 255, 255), 2);
            }
        }

//Black
        if (circlesblack.cols() > 0) {
            for (int x = 0; x < Math.min(circlesblack.cols(), 1); x++) {
                double circleVec[] = circlesblack.get(0, x);
                if (circleVec == null) {
                    break;
                }
                Point center = new Point((int) circleVec[0], (int) circleVec[1]);
//                tp21 = (int) circleVec[0];
//                tp22 = (int) circleVec[1];
                int radius = (int) circleVec[2];
                Imgproc.circle(srcImg, center, 3, new Scalar(255, 255, 255), 4);
                Imgproc.circle(srcImg, center, radius, new Scalar(255, 255, 255), 2);
            }
        }

//Green
        if (circlesgreen.cols() > 0) {
            for (int x = 0; x < Math.min(circlesgreen.cols(), 1); x++) {
                double circleVec[] = circlesgreen.get(0, x);
                if (circleVec == null) {
                    break;
                }
                Point center = new Point((int) circleVec[0], (int) circleVec[1]);
//                tp31 = (int) circleVec[0];
//                tp32 = (int) circleVec[1];
                int radius = (int) circleVec[2];
                Imgproc.circle(srcImg, center, 3, new Scalar(255, 255, 255), 4);
                Imgproc.circle(srcImg, center, radius, new Scalar(255, 255, 255), 2);
            }
        }

//Blue
        if (circlesblue.cols() > 0) {
            for (int x = 0; x < Math.min(circlesblue.cols(), 1); x++) {
                double circleVec[] = circlesblue.get(0, x);
                if (circleVec == null) {
                    break;
                }
                Point center = new Point((int) circleVec[0], (int) circleVec[1]);
//                tp41 = (int) circleVec[0];
//                tp42 = (int) circleVec[1];
                int radius = (int) circleVec[2];
                Imgproc.circle(srcImg, center, 3, new Scalar(255, 255, 255), 4);
                Imgproc.circle(srcImg, center, radius, new Scalar(255, 255, 255), 2);
            }
        }

        Mat rotationMatrix = new Mat();
        Mat rvec = new Mat();
        Mat tvec = new Mat();


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

        w = 1200;
        h = 605;
        per_px_r = (100*tp11)/w;
        per_py_r = (100*tp12)/h;

        per_px_bl = (100*tp21)/w;
        per_py_bl = (100*tp22)/h;

        per_px_g = (100*tp31)/w;
        per_py_g = (100*tp32)/h;

        per_px_b = (100*tp41)/w;
        per_py_b = (100*tp42)/h;

        tp11f = (per_px_r*4000)/100;
        tp12f = (per_py_r*2250)/100;

        tp21f = (per_px_bl*4000)/100;
        tp22f = (per_py_bl*2250)/100;

        tp31f = (per_px_g*4000)/100;
        tp32f = (per_py_g*2250)/100;

        tp41f = (per_px_b*4000)/100;
        tp42f = (per_py_b*2250)/100;

        MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(tp11f, tp12f), new Point(tp21f, tp22f), new Point(tp31f, tp32f),new Point(tp41f, tp42f));

//Objects Points

        MatOfPoint3f objPoints = new MatOfPoint3f(new Point3(-128, 148, 0), new Point3(128, 148, 0),
                new Point3(-128, -148, 0),new Point3(128, -148, 0));

//SolvePNP Function

        Calib3d.solvePnP(objPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

// Eular Angles and Camera Position

        Calib3d.Rodrigues (rvec, rotationMatrix);
        Mat projectionMatrix = new Mat(4, 4, CvType.CV_64F);
        Mat projectionMatrix1 = new Mat(3, 4, CvType.CV_64F);
        Mat b1 = new Mat(3, 4, CvType.CV_64F);

        projectionMatrix.put(0, 0,
                rotationMatrix.get(0, 0)[0], rotationMatrix.get(0, 1)[0], rotationMatrix.get(0, 2)[0], tvec.get(0, 0)[0],
                rotationMatrix.get(1, 0)[0], rotationMatrix.get(1, 1)[0], rotationMatrix.get(1, 2)[0], tvec.get(1, 0)[0],
                rotationMatrix.get(2, 0)[0], rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0], tvec.get(2, 0)[0],
                0,0,0,1
        );

        projectionMatrix1.put(0, 0,
                rotationMatrix.get(0, 0)[0], rotationMatrix.get(0, 1)[0], rotationMatrix.get(0, 2)[0], tvec.get(0, 0)[0],
                rotationMatrix.get(1, 0)[0], rotationMatrix.get(1, 1)[0], rotationMatrix.get(1, 2)[0], tvec.get(1, 0)[0],
                rotationMatrix.get(2, 0)[0], rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0], tvec.get(2, 0)[0]
        );

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

// Print function

        double tx = Double.parseDouble((String.valueOf((b1.get(0,3)[0])/1000)));
        double ty = Double.parseDouble((String.valueOf((b1.get(1,3)[0])/1000)));
        double tz = Double.parseDouble((String.valueOf((b1.get(2,3)[0])/1000)));
        double tpitch = Double.parseDouble((String.valueOf(eulerAngles.get(0,0)[0])));
        double tyaw = Double.parseDouble((String.valueOf(eulerAngles.get(1,0)[0])));
        double troll = Double.parseDouble((String.valueOf(eulerAngles.get(2,0)[0])));
        Intent intent = new Intent(this, MainActivityL.class);
        intent.putExtra(EXTRA_NUMBER1,tx);
        intent.putExtra(EXTRA_NUMBER2,ty);
        intent.putExtra(EXTRA_NUMBER3,tz);
        intent.putExtra(EXTRA_NUMBER4,tpitch);
        intent.putExtra(EXTRA_NUMBER5,tyaw);
        intent.putExtra(EXTRA_NUMBER6,troll);
        startActivity(intent);

        theTextView1.setText("P1:" + String.format("%.2f",tp11f) + "," + String.format("%.2f",tp12f));
        theTextView1.setTextColor(Color.RED);

        theTextView2.setText("P2:" + String.format("%.2f",tp21f) + "," + String.format("%.2f",tp22f));
        theTextView2.setTextColor(Color.RED);

        theTextView3.setText("P3:" + String.format("%.2f",tp31f) + "," + String.format("%.2f",tp32f));
        theTextView3.setTextColor(Color.RED);

        theTextView4.setText("P4:" + String.format("%.2f",tp41f) + "," + String.format("%.2f",tp42f));
        theTextView4.setTextColor(Color.RED);

        return srcImg1;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case 1: {
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                } else {
                    Toast.makeText(ColorDetection.this, "Permission denied to read your External storage", Toast.LENGTH_SHORT).show();
                }
                return;
            }
        }
    }


    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            _baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

}