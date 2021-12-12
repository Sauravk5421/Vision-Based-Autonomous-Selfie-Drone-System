package com.dji.FPVDemo;

import android.content.Intent;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PointF;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.GridView;
import android.widget.ImageView;
import android.widget.RelativeLayout;

import androidx.appcompat.app.AppCompatActivity;

import java.util.ArrayList;

import dji.common.error.DJIError;
import dji.common.util.CommonCallbacks;


public class TemplateActivity extends AppCompatActivity {
    private static final String TAG = TemplateActivity.class.getSimpleName();
    static boolean isOpenCVInit=false;
    //private ImageClassifier classifier = null;
    GridView gridView;
    //private DrawView drawView = null;
    private float[][] keypoints=null;
    private ImageView imageView=null;
    private int width,height;
    ArrayList<PointF> mDrawPoint;
    Boolean direction;
    float[] body_features=new float[9];
    Paint paint = new Paint();
    private RelativeLayout template_layout;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.grid_layout);
        gridView = (GridView) findViewById(R.id.grid_view);
        gridView.setAdapter(new ImageAdapter( this));

        gridView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int position, long l) {
                Intent intent = new Intent(getApplicationContext(),ColorDetection.class);
                intent.putExtra("id", position);
                Canvas canvas = new Canvas();
                canvas.drawColor(Color.RED);
                canvas.drawLine(0, 0, 20, 20, paint);
                canvas.drawLine(20, 0, 0, 20, paint);
                startActivity(intent);

            }
        });

    }

}
