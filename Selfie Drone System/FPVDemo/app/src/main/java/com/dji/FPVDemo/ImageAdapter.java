package com.dji.FPVDemo;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.GridView;
import android.widget.ImageView;
import android.widget.TextView;

public class ImageAdapter extends BaseAdapter {
    private Context mContext;
    private final LayoutInflater mInflater;

    // Keep all Template Images in array
    public Integer[] mThumbIds = {
            R.drawable.temp1, R.drawable.temp2,
            R.drawable.temp3, R.drawable.temp4,
            R.drawable.temp5, R.drawable.temp6,
            R.drawable.temp7, R.drawable.temp8,
            R.drawable.temp9, R.drawable.temp10,
            R.drawable.temp11, R.drawable.temp12,
    };

    // Constructor
    public ImageAdapter(Context mContext){
        this.mContext=mContext;
        mInflater = LayoutInflater.from(mContext);
    }

    @Override
    public int getCount() {
        return mThumbIds.length;
    }

    @Override
    public Object getItem(int position) {
        return mThumbIds[position];
    }

    @Override
    public long getItemId(int position) {
        return 0;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent) {
        ImageView imageView = new ImageView(mContext);
        imageView.setImageResource(mThumbIds[position]);
        imageView.setScaleType(ImageView.ScaleType.CENTER_CROP);
        imageView.setLayoutParams(new GridView.LayoutParams(545,400));
        return imageView;

    }

}