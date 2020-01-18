package org.visp.android;

import java.text.DecimalFormat;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

public class FpsMeter {
    private static final String TAG               = "FpsMeter";
    private static final int    STEP              = 20;
    private static final DecimalFormat FPS_FORMAT = new DecimalFormat("0.00");

    private int                 mFramesCouner;
    private double              mFrequency;
    private long                mprevFrameTime;
    private String              mStrfps;
    Paint                       mPaint;
    boolean                     mIsInitialized = false;
    int                         mWidth = 0;
    int                         mHeight = 0;

    public void init(int tickFreq, long tickCount) {
        mFramesCouner = 0;
        mFrequency = tickFreq;
        mprevFrameTime = tickCount;
        mStrfps = "";

        mPaint = new Paint();
        mPaint.setColor(Color.BLUE);
        mPaint.setTextSize(20);
    }

    public void measure(int tickFreq, long tickCount) {
        if (!mIsInitialized) {
            init(tickFreq,tickCount);
            mIsInitialized = true;
        } else {
            mFramesCouner++;
            if (mFramesCouner % STEP == 0) {
                long time = tickCount;
                double fps = STEP * mFrequency / (time - mprevFrameTime);
                mprevFrameTime = time;
                if (mWidth != 0 && mHeight != 0)
                    mStrfps = FPS_FORMAT.format(fps) + " FPS@" + Integer.valueOf(mWidth) + "x" + Integer.valueOf(mHeight);
                else
                    mStrfps = FPS_FORMAT.format(fps) + " FPS";
                Log.i(TAG, mStrfps);
            }
        }
    }

    public void setResolution(int width, int height) {
        mWidth = width;
        mHeight = height;
    }

    public void draw(Canvas canvas, float offsetx, float offsety) {
        Log.d(TAG, mStrfps);
        canvas.drawText(mStrfps, offsetx, offsety, mPaint);
    }

}
