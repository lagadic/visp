package com.example.apriltagdetection;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.util.List;
import java.util.Locale;

public class DisplaySurfaceView extends SurfaceView implements SurfaceHolder.Callback {
    private static final String TAG = "DisplaySurfaceView";
    private SurfaceHolder mSurfaceHolder;

    public DisplaySurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        mSurfaceHolder = getHolder();
        mSurfaceHolder.addCallback(this);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
    }

    public void clear() {
        Canvas canvas = mSurfaceHolder.lockCanvas();
        if (canvas != null) {
            // https://stackoverflow.com/a/9035709
            canvas.drawColor( 0, PorterDuff.Mode.CLEAR );
            mSurfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    public void drawLines(List<double[]> list_startX, List<double[]> list_startY, List<double[]> list_stopX, List<double[]> list_stopY,
                          int[] color_, int[] strokeWidth_, List<Double> centerX, List<Double> centerY, int[] ids_, double[] dist_,
                          int orientation, int width, int height, boolean draw_frame, List<double[]> list_frameX, List<double[]> list_frameY) {
        Canvas canvas = mSurfaceHolder.lockCanvas();

        if (canvas != null) {
            float view_w = getWidth();
            float view_h = getHeight();

            for (int i = 0; i < list_startX.size(); i++) {
                for (int j = 0; j < list_startX.get(i).length; j++) {
                    double startX = list_startX.get(i)[j];
                    double stopX = list_stopX.get(i)[j];
                    double startY = list_startY.get(i)[j];
                    double stopY = list_stopY.get(i)[j];

                    int color = color_[j];
                    int strokeWidth = strokeWidth_[j];

                    Paint paint = new Paint();
                    paint.setColor(color);
                    paint.setStrokeWidth(strokeWidth);

                    canvasDrawLine(canvas, startX, startY, stopX, stopY, orientation, width, height, paint);
                }

                // Draw id
                Paint paint = new Paint();
                paint.setColor(Color.parseColor("aqua"));
                paint.setStrokeWidth(8);
                paint.setTextSize(50);
                paint.setTextAlign(Paint.Align.CENTER);

                Paint paint2 = new Paint(paint);
                paint2.setColor(Color.parseColor("fuchsia"));

                int offset_dist = 50;
                if (orientation == 0) {
                    float scaleX = view_w / width;
                    float scaleY = view_h / height;

                    canvas.drawText(String.valueOf(ids_[i]), scaleX * centerX.get(i).floatValue(), scaleY * centerY.get(i).floatValue(), paint);

                    canvas.drawText(String.format(Locale.US, "%.2f", dist_[i]) + " m", scaleX * centerX.get(i).floatValue(),
                            scaleY * centerY.get(i).floatValue() + offset_dist, paint2);
                } else if (orientation == 90) {
                    float scaleX = view_w / height;
                    float scaleY = view_h / width;

                    canvas.drawText(String.valueOf(ids_[i]), scaleX * (view_w - centerY.get(i).floatValue()), scaleY * (centerX.get(i).floatValue()), paint);
                    canvas.drawText(String.format(Locale.US, "%.2f", dist_[i]) + " m", scaleX * (view_w - centerY.get(i).floatValue()),
                            scaleY * (centerX.get(i).floatValue()) + offset_dist, paint2);
                } else {
                    // 180°
                    float scaleX = view_w / width;
                    float scaleY = view_h / height;

                    canvas.drawText(String.valueOf(ids_[i]), scaleX * (width - centerX.get(i).floatValue()), scaleY * (height - centerY.get(i).floatValue()), paint);
                    canvas.drawText(String.format(Locale.US, "%.2f", dist_[i]) + " m", scaleX * (width - centerX.get(i).floatValue()),
                            scaleY * (height - centerY.get(i).floatValue()) + offset_dist, paint2);
                }

                // Draw tag frame
                if (draw_frame) {
                    Paint paint_frame = new Paint();
                    paint_frame.setStrokeWidth(8);

                    // oX
                    {
                        paint_frame.setColor(Color.parseColor("red"));
                        double startX = list_frameX.get(i)[0];
                        double stopX = list_frameX.get(i)[1];
                        double startY = list_frameY.get(i)[0];
                        double stopY = list_frameY.get(i)[1];

                        canvasDrawLine(canvas, startX, startY, stopX, stopY, orientation, width, height, paint_frame);
                    }
                    // oY
                    {
                        paint_frame.setColor(Color.parseColor("green"));

                        double startX = list_frameX.get(i)[0];
                        double stopX = list_frameX.get(i)[2];
                        double startY = list_frameY.get(i)[0];
                        double stopY = list_frameY.get(i)[2];

                        canvasDrawLine(canvas, startX, startY, stopX, stopY, orientation, width, height, paint_frame);
                    }
                    // oZ
                    {
                        paint_frame.setColor(Color.parseColor("blue"));

                        double startX = list_frameX.get(i)[0];
                        double stopX = list_frameX.get(i)[3];
                        double startY = list_frameY.get(i)[0];
                        double stopY = list_frameY.get(i)[3];

                        canvasDrawLine(canvas, startX, startY, stopX, stopY, orientation, width, height, paint_frame);
                    }
                }
            }

            mSurfaceHolder.unlockCanvasAndPost(canvas);
        }
    }

    private void canvasDrawLine(Canvas canvas, double startX, double startY, double stopX, double stopY, int orientation, int width, int height, Paint paint) {
        float view_w = getWidth();
        float view_h = getHeight();

        if (orientation == 0) {
            double scaleX = view_w / width;
            double scaleY = view_h / height;

            canvas.drawLine((float) (scaleX * startX), (float) (scaleY * startY),
                    (float) (scaleX * stopX), (float) (scaleY * stopY), paint);
        } else if (orientation == 90) {
            double scaleX = view_w / height;
            double scaleY = view_h / width;

            canvas.drawLine((float) (scaleX * (view_w - startY)), (float) (scaleY * startX),
                    (float) (scaleX * (view_w - stopY)), (float) (scaleY * stopX), paint);
        } else {
            // 180°
            double scaleX = view_w / width;
            double scaleY = view_h / height;

            canvas.drawLine((float) (scaleX * (width - startX)), (float) (scaleY * (height - startY)),
                    (float) (scaleX * (width - stopX)), (float) (scaleY * (height - stopY)), paint);
        }
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
    }
}
