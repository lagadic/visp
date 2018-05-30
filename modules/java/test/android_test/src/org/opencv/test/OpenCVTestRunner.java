package org.visp.test;

import java.io.File;
import java.io.IOException;
import junit.framework.Assert;

import org.visp.android.BaseLoaderCallback;
import org.visp.android.LoaderCallbackInterface;
import org.visp.android.ViSPLoader;
import org.visp.android.Utils;
import org.visp.core.Mat;

import android.content.Context;
import android.test.AndroidTestRunner;
import android.test.InstrumentationTestRunner;
import android.util.Log;

/**
 * This only class is Android specific.
 */

public class ViSPTestRunner extends InstrumentationTestRunner {

    private static final long MANAGER_TIMEOUT = 3000;
    public static String LENA_PATH;
    public static String CHESS_PATH;
    public static String LBPCASCADE_FRONTALFACE_PATH;
    public static Context context;

    private AndroidTestRunner androidTestRunner;
    private static String TAG = "visp_test_java";

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(getContext()) {

        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log("ViSP loaded successfully");
                    synchronized (this) {
                        notify();
                    }
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public static String getTempFileName(String extension)
    {
        File cache = context.getCacheDir();
        if (!extension.startsWith("."))
            extension = "." + extension;
        try {
            File tmp = File.createTempFile("OpenCV", extension, cache);
            String path = tmp.getAbsolutePath();
            tmp.delete();
            return path;
        } catch (IOException e) {
            Log("Failed to get temp file name. Exception is thrown: " + e);
        }
        return null;
    }

    static public void Log(String message) {
        Log.e(TAG, message);
    }

    static public void Log(Mat m) {
        Log.e(TAG, m + "\n " + m.dump());
    }

    @Override
    public void onStart() {
        // try to load internal libs
        if (!ViSPLoader.initDebug()) {
            // There is no internal ViSP libs
            // Using ViSP Manager for initialization;

            Log("Internal ViSP library not found. Using ViSP Manager for initialization");
            ViSPLoader.initAsync(ViSPLoader.VISP_VERSION, getContext(), mLoaderCallback);

            synchronized (this) {
                try {
                    wait(MANAGER_TIMEOUT);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } else {
            Log("ViSP library found inside test package. Using it!");
        }

        context = getContext();
        Assert.assertTrue("Context can't be 'null'", context != null);
        LENA_PATH = Utils.exportResource(context, R.drawable.lena);
        CHESS_PATH = Utils.exportResource(context, R.drawable.chessboard);
        LBPCASCADE_FRONTALFACE_PATH = Utils.exportResource(context, R.raw.lbpcascade_frontalface);

        /*
         * The original idea about test order randomization is from
         * marek.defecinski blog.
         */
        //List<TestCase> testCases = androidTestRunner.getTestCases();
        //Collections.shuffle(testCases); //shuffle the tests order

        super.onStart();
    }

    @Override
    protected AndroidTestRunner getAndroidTestRunner() {
        androidTestRunner = super.getAndroidTestRunner();
        return androidTestRunner;
    }

    public static String getOutputFileName(String name)
    {
        return context.getExternalFilesDir(null).getAbsolutePath() + File.separatorChar + name;
    }
}
