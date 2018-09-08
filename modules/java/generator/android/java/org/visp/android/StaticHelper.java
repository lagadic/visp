package org.visp.android;

import java.util.StringTokenizer;
import android.util.Log;

class StaticHelper {
    public static boolean initViSP()
    {
        boolean result;
        String libs = "";

        Log.d(TAG, "Trying to get library list");

        try
        {
            System.loadLibrary("visp_core");
            libs = getLibraryList();
        }
        catch(UnsatisfiedLinkError e)
        {
            Log.e(TAG, "ViSP error: Cannot load info library for ViSP");
        }

        Log.d(TAG, "Library list: \"" + libs + "\"");
        Log.d(TAG, "First attempt to load libs");
        if (initViSPLibs(libs))
        {
            Log.d(TAG, "First attempt to load libs is OK");
            result = true;
        }
        else
        {
            Log.d(TAG, "First attempt to load libs fails");
            result = false;
        }

        return result;
    }

    private static boolean loadLibrary(String Name)
    {
        boolean result = true;

        Log.d(TAG, "Trying to load library " + Name);
        try
        {
            System.loadLibrary(Name);
            Log.d(TAG, "Library " + Name + " loaded");
        }
        catch(UnsatisfiedLinkError e)
        {
            Log.d(TAG, "Cannot load library \"" + Name + "\"");
            e.printStackTrace();
            result = false;
        }

        return result;
    }

    private static boolean initViSPLibs(String Libs)
    {
        Log.d(TAG, "Trying to init ViSP libs");

        boolean result = true;

        if ((null != Libs) && (Libs.length() != 0))
        {
            Log.d(TAG, "Trying to load libs by dependency list");
            StringTokenizer splitter = new StringTokenizer(Libs, ";");
            while(splitter.hasMoreTokens())
            {
                result &= loadLibrary(splitter.nextToken());
            }
        }
        else
        {
            // If dependencies list is not defined or empty.
            result = loadLibrary("visp_java");
        }

        return result;
    }

    private static final String TAG = "ViSP/StaticHelper";

    private static native String getLibraryList();
}
