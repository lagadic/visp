package org.visp.test;

import java.io.File;
import java.io.IOException;

import org.visp.core.Mat;

public class ViSPTestRunner {
    public static String LENA_PATH = "";
    public static String CHESS_PATH = "";
    public static String LBPCASCADE_FRONTALFACE_PATH = "";

    private static String TAG = "visp_test_java";


    public static String getTempFileName(String extension)
    {
        if (!extension.startsWith("."))
            extension = "." + extension;
        try {
            File tmp = File.createTempFile("ViSP", extension);
            String path = tmp.getAbsolutePath();
            tmp.delete();
            return path;
        } catch (IOException e) {
            Log("Failed to get temp file name. Exception is thrown: " + e);
        }
        return null;
    }

    static public void Log(String message) {
        System.out.println(TAG + " :: " +  message);
    }

    static public void Log(Mat m) {
        System.out.println(TAG + " :: " + m + "\n " + m.dump());
    }

    public static String getOutputFileName(String name)
    {
        return getTempFileName(name);
    }
}
