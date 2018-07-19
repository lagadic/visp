package org.visp.android;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;

import org.visp.core.VpImageUChar;
import org.visp.core.VpImageRGBa;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class Utils {

    public static String exportResource(Context context, int resourceId) {
        return exportResource(context, resourceId, "ViSP_data");
    }

    public static String exportResource(Context context, int resourceId, String dirname) {
        String fullname = context.getResources().getString(resourceId);
        String resName = fullname.substring(fullname.lastIndexOf("/") + 1);
        try {
            InputStream is = context.getResources().openRawResource(resourceId);
            File resDir = context.getDir(dirname, Context.MODE_PRIVATE);
            File resFile = new File(resDir, resName);

            FileOutputStream os = new FileOutputStream(resFile);

            byte[] buffer = new byte[4096];
            int bytesRead;
            while ((bytesRead = is.read(buffer)) != -1) {
                os.write(buffer, 0, bytesRead);
            }
            is.close();
            os.close();

            return resFile.getAbsolutePath();
        } catch (IOException e) {
            e.printStackTrace();
            Log.e("VISP::Utils.java","Failed to export resource " + resName + ". Exception thrown: " + e);
        }
				return null;
    }

    public static VpImageUChar loadResource(Context context, int resourceId) throws IOException
    {
        InputStream is = context.getResources().openRawResource(resourceId);
        ByteArrayOutputStream os = new ByteArrayOutputStream(is.available());

        byte[] buffer = new byte[4096];
        int bytesRead;
        while ((bytesRead = is.read(buffer)) != -1)
            os.write(buffer, 0, bytesRead);
        is.close();

        VpImageUChar encoded = new VpImageUChar(os.toByteArray(), 1, os.size(), true);
        os.close();
        return encoded;
    }

	// Converts Android Bitmap to ViSP VpImageUChar
    public static void bitmapToVpImageUChar(Bitmap bmp, VpImageUChar image) {
        if (bmp == null)
            throw new java.lang.IllegalArgumentException("bmp == null");
        if (image == null)
            throw new java.lang.IllegalArgumentException("image == null");
        bitmapToVpImageUChar(bmp, image.nativeObj);
    }

	// Converts VpImageUChar to Android Bitmap
    public static void vpImageUCharToBitmap(VpImageUChar image, Bitmap bmp) {
        if (image == null)
            throw new java.lang.IllegalArgumentException("image == null");
        if (bmp == null)
            throw new java.lang.IllegalArgumentException("bmp == null");
        vpImageUCharToBitmap(image.nativeObj, bmp);
    }

	// Converts Android Bitmap to ViSP VpImageRGBa
    public static void bitmapToVpImageRGBa(Bitmap bmp, VpImageRGBa image) {
        if (bmp == null)
            throw new java.lang.IllegalArgumentException("bmp == null");
        if (image == null)
            throw new java.lang.IllegalArgumentException("image == null");
        bitmapToVpImageRGBa(bmp, image.nativeObj);
    }

	// Converts VpImageRGBa to Android Bitmap
    public static void vpImageRGBaToBitmap(VpImageRGBa image, Bitmap bmp) {
        if (image == null)
            throw new java.lang.IllegalArgumentException("image == null");
        if (bmp == null)
            throw new java.lang.IllegalArgumentException("bmp == null");
        vpImageRGBaToBitmap(image.nativeObj, bmp);
    }

    public static native void bitmapToVpImageUChar(Bitmap b, long m_addr);

    public static native void vpImageUCharToBitmap(long m_addr, Bitmap b);

    public static native void bitmapToVpImageRGBa(Bitmap b, long m_addr);

    public static native void vpImageRGBaToBitmap(long m_addr, Bitmap b);
}
