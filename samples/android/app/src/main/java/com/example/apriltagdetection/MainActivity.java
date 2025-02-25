package com.example.apriltagdetection;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import androidx.annotation.NonNull;
import com.google.android.material.snackbar.Snackbar ;
import androidx.core.app.ActivityCompat;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import org.visp.core.VpImageUChar;


public class MainActivity extends AppCompatActivity implements ActivityCompat.OnRequestPermissionsResultCallback {

    private static final int PERMISSION_REQUEST_CAMERA = 0;
    private View mLayout;


    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("visp_java3");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mLayout = findViewById(R.id.main_layout);

        Button button = findViewById(R.id.ButtonSelectImage);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showCameraPreview();
            }
        });

        findViewById(R.id.getTags).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent browserIntent = new Intent("android.intent.action.VIEW", android.net.Uri.parse("https://april.eecs.umich.edu/media/apriltag/?C=D;O=D"));
                startActivity(browserIntent);
            }
        });

        findViewById(R.id.project).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent browserIntent = new Intent("android.intent.action.VIEW", android.net.Uri.parse("https://summerofcode.withgoogle.com/archive/2018/projects/5711312218750976/"));
                startActivity(browserIntent);
            }
        });

        findViewById(R.id.credits).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent browserIntent = new Intent("android.intent.action.VIEW", android.net.Uri.parse("https://www.facebook.com/1996AKS"));
                startActivity(browserIntent);
            }
        });
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult( requestCode, permissions, grantResults);
        if (requestCode == PERMISSION_REQUEST_CAMERA) {
            // Request for camera permission.
            if (grantResults.length == 1 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Permission has been granted. Start camera preview Activity.
                Snackbar.make(mLayout, "Camera permission was granted. Starting preview.",
                                Snackbar.LENGTH_SHORT)
                        .show();
                startCamera();
            } else {
                // Permission request was denied.
                Snackbar.make(mLayout, "Camera permission request was denied.",
                                Snackbar.LENGTH_SHORT)
                        .show();
            }
        }
    }

    private void showCameraPreview() {
        // Check if the Camera permission has been granted
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            // Permission is already available, start camera preview
            Snackbar.make(mLayout,
                    "Camera permission is available. Starting preview.",
                    Snackbar.LENGTH_SHORT).show();
            startCamera();
        } else {
            // Permission is missing and must be requested.
            requestCameraPermission();
        }
    }

    /**
     * Requests the {@link android.Manifest.permission#CAMERA} permission.
     * If an additional rationale should be displayed, the user has to launch the request from
     * a SnackBar that includes additional information.
     */
    private void requestCameraPermission() {
        // Permission has not been granted and must be requested.
        if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                Manifest.permission.CAMERA)) {
            // Provide an additional rationale to the user if the permission was not granted
            // and the user would benefit from additional context for the use of the permission.
            // Display a SnackBar with a button to request the missing permission.
            Snackbar.make(mLayout, "Camera access is required to display the camera preview.",
                    Snackbar.LENGTH_INDEFINITE).setAction("OK", new View.OnClickListener() {
                @Override
                public void onClick(View view) {
                    // Request the permission
                    ActivityCompat.requestPermissions(MainActivity.this,
                            new String[]{Manifest.permission.CAMERA},
                            PERMISSION_REQUEST_CAMERA);
                }
            }).show();

        } else {
            Snackbar.make(mLayout,
                    "Permission is not available. Requesting camera permission.",
                    Snackbar.LENGTH_SHORT).show();
            // Request the permission. The result will be received in onRequestPermissionResult().
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA},
                    PERMISSION_REQUEST_CAMERA);
        }
    }

    private void startCamera() {
        Intent intent = new Intent(this, CameraPreviewActivity.class);
        startActivity(intent);
    }
}
