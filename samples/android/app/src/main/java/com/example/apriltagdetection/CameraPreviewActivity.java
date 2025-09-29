package com.example.apriltagdetection;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.graphics.Color;
import android.os.Bundle;

import android.graphics.PixelFormat;
import android.hardware.Camera;
import android.util.Log;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.view.View;

import org.visp.core.VpCameraParameters;
import org.visp.core.VpHomogeneousMatrix;
import org.visp.core.VpImagePoint;
import org.visp.core.VpPoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Displays a {@link CameraPreview} of the first {@link Camera}.
 * An error message is displayed if the Camera is not available.
 * <p>
 * This Activity is only used to illustrate that access to the Camera API has been granted (or
 * denied) as part of the runtime permissions model. It is not relevant for the use of the
 * permissions API.
 * <p>
 * Implementation is based directly on the documentation at
 * https://developer.android.com/media/camera/camera-deprecated/camera-api
 */
public class CameraPreviewActivity extends MainActivity  {
    /**
     * Id of the camera to access. 0 is the first camera.
     */
    private static final int CAMERA_ID = 0;
    private static final String TAG = "CameraPreviewActivity";
    private static final int SETTINGS_REQUEST_CODE = 1;

    Camera.CameraInfo mCameraInfo;
    private Camera mCamera;
    private int mW, mH;
    static TextView mResultInfo;
    static DisplaySurfaceView mLineSurface;
    private Spinner mSpinner;
    FrameLayout mFrameLayout;
    private CameraPreview mPreview;
    private Button mBtnSettings;
    private boolean mCameraFlash;
    private double mFocalLength;
    private double mTagSize;
    private Button mBtnAutoFocus;
    private boolean mUpdatedFocal;
    private boolean mUpdatedTagSize;
    private boolean mDisplayTagFrame;
    private double mTagFrameDisplayRatio;
    private int mAprilTagSelectionPosition;
    private int mAprilTagQuadDecimate;
    private int mAprilTagDecisionMargin;
    private int mAprilTagNbThreads;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.d(TAG, "CameraPreviewActivity::onCreate()");
        super.onCreate(savedInstanceState);

        // Arbitrary value for the request code? --> https://stackoverflow.com/a/36653669 ?
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, 100);
        }

        mUpdatedFocal = false;
        mUpdatedTagSize = false;
        init();
    }

    private void init() {
        // Open an instance of the first camera and retrieve its info.
        mCameraInfo = new Camera.CameraInfo();
        mCameraFlash = false;
        mCamera = getCameraInstance(CAMERA_ID);

        if (mCamera == null) {
            // Camera is not available, display error message
            Toast.makeText(this, "Camera is not available.", Toast.LENGTH_SHORT).show();
            setContentView(R.layout.camera_unavailable);

            return;
        }

        Camera.getCameraInfo(CAMERA_ID, mCameraInfo);
        setContentView(R.layout.activity_camera_preview);

        mBtnSettings = findViewById(R.id.btnSettings);
        mBtnSettings.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Camera.Parameters params = mCamera.getParameters();
                // the focal length. Returns -1.0 when the device doesn't report focal length information.
                float focalLength_mm = params.getFocalLength();
                Camera.Size size = params.getPictureSize();

                // Physical sensor size
                // See: https://stackoverflow.com/a/41032402/6055233
                // "horizontal angle of view. Returns -1.0 when the device doesn't report view angle information."
                float horizontalViewAngle = params.getHorizontalViewAngle();
                // "vertical angle of view. Returns -1.0 when the device doesn't report view angle information."
                float verticalViewAngle = params.getVerticalViewAngle();
                double sensorWidth = focalLength_mm * 2*Math.tan(Math.toRadians(horizontalViewAngle/2));
                double sensorHeight = focalLength_mm * 2*Math.tan(Math.toRadians(verticalViewAngle/2));

                mCamera.setPreviewCallback(null);
                mFrameLayout.removeView(mPreview);
                mPreview = null;
                releaseCamera();

                Intent intent = new Intent(CameraPreviewActivity.this, SettingsPanelActivity.class);

                intent.putExtra("camera_flash", mCameraFlash);

                intent.putExtra("camera_focal_mm", focalLength_mm);
                intent.putExtra("camera_native_w", size.width);
                intent.putExtra("camera_native_h", size.height);
                intent.putExtra("camera_hfov", horizontalViewAngle);
                intent.putExtra("camera_vfov", verticalViewAngle);
                intent.putExtra("camera_sensor_w", sensorWidth);
                intent.putExtra("camera_sensor_h", sensorHeight);

                intent.putExtra("image_w", mW);
                intent.putExtra("image_h", mH);

                intent.putExtra("focal", mFocalLength);
                intent.putExtra("tag_size", mTagSize);

                intent.putExtra("display_tag_frame", mDisplayTagFrame);
                intent.putExtra("display_tag_frame_ratio", mTagFrameDisplayRatio);

                intent.putExtra("aprilTag_selection_position", mAprilTagSelectionPosition);
                intent.putExtra("aprilTag_quad_decimate", mAprilTagQuadDecimate);
                intent.putExtra("aprilTag_decision_margin", mAprilTagDecisionMargin);
                intent.putExtra("aprilTag_nb_threads", mAprilTagNbThreads);

                startActivityForResult(intent, SETTINGS_REQUEST_CODE);
            }
        });

        mBtnAutoFocus = findViewById(R.id.btnAutoFocus);
        // Set up the autofocus button
        mBtnAutoFocus.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mCamera != null) {
                    mCamera.autoFocus(new Camera.AutoFocusCallback() {
                        @Override
                        public void onAutoFocus(boolean success, Camera camera) {
                            if (!success) {
                                Toast.makeText(CameraPreviewActivity.this, "Cannot perform camera autofocus",
                                        Toast.LENGTH_SHORT).show();
                            }
                        }
                    });
                }
            }
        });

        mSpinner = findViewById(R.id.spinner);
        String[] items = {
                "TAG_36h11", "TAG_25h9", "TAG_25h7", "TAG_16h5", "TAG_CIRCLE21h7",
                "TAG_ARUCO_4x4", "TAG_ARUCO_5x5", "TAG_ARUCO_6x6", "TAG_ARUCO_MIP_36h12"
        };

        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, items);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        mSpinner.setAdapter(adapter);
        mAprilTagSelectionPosition = 0;
        mSpinner.setSelection(mAprilTagSelectionPosition);

        mSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parentView, View selectedItemView, int position, long id) {
                String selectedItem = parentView.getItemAtPosition(position).toString();
                Toast.makeText(CameraPreviewActivity.this, "Selected: " + selectedItem, Toast.LENGTH_SHORT).show();

                mAprilTagSelectionPosition = position;
                mPreview.setAprilTagMethod(position);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parentView) {
                Toast.makeText(CameraPreviewActivity.this, "No item selected", Toast.LENGTH_SHORT).show();
            }
        });

        mResultInfo = findViewById(R.id.resultTV);
        mLineSurface = findViewById(R.id.surfaceView);
        mLineSurface.setZOrderOnTop(true);
        mLineSurface.getHolder().setFormat(PixelFormat.TRANSLUCENT);

        // init the byte array
        mW = mCamera.getParameters().getPreviewSize().width;
        mH = mCamera.getParameters().getPreviewSize().height;

        // Get the rotation of the screen to adjust the preview image accordingly.
        final int displayRotation = getWindowManager().getDefaultDisplay().getRotation();

        // Create the Preview view and set it as the content of this Activity.
        mPreview = new CameraPreview(this, mCamera, mCameraInfo, displayRotation, mDisplayTagFrame, mTagFrameDisplayRatio);
        mFrameLayout = findViewById(R.id.camera_preview);
        mFrameLayout.addView(mPreview);

        if (!mUpdatedFocal) {
            Camera.Parameters params = mCamera.getParameters();
            float focalLength_mm = params.getFocalLength();
            Camera.Size size = params.getPictureSize();

            float horizontalViewAngle = params.getHorizontalViewAngle();
            if (horizontalViewAngle > 0) {
                double sensorWidth = focalLength_mm * 2*Math.tan(Math.toRadians(horizontalViewAngle/2));

                // Focal length computed from sensor specs
                mFocalLength = focalLength_mm / (sensorWidth / size.width);
                // Scale the focal length wrt. the current image resolution
                mFocalLength = mFocalLength * mW / size.width;
            } else {
                mFocalLength = 600;
            }

            mPreview.setCameraFocal(mFocalLength);
        }
        if (!mUpdatedTagSize) {
            mTagSize = 0.1;
            mPreview.setTagSize(mTagSize);
        }

        mDisplayTagFrame = false;
        mTagFrameDisplayRatio = 0.5;

        mAprilTagSelectionPosition = 0;
        mAprilTagQuadDecimate = 2;
        mPreview.setAprilTagQuadDecimate(mAprilTagQuadDecimate);
        mAprilTagDecisionMargin = 50;
        mPreview.setAprilTagMarginThreshold(mAprilTagDecisionMargin);
        mAprilTagNbThreads = 1;
        mPreview.setAprilTagNbThreads(mAprilTagNbThreads);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        init();

        if (requestCode == SETTINGS_REQUEST_CODE && resultCode == RESULT_OK) {
            String cameraFlash_str = data.getStringExtra("cameraFlashChecked");
            if (cameraFlash_str != null) {
                mCameraFlash = Boolean.parseBoolean(cameraFlash_str);

                // Camera flash
                Camera.Parameters cam_params = mCamera.getParameters();
                if (mCameraFlash && cam_params != null) {
                    if (cam_params.getSupportedFlashModes() != null && cam_params.getSupportedFlashModes().contains(Camera.Parameters.FLASH_MODE_TORCH)) {
                        cam_params.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
                        mCamera.setParameters(cam_params);
                    } else {
                        Toast.makeText(this, "Camera flash is not available.", Toast.LENGTH_SHORT).show();
                    }
                }
            }

            String cameraFocalValue_str = data.getStringExtra("cameraFocalValue");
            if (cameraFocalValue_str != null) {
                mFocalLength = Double.parseDouble(cameraFocalValue_str);
                mPreview.setCameraFocal(mFocalLength);

                mUpdatedFocal = true;
            }

            String tagSizeValue_str = data.getStringExtra("tagSizeValue");
            if (tagSizeValue_str != null) {
                mTagSize = Double.parseDouble(tagSizeValue_str);
                mPreview.setTagSize(mTagSize);

                mUpdatedTagSize = true;
            }

            String displayTagFrame_str = data.getStringExtra("displayFrameChecked");
            if (displayTagFrame_str != null) {
                mDisplayTagFrame = Boolean.parseBoolean(displayTagFrame_str);
                mPreview.setDisplayTagFrame(mDisplayTagFrame);

                String tagFrameSizeRatio_str = data.getStringExtra("tagFrameRatio");
                if (tagFrameSizeRatio_str != null) {
                    mTagFrameDisplayRatio = Double.parseDouble(tagFrameSizeRatio_str) > 0 ? Double.parseDouble(tagFrameSizeRatio_str) : mTagFrameDisplayRatio;
                    mPreview.setDisplayTagFrameRatio(mTagFrameDisplayRatio);
                }
            }

            // Set the AprilTag method selection to the correct value
            String aprilTagSelectionPosition_str = data.getStringExtra("aprilTagSelectionPosition");
            if (aprilTagSelectionPosition_str != null) {
                mAprilTagSelectionPosition = Integer.parseInt(aprilTagSelectionPosition_str);
                mSpinner.setSelection(mAprilTagSelectionPosition);
                mPreview.setAprilTagMethod(mAprilTagSelectionPosition);
            }

            String aprilTagQuadDecimate_str = data.getStringExtra("aprilTagQuadDecimate");
            if (aprilTagQuadDecimate_str != null) {
                mAprilTagQuadDecimate = Integer.parseInt(aprilTagQuadDecimate_str);
                mPreview.setAprilTagQuadDecimate(mAprilTagQuadDecimate);
            }

            String aprilTagDecisionMargin_str = data.getStringExtra("aprilTagDecisionMargin");
            if (aprilTagDecisionMargin_str != null) {
                mAprilTagDecisionMargin = Integer.parseInt(aprilTagDecisionMargin_str);
                mPreview.setAprilTagMarginThreshold(mAprilTagDecisionMargin);
            }

            String aprilTagNbThreads_str = data.getStringExtra("aprilTagNbThreads");
            if (aprilTagNbThreads_str != null) {
                mAprilTagNbThreads = Integer.parseInt(aprilTagNbThreads_str);
                mPreview.setAprilTagNbThreads(mAprilTagNbThreads);
            }
        }
    }

    public static void updateResults(List<List<VpImagePoint>> cornersList, int strokeWidth, int[] ids, List<VpHomogeneousMatrix> cMoList,
                                     String s, int orientation, int w, int h, boolean displayTagFrame, double tagFrameRatio,
                                     double tagSize, VpCameraParameters cam) {
        int RED = Color.RED;        // -65536
        int GREEN = Color.GREEN;    // -16711936
        int YELLOW = Color.YELLOW;  // -256
        int BLUE = Color.BLUE;      // -16776961

        int[] color_ = {RED, GREEN, YELLOW, BLUE};
        int[] strokeWidth_ = {strokeWidth, strokeWidth, strokeWidth, strokeWidth};

        List<double[]> list_startX = new ArrayList<>(cornersList.size());
        List<double[]> list_startY = new ArrayList<>(cornersList.size());
        List<double[]> list_stopX = new ArrayList<>(cornersList.size());
        List<double[]> list_stopY = new ArrayList<>(cornersList.size());

        mLineSurface.clear();

        List<Double> centerX = new ArrayList<>(cornersList.size());
        List<Double> centerY = new ArrayList<>(cornersList.size());
        for (List<VpImagePoint> corners : cornersList) {
            double[] startX_ = {corners.get(0).get_u(), corners.get(0).get_u(), corners.get(1).get_u(), corners.get(2).get_u()};
            double[] startY_ = {corners.get(0).get_v(), corners.get(0).get_v(), corners.get(1).get_v(), corners.get(2).get_v()};
            double[] stopX_ = {corners.get(1).get_u(), corners.get(3).get_u(), corners.get(2).get_u(), corners.get(3).get_u()};
            double[] stopY_ = {corners.get(1).get_v(), corners.get(3).get_v(), corners.get(2).get_v(), corners.get(3).get_v()};

            list_startX.add(startX_);
            list_startY.add(startY_);
            list_stopX.add(stopX_);
            list_stopY.add(stopY_);

            centerX.add( (corners.get(0).get_u() + corners.get(1).get_u() + corners.get(2).get_u() + corners.get(3).get_u()) / 4 );
            centerY.add( (corners.get(0).get_v() + corners.get(1).get_v() + corners.get(2).get_v() + corners.get(3).get_v()) / 4 );
        }

        List<Double> tagDistances = new ArrayList<>(cMoList.size());
        List<double[]> list_oX = new ArrayList<>(cornersList.size());
        List<double[]> list_oY = new ArrayList<>(cornersList.size());
        for (VpHomogeneousMatrix cMo : cMoList) {
            double[] cMo_array = new double[16];

            String[] cMo_array_str = cMo.toString().split("\\s+");
            cMo_array = Arrays.stream(cMo_array_str)
                    .mapToDouble(Double::parseDouble)
                    .toArray();

            double tx = cMo_array[3];
            double ty = cMo_array[7];
            double tz = cMo_array[11];

            double dist = Math.sqrt(tx*tx + ty*ty + tz*tz);
            tagDistances.add(dist);

            // Tag frame
            VpPoint origin = new VpPoint(0, 0, 0);
            VpImagePoint im_origin = project(cMo, origin, cam);

            VpPoint oX = new VpPoint(tagSize * tagFrameRatio, 0, 0);
            VpImagePoint im_ox = project(cMo, oX, cam);

            VpPoint oY = new VpPoint(0, tagSize * tagFrameRatio, 0);
            VpImagePoint im_oy = project(cMo, oY, cam);

            VpPoint oZ = new VpPoint(0, 0, tagSize * tagFrameRatio);
            VpImagePoint im_oz = project(cMo, oZ, cam);

            list_oX.add(new double[] {im_origin.get_u(), im_ox.get_u(), im_oy.get_u(), im_oz.get_u()} );
            list_oY.add(new double[] {im_origin.get_v(), im_ox.get_v(), im_oy.get_v(), im_oz.get_v()} );
        }

        mLineSurface.drawLines(list_startX, list_startY, list_stopX, list_stopY, color_, strokeWidth_, centerX, centerY, ids,
                tagDistances.stream().mapToDouble(Double::valueOf).toArray(), orientation, w, h, displayTagFrame, list_oX, list_oY);

        mResultInfo.setText(s);
    }

    private static VpImagePoint project(VpHomogeneousMatrix cMo, VpPoint obj, VpCameraParameters cam) {
        obj.changeFrame(cMo);
        obj.projection();
        double px = cam.get_px(), py = cam.get_py(), u0 = cam.get_u0(), v0 = cam.get_v0();
        double u = px * obj.get_x() + u0;
        double v = py * obj.get_y() + v0;

        return new VpImagePoint(v, u);
    }

    @Override
    public void onPause() {
        super.onPause();
        // Stop camera access
        releaseCamera();
    }

    @Override
    public void onResume() {
        super.onResume();
        if (mCamera == null) {
            try {
                mCamera = getCameraInstance(CAMERA_ID);
            } catch (Exception e) {
                Log.e(TAG, "CameraPreviewActivity::onResume() ; Error opening camera: " + e.getMessage());
            }
        }

        if (mPreview == null) {
            init();
        }
    }

    /** A safe way to get an instance of the Camera object. */
    private Camera getCameraInstance(int cameraId) {
        Camera c = null;
        try {
            c = Camera.open(cameraId); // attempt to get a Camera instance
        } catch (Exception e) {
            Log.e(TAG, "CameraPreviewActivity::getCameraInstance() ; Camera " + cameraId + " is not available: " + e.getMessage());
            // Camera is not available (in use or does not exist)
            Toast.makeText(this, "Camera " + cameraId + " is not available: " + e.getMessage(),
                    Toast.LENGTH_SHORT).show();
        }
        return c; // returns null if camera is unavailable
    }

    private void releaseCamera() {
        if (mCamera != null) {
            mCamera.stopPreview();
            mCamera.release();
            mCamera = null;
        }
    }
}
