package com.example.apriltagdetection;

import android.content.Intent;
import android.os.Bundle;
import android.text.method.LinkMovementMethod;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import java.util.Map;

public class SettingsPanelActivity extends AppCompatActivity {
    private static final String TAG = "SettingsPanelActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        TextView originalCreditDesc = findViewById(R.id.originalCreditDesc);
        originalCreditDesc.setMovementMethod(LinkMovementMethod.getInstance());

        TextView apriltagDesc = findViewById(R.id.apriltagDesc);
        apriltagDesc.setMovementMethod(LinkMovementMethod.getInstance());

        TextView onlineMarkers = findViewById(R.id.onlineMarkers);
        onlineMarkers.setMovementMethod(LinkMovementMethod.getInstance());

        // Update values
        Intent intent = getIntent();

        // Flash
        CheckBox cameraFlashInput = findViewById(R.id.cameraFlash);
        boolean camera_flash = intent.getBooleanExtra("camera_flash", false);
        cameraFlashInput.setChecked(camera_flash);

        // Camera specs
        float camera_focal_mm = intent.getFloatExtra("camera_focal_mm", 0);
        TextView camera_focal_mm_value = findViewById(R.id.camera_lens_focal_mm_value);
        camera_focal_mm_value.setText(camera_focal_mm + " mm");

        int camera_native_resolution_w = intent.getIntExtra("camera_native_w", 0);
        int camera_native_resolution_h = intent.getIntExtra("camera_native_h", 0);
        TextView camera_native_resolution_value = findViewById(R.id.camera_native_resolution_value);
        camera_native_resolution_value.setText(camera_native_resolution_w + " X " + camera_native_resolution_h);

        float camera_hfov = intent.getFloatExtra("camera_hfov", 0);
        float camera_wfov = intent.getFloatExtra("camera_wfov", 0);
        TextView camera_fov_value = findViewById(R.id.camera_fov_value);
        camera_fov_value.setText(camera_hfov + "° (hfov) " + camera_wfov + "° (vfov)");

        double camera_sensor_w = intent.getDoubleExtra("camera_sensor_w", 0);
        double camera_sensor_h = intent.getDoubleExtra("camera_sensor_h", 0);
        TextView camera_sensor_size_value = findViewById(R.id.camera_sensor_size_value);
        camera_sensor_size_value.setText(String.format("%.3f", camera_sensor_w) + " mm X " + String.format("%.3f", camera_sensor_h) + " mm");

        int image_w = intent.getIntExtra("image_w", 0);
        int image_h = intent.getIntExtra("image_h", 0);
        TextView image_resolution_value = findViewById(R.id.image_resolution_value);
        image_resolution_value.setText(image_w + " X " + image_h);

        EditText cameraFocalInput = findViewById(R.id.cameraFocalInput);
        double focalLength = intent.getDoubleExtra("focal", 0);
        cameraFocalInput.setText(String.valueOf(focalLength));

        EditText tagSizeInput = findViewById(R.id.tagSizeInput);
        double tagSize = intent.getDoubleExtra("tag_size", 0);
        tagSizeInput.setText(String.valueOf(tagSize));

        CheckBox displayFrameInput = findViewById(R.id.displayMarkerFrame);
        boolean display_frame = intent.getBooleanExtra("display_tag_frame", false);
        displayFrameInput.setChecked(display_frame);

        EditText frameSizeRatioInput = findViewById(R.id.markerFrameSizeRatio);
        double frame_size_ratio = intent.getDoubleExtra("display_tag_frame_ratio", 0.5);
        frameSizeRatioInput.setText(String.valueOf(frame_size_ratio));

        // AprilTag

        // selection position
        int apriltag_selection = intent.getIntExtra("aprilTag_selection_position", 0);

        // quad_decimate
        Spinner quad_decimate_spinner = findViewById(R.id.aprilTagQuadDecimate);
        Integer[] quad_decimate_options = {1, 2, 3, 4, 6, 8};
        ArrayAdapter<Integer> quad_decimate_adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, quad_decimate_options);
        quad_decimate_adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        quad_decimate_spinner.setAdapter(quad_decimate_adapter);
        Map<Integer, Integer> quad_decimate_options_map = Map.of(
                1, 0,
                2, 1,
                3, 2,
                4, 3,
                6, 4,
                8, 5
        );
        quad_decimate_spinner.setSelection(quad_decimate_options_map.get(intent.getIntExtra("aprilTag_quad_decimate", 1)));

        // decision_margin
        SeekBar decision_margin_seekBar = findViewById(R.id.aprilTagDecisionMargin);
        TextView decision_margin_value = findViewById(R.id.aprilTagDecisionMarginValue);
        decision_margin_seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                decision_margin_value.setText(String.valueOf(progress));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                Toast.makeText(SettingsPanelActivity.this, "Margin value: " + decision_margin_seekBar.getProgress(), Toast.LENGTH_SHORT).show();
            }
        });
        decision_margin_seekBar.setProgress(intent.getIntExtra("aprilTag_decision_margin", 50));


        // nb threads
        Spinner nb_threads_spinner = findViewById(R.id.aprilTagNbThreads);
        Integer[] nb_threads_options = {1, 2, 3, 4};
        ArrayAdapter<Integer> nb_threads_adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, nb_threads_options);
        nb_threads_adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        nb_threads_spinner.setAdapter(nb_threads_adapter);
        nb_threads_spinner.setSelection(intent.getIntExtra("aprilTag_nb_threads", 1) - 1);

        Button validate = findViewById(R.id.btnValidateSettings);
        validate.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String cameraFocalInput_str = cameraFocalInput.getText().toString();
                String tagSizeInput_str = tagSizeInput.getText().toString();
                String tagFrameRatioInput_str = frameSizeRatioInput.getText().toString();
                String aprilTagSelectionPosition_str = String.valueOf(apriltag_selection);
                String aprilTagQuadDecimate_str = quad_decimate_spinner.getSelectedItem().toString();
                String aprilTagMargin_str = String.valueOf(decision_margin_seekBar.getProgress());
                String aprilTagNbThreads_str = nb_threads_spinner.getSelectedItem().toString();

                Intent resultIntent = new Intent();
                resultIntent.putExtra("cameraFlashChecked", String.valueOf(cameraFlashInput.isChecked()));
                resultIntent.putExtra("cameraFocalValue", cameraFocalInput_str);
                resultIntent.putExtra("tagSizeValue", tagSizeInput_str);
                resultIntent.putExtra("displayFrameChecked", String.valueOf(displayFrameInput.isChecked()));
                resultIntent.putExtra("tagFrameRatio", tagFrameRatioInput_str);
                resultIntent.putExtra("aprilTagSelectionPosition", aprilTagSelectionPosition_str);
                resultIntent.putExtra("aprilTagQuadDecimate", aprilTagQuadDecimate_str);
                resultIntent.putExtra("aprilTagDecisionMargin", aprilTagMargin_str);
                resultIntent.putExtra("aprilTagNbThreads", aprilTagNbThreads_str);
                setResult(RESULT_OK, resultIntent);

                finish();
            }
        });
    }
}
