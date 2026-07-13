# Stereo calibration (F1)

Real intrinsics/extrinsics for the ELP H120 `left_wide` / `right_wide` streams. The
GStreamer camera node ships a *fabricated* CameraInfo (fx = width); the metric nav
pipeline uses the calibrated YAMLs here instead, published by `stereo_camera_info_pub`.

## Important

- **Calibrate at the exact resolution nav mode runs.** For nav we recommend running the
  wide streams at **640×480** (`output_width:=640 output_height:=480` on the camera
  node). If you change that resolution, **recalibrate**.
- The two eyes are **swapped** in the sensor frame (left eye = right half of the raw
  image); the camera node already de-swaps them into `left_wide` / `right_wide`, so
  calibrate the published topics, not the raw device.
- These are downsampled full-FOV streams — fine for room-scale stereo; ESS resizes
  internally. JPEG artifacts are assessed at the "USB-stereo quality gate".

## Procedure

1. Decode the wide streams to raw (calibration tools want `sensor_msgs/Image`):

   ```bash
   ros2 run image_transport republish compressed raw \
     --ros-args -r in/compressed:=/alfie/stereo_camera/left_wide/image_raw/compressed \
                -r out:=/alfie/stereo_camera/left_wide/image_raw
   # ...and the same for right_wide
   ```

2. Run the stereo calibrator with your checkerboard (adjust `--size` / `--square`):

   ```bash
   ros2 run camera_calibration cameracalibrator \
     --size 8x6 --square 0.025 --approximate 0.1 \
     --ros-args \
       -r left:=/alfie/stereo_camera/left_wide/image_raw \
       -r right:=/alfie/stereo_camera/right_wide/image_raw \
       -r left_camera:=/alfie/stereo_camera/left \
       -r right_camera:=/alfie/stereo_camera/right
   ```

3. Move the board through the frame (near/far, tilts, corners) until X/Y/Size/Skew bars
   fill, then **Calibrate** → **Save**. Extract `left.yaml` and `right.yaml` from the
   tarball and drop them here.

4. Verify: stereo RMS < ~0.5 px; the calibrated `right.yaml` `projection_matrix` P[0][3]
   encodes the baseline (`Tx = -fx * baseline`). Point `stereo_camera_info_pub` at these
   files (defaults: `left.yaml` / `right.yaml` in this directory).
