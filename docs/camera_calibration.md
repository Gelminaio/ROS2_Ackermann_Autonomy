# Camera calibration

IMX708 @ 640x480. Checkerboard 7x9 corners, 20mm.

Driver: `ros2 launch ackermann_bringup camera.launch.py`
Calibrate: `ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.02 --ros-args -r image:=/camera/image_raw -p camera:=/camera -p image_transport:=raw`

YAML in `config/camera_calibration.yaml`, loaded via `camera_info_url`.
