# Validation status

Base: ros branch, 2e2e053c4c00561e78ef67a5b24c46204c5f4dae.

- Local Windows / Python 3.12.14: 27 unit/contract tests passed; one real ROS integration test skipped because rclpy is unavailable.
- Python sources parsed using the Python 3.8 grammar; package XML parsed successfully.
- Unit coverage includes 1000 incoming commands/s, slew/angle limits, heartbeat, stale input, invalid values, manual/auto switching, lost camera/PC, orange finish latch, and two simulated laps.
- Detector contract tests use mocked imports and real NumPy; they do NOT execute TensorRT/CUDA.
- ROS 2 Foxy CI is configured for Ubuntu 20.04 / Python 3.8 on AMD64 and ARM64, including colcon build/test, real UDP/rclpy integration and installed launch/entry points.
- Reference Arduino Uno compile is configured in CI; the actual board model is not verified.
- CI outcome is recorded on the GitHub pull request; configuration alone is not a passing build.
- No Jetson, ZED, trained cone engine or physical actuator is attached to this development host. GPU execution, hardware operation, steering calibration and real-time performance remain unverified until the on-device checklist in README.md is completed.
