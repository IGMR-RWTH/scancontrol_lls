# ROS2 driver for µ-Epsilon ScanControl Sensors

SCANCONTROL_LLS_driver is a ROS2 node designed to interface with laser line scanners (LLS) manufactured by micro-epsilon GmbH.

To avoid having to use meson to build the sensor SDK, a cmake file is provided.

A dockerfile for a build container is provided.

## Building

The vendored SDK is **not** committed (`ext/` is git-ignored). Fetch it first,
then build:

```bash
./get_sdk.sh                 # downloads + patches the scanCONTROL SDK into ext/
colcon build                 # build the workspace
```

`get_sdk.sh` must be run before `colcon build` (and before building the Docker
image) - otherwise `find_package(libllt)` / `find_package(libmescan)` will fail.

## Packages

- **scancontrol_lls_driver:** The package containing the `lls_driver` node
- **scancontrol_lls_msgs:** The package containing the message definitions to talk to the driver node