# ROS2 driver for µ-Epsilon scanCONTROL sensors

Before building, download the SDK by running the `get_sdk.sh` script at the repo
root (see the top-level README).

## Running

```bash
ros2 launch scancontrol_lls_driver lls_driver.launch.py
# override the parameter file:
ros2 launch scancontrol_lls_driver lls_driver.launch.py \
  params_file:=/path/to/your.params.yaml
```

The default parameters live in [`config/lls_driver.params.yaml`](config/lls_driver.params.yaml).

## Behaviour

On startup the node declares its parameters and creates its services and the
PointCloud2 data publisher. If `connect_on_startup` is `true` (the default) it
also tries to connect to the first available interface; a failure there is
**non-fatal** - the node stays up and you can connect later via the
`set_interface` service.

- If a single scanner is connected, just call the `start_scan` / `stop_scan`
  services (`std_srvs/srv/Trigger`) to start and stop a scan.
- To pick a specific scanner, use the `discovery` service to list interfaces and
  the `set_interface` service to select one. `set_interface` re-initialises the
  scanner/SDK and is rejected while a scan is running.
- Profiles are published as `sensor_msgs/msg/PointCloud2` using `SensorDataQoS`.
- All topic and service names, and the sensor configuration, can be overridden
  via node parameters (see the config file).

## Features / ToDo's
- [x] configure scanner from parameter *.yaml file
- [x] publish scan points on a configurable topic
- [x] provide start_scan / stop_scan services
- [x] provide an interface query (discovery) service
- [x] provide an interface selection service
- [x] migrate service / message interface definitions to a separate package
- [x] ship a launch file and installed default config
