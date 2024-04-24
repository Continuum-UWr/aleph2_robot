{ buildRosPackage, ament-cmake, boost, ament-index-cpp, canopen-core
, canopen-interfaces, canopen-proxy-driver, controller-manager
, hardware-interface, lely-core-libraries, pluginlib, rclcpp-components
, sensor-msgs, robot-state-publisher, joint-state-broadcaster
, forward-command-controller, ament-lint-auto, ament-cmake-cpplint
, ament-cmake-lint-cmake, ament-cmake-uncrustify }:
buildRosPackage {
  pname = "nanotec_driver";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    boost
    ament-index-cpp
    canopen-core
    canopen-interfaces
    canopen-proxy-driver
    controller-manager
    hardware-interface
    lely-core-libraries
    pluginlib
    rclcpp-components
    sensor-msgs
    robot-state-publisher
    joint-state-broadcaster
    forward-command-controller
  ];
  checkInputs = [
    ament-lint-auto
    ament-cmake-cpplint
    ament-cmake-lint-cmake
    ament-cmake-uncrustify
  ];
  nativeBuildInputs = [ ament-cmake ];
}
