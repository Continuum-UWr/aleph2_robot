{ buildRosPackage, ament-cmake-ros, lely-core-libraries, aleph2-description
, canopen-core, canopen-master-driver, diff-drive-controller
, joint-state-broadcaster, nanotec-driver, xacro, ament-lint-auto
, ament-cmake-lint-cmake, ament-cmake-xmllint }:
buildRosPackage {
  pname = "aleph2-bringup";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake-ros lely-core-libraries ];
  propagatedBuildInputs = [
    aleph2-description
    canopen-core
    canopen-master-driver
    diff-drive-controller
    joint-state-broadcaster
    nanotec-driver
    xacro
  ];
  checkInputs = [ ament-lint-auto ament-cmake-lint-cmake ament-cmake-xmllint ];
  nativeBuildInputs = [ ament-cmake-ros ];
}
