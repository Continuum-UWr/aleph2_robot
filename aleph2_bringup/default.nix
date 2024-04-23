{ buildRosPackage, ament-cmake, xacro, ament-lint-auto, ament-cmake-lint-cmake
, ament-cmake-xmllint }:
buildRosPackage {
  pname = "aleph2_bringup";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ]; # lely-core-libraries
  propagatedBuildInputs = [ xacro ]; # aleph2_description nanotec_driver
  checkInputs = [ ament-lint-auto ament-cmake-lint-cmake ament-cmake-xmllint ];
  nativeBuildInputs = [ ament-cmake ];
}
