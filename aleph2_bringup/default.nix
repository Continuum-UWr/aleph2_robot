{ buildRosPackage, ament-cmake-ros, lely-core-libraries, aleph2-description
, nanotec-driver, xacro, ament-lint-auto, ament-cmake-lint-cmake
, ament-cmake-xmllint }:
buildRosPackage {
  pname = "aleph2-bringup";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake-ros lely-core-libraries ];
  propagatedBuildInputs = [ xacro aleph2-description nanotec-driver ];
  checkInputs = [ ament-lint-auto ament-cmake-lint-cmake ament-cmake-xmllint ];
  nativeBuildInputs = [ ament-cmake-ros ];
}
