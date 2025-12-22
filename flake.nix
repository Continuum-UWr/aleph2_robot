{
  inputs = {
    nixpkgs.follows = "aleph2-common/nixpkgs";
    flake-utils.follows = "aleph2-common/flake-utils";
    nix-ros-overlay.follows = "aleph2-common/nix-ros-overlay";
    aleph2-common.url = "git+https://gitlab.continuum.ii.uni.wroc.pl/continuum/software/aleph2_common";
  };
  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      nix-ros-overlay,
      aleph2-common,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs =
          (import nixpkgs {
            system = system;
            overlays = [ nix-ros-overlay.overlays.default ];
          }).pkgs;
        ros = pkgs.rosPackages.jazzy;
        aleph2-common-packages = aleph2-common.packages.${system};

        aleph2-description = aleph2-common.packages.${system}.aleph2-description;

        nanotec-driver = ros.callPackage (import ./nanotec_driver) { };
        aleph2-bringup = ros.callPackage (import ./aleph2_bringup) {
          inherit aleph2-description nanotec-driver;
        };
        devEnv = ros.buildEnv {
          paths = [
            aleph2-bringup
            nanotec-driver
          ];
        };

      in
      {
        packages = {
          inherit aleph2-bringup nanotec-driver devEnv;
          default = aleph2-bringup;
        };

        devShells.default = pkgs.mkShell {
          nativeBuildInputs = [
            aleph2-common-packages.devEnv
            devEnv
          ];
        };
        formatter = pkgs.nixfmt;
      }
    );
  nixConfig = {
    extra-substituters = [ "https://nix-continuum.s3-web.uwukado.me" ];
    extra-trusted-public-keys = [ "nix-continuum:rc3o+NH47H8tmR2RW1fE6NWfRtzNP20ClkTnVgCOteA=" ];
  };

}
