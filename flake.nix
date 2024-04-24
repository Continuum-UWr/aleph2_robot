{
  inputs = {
    nixpkgs.follows = "aleph2-common/nixpkgs";
    flake-utils.follows = "aleph2-common/flake-utils";
    nix-ros-overlay.follows = "aleph2-common/nix-ros-overlay";
    aleph2-common.url =
      "git+https://gitlab.continuum.ii.uni.wroc.pl/continuum/software/aleph2_common?ref=nix-update";
  };
  outputs = { self, nixpkgs, flake-utils, nix-ros-overlay, aleph2-common }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = (import nixpkgs {
          system = system;
          overlays = [ ];
        }).pkgs;
        ros = (import nixpkgs {
          system = system;
          overlays = [ nix-ros-overlay.overlays.default ];
        }).pkgs.rosPackages.rolling;

        aleph2-description =
          aleph2-common.packages.${system}.aleph2-description;

        nanotec-driver = ros.callPackage (import ./nanotec_driver) { };
        aleph2-bringup = ros.callPackage (import ./aleph2_bringup) {
          inherit aleph2-description nanotec-driver;
        };

      in {
        packages = {
          inherit aleph2-bringup nanotec-driver;
          default = aleph2-bringup;
        };
        devShells.default = pkgs.mkShell {
          inputsFrom = [ aleph2-bringup ];
          packages = [ ros.ros-core aleph2-bringup ];
        };
        formatter = pkgs.nixfmt-classic;
      });
}

