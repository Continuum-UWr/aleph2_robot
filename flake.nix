{
  inputs = {
    nixpkgs.follows = "aleph2_common/nixpkgs";
    flake-utils.follows = "aleph2_common/flake-utils";
    nix-ros-overlay.follows = "aleph2_common/nix-ros-overlay";
    aleph2_common.url =
      "git+https://gitlab.continuum.ii.uni.wroc.pl/continuum/software/aleph2_common";
  };
  outputs = { self, nixpkgs, flake-utils, nix-ros-overlay, aleph2_common }:
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

        aleph2_bringup = ros.callPackage (import ./aleph2_bringup) { };

      in {
        packages = {
          inherit aleph2_bringup;
          default = aleph2_bringup;
        };
        devShells.default = pkgs.mkShell {
          inputsFrom = [ aleph2_bringup ];
          packages = [ ros.ros-core aleph2_bringup ];
        };
        formatter = pkgs.nixfmt-classic;
      });
}

