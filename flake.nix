{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros-overlay = {
      url =
        "git+https://gitlab.continuum.ii.uni.wroc.pl/continuum/software/nix-ros-overlay?ref=continuum";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    aleph2_common.url =
      "git+https://gitlab.continuum.ii.uni.wroc.pl/continuum/software/aleph2_common";
  };
  outputs = { self, nixpkgs, flake-utils, nix-ros-overlay }:
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
        # nanotec_driver = ros.callPackage (import ./nanotec_driver) { };

      in {
        packages = {
          # inherit aleph2_bringup nanotec_driver;
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

