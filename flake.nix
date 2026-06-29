{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    flake-utils.follows = "nix-ros-overlay/flake-utils";
  };

  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros-packages =
          with pkgs.rosPackages.jazzy;
          buildEnv {
            paths = [
              ros-core
              rclcpp
              rclpy
              sensor-msgs
              geometry-msgs
              nav-msgs
              ament-cmake
              ament-cmake-core
              cv-bridge
              example-interfaces
              aruco

              teleop-twist-keyboard
            ];
          };

      in
      {
        devShells.default = pkgs.mkShell {
          name = "Example project";

          nativeBuildInputs = [
            pkgs.cmake
            pkgs.clang-tools
            pkgs.colcon
            pkgs.libcamera
            pkgs.gst_all_1.gstreamer
            pkgs.gst_all_1.gst-plugins-base
            pkgs.gst_all_1.gst-plugins-good
            pkgs.gst_all_1.gst-plugins-bad
            pkgs.gst_all_1.gst-plugins-ugly
            pkgs.gst_all_1.gst-libav
            pkgs.gst_all_1.gst-plugins-rs
            pkgs.libcamera
          ];

          buildInputs = [
            pkgs.libgpiod
            pkgs.lgpio
            pkgs.opencv
            pkgs.pcl
            pkgs.libserialport
            ros-packages
          ];

          shellHook = ''
            export LD_LIBRARY_PATH=${pkgs.lgpio}/lib:$LD_LIBRARY_PATH
            export LIBRARY_PATH=${pkgs.lgpio}/lib:$LIBRARY_PATH
            export CPATH=${pkgs.lgpio}/include:$CPATH
          '';
        };
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
