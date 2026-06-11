{
  inputs = {
    # nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
  };
  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "Example project";
          packages = [
            pkgs.colcon
            pkgs.opencv

            #c++
            pkgs.gcc
            pkgs.cmake
            pkgs.pcl
            pkgs.clang-tools
            #rust
            # pkgs.cargo
            # pkgs.rustc
            # pkgs.rust-analyzer
            (
              with pkgs.rosPackages.jazzy;
              buildEnv {
                # underlay = true;
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
                ];
              }
            )
          ];
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
