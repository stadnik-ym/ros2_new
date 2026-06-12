{
  description = "Optimized Docker image for C++ ROS 2 nodes workspace";

  inputs = {
    main-project.url = "path:../";
  };

  outputs =
    { self, main-project }:
    main-project.inputs.nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import main-project.inputs.nixpkgs {
          inherit system;
          overlays = [ main-project.inputs.nix-ros-overlay.overlays.default ];
        };

        rosEnv = pkgs.rosPackages.jazzy;

        # Фильтруем исходники, чтобы не тащить локальный мусор в Nix-память
        srcFilter =
          path: type:
          let
            baseName = baseNameOf path;
          in
          !(
            type == "directory"
            && (
              baseName == "build"
              || baseName == "install"
              || baseName == "log"
              || baseName == ".cache"
              || baseName == ".git"
            )
          );
        cleanSrc = builtins.filterSource srcFilter ../.;

        # --- Сборка нашего C++ ворксейса ---
        myRosWorkspace = pkgs.stdenv.mkDerivation {
          pname = "ros2-cpp-workspace";
          version = "0.1.0";
          src = cleanSrc;

          # Зависимости для этапа сборки
          nativeBuildInputs = [
            pkgs.cmake
            pkgs.colcon
            rosEnv.ament-cmake
            rosEnv.ament-cmake-core
          ];

          # Зависимости для работы нод
          buildInputs = [
            pkgs.opencv
            # pkgs.pcl # <-- УБЕРИ эту строку, если PCL не нужен ноде! Он весит ~600+ МБ с зависимостями.
            rosEnv.ros-core
            rosEnv.rclcpp
            rosEnv.sensor-msgs
            rosEnv.geometry-msgs
            rosEnv.nav-msgs
            rosEnv.cv-bridge
            rosEnv.example-interfaces
            rosEnv.aruco
          ];

          dontConfigure = true;

          buildPhase = ''
            export HOME=$TMPdir
            colcon build \
              --merge-install \
              --install-base $out \
              --cmake-args -DCMAKE_BUILD_TYPE=Release
          '';

          # Обрываем текстовые ссылки на nativeBuildInputs
          postInstall = ''
            echo "Removing Colcon environment setups to reduce closure size..."
            rm -f $out/_local_setup.lambda
            rm -f $out/local_setup.*
            rm -f $out/setup.*
            rm -f $out/_local_setup.*
          '';
        };

      in
      {
        # --- Сборка легковесного Docker-образа ---
        packages.dockerImage = pkgs.dockerTools.buildLayeredImage {
          name = "ros2-cpp-app";
          tag = "latest";

          contents = [
            myRosWorkspace
            rosEnv.ros-core
            rosEnv.rmw-fastrtps-cpp # Исправлена опечатка (было fastrps)
          ];

          config = {
            # Запускаем бинарник напрямую, раз уж удалили скрипты colcon
            Cmd = [ "${myRosWorkspace}/lib/aruco_detector/aruco_detector_node" ];

            Env = [
              "AMENT_PREFIX_PATH=${myRosWorkspace}:${rosEnv.ros-core}:${rosEnv.rmw-fastrtps-cpp}"
              "LD_LIBRARY_PATH=${myRosWorkspace}/lib"
              "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
              "PYTHONPATH=${myRosWorkspace}/lib/python3.11/site-packages"
              "ROS_DISTRO=jazzy"
              "PYTHONUNBUFFERED=1"
            ];
          };
        };
      }
    );
}
