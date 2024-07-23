# Scan 'N Plan Workshop

Framework for developing and operating simple Scan 'N Plan applications in which a robot:
- Executes a scan path with a depth camera
- Reconstructs the surface of the objects observed by the camera
- Plans a tool path on the reconstructed surface
- Plans robot motions to execute the tool path
- Executes the process path

## Build Setup

1. Install the source dependencies after cloning this repository into a `colcon` workspace
    ```bash
    cd <snp_workspace>
    vcs import --shallow --debug src < src/scan_n_plan_workshop/dependencies_tesseract.repos
    vcs import --shallow --debug src < src/scan_n_plan_workshop/dependencies.repos
    # Source the ROS distro before running rosdep
    source /opt/ros/<distro>/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

    > Note: some `rosdep` dependencies specified by `tesseract` may be not resolved, but it is generally okay to ignore these if building with the command specified below.

## Build
Build the repository with the following command (including a few CMake arguments to ignore modules of `tesseract` that are not needed)

```bash
colcon build --cmake-args -DTESSERACT_BUILD_FCL=OFF -DBUILD_RENDERING=OFF
```

## Application-specific Implementations
This repository provides a framework for operating a Scan 'N Plan application.
Several complete Scan 'N Plan applications based on this repository can be found in the following repositories:

- [Automate 2022](https://github.com/ros-industrial-consortium/snp_automate_2022)
- [Automate 2023](https://github.com/ros-industrial-consortium/snp_automate_2023)
- [Robotic Blending Milestone 5](https://github.com/ros-industrial-consortium/snp_blending)
