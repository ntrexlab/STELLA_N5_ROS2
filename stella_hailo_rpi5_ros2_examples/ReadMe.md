# stella_hailo_rpi5_ros2_examples

## Overview
This package is developed based on the [hailo-rpi5-example](https://github.com/hailo-ai/hailo-rpi5-examples). It is designed for ROS2 Jazzy and works by receiving the `image_raw` topic as input and publishing the `detection_image` topic as output.

## Installation
1. **Clone the Repository**  
    - Download the `hailo-rpi5-example` repository from Git.

2. **Move the Installation Script**  
    - Copy the `install_ros2.sh` script from this package into the root directory of the `hailo-rpi5-example` repository.

    Example commands:
    ```bash
    cp install_ros2.sh /path_of_your_git_clone_dir/hailo-rpi5-example/
    ```

3. **Run the Installation Script**  
    - Execute the `install_ros2.sh` script **without** using a virtual environment.
    - This script is a modified version of the original `install.sh` provided in hailo-rpi5-example, adjusted to install the required libraries globally rather than within a virtual environment.

    Example commands:    
    ```bash
    cd /path_of_your_git_clone_dir/hailo-rpi5-example
    ./install_ros2.sh
    ```

4. **Update Your .bashrc**
    - At the end of the installation process, add the following line to your .bashrc to export the required environment variable:
    ```bash
    export TAPPAS_POST_PROC_DIR="/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes"
    ```

## Additional Information
- The globally installed version of numpy must match the version required by hailo-rpi5-example.
- This package only supports the example models provided by hailo-rpi5-example.
- For developing instance segmentation or pose estimation models, by referring to both hailo-rpi5-example and this package, you can develop them.

## License
This package is released under the MIT License, following the license of hailo-rpi5-example.
