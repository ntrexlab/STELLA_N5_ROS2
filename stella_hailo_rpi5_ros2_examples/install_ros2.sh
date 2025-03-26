#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Install additional system dependencies (if needed)
echo "Installing additional system dependencies..."
sudo apt install -y rapidjson-dev

# Initialize variables
DOWNLOAD_RESOURCES_FLAG=""
PYHAILORT_WHL=""
PYTAPPAS_WHL=""
INSTALL_TEST_REQUIREMENTS=false

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --pyhailort) PYHAILORT_WHL="$2"; shift ;;
        --pytappas) PYTAPPAS_WHL="$2"; shift ;;
        --test) INSTALL_TEST_REQUIREMENTS=true ;;
        --all) DOWNLOAD_RESOURCES_FLAG="--all" ;;
        --tag) TAG="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Install specified Python wheels
if [[ -n "$PYHAILORT_WHL" ]]; then
    echo "Installing pyhailort wheel: $PYHAILORT_WHL"
    sudo python3 -m pip install "$PYHAILORT_WHL"
fi

if [[ -n "$PYTAPPAS_WHL" ]]; then
    echo "Installing pytappas wheel: $PYTAPPAS_WHL"
    sudo python3 -m pip install "$PYTAPPAS_WHL"
fi

# Install the required Python dependencies
echo "Installing required Python dependencies..."
sudo python3 -m pip install -r requirements.txt

# Install Hailo Apps Infrastructure from specified tag/branch
echo "Installing Hailo Apps Infrastructure from version: $TAG..."
pip install "git+https://github.com/hailo-ai/hailo-apps-infra.git"

# Install test requirements if needed
if [[ "$INSTALL_TEST_REQUIREMENTS" == true ]]; then
    echo "Installing test requirements..."
    sudo python3 -m pip install -r tests/test_resources/requirements.txt
fi

# Download resources needed for the pipelines
echo "Downloading resources needed for the pipelines..."
./download_resources.sh $DOWNLOAD_RESOURCES_FLAG

echo "Installation completed successfully."
