#!/bin/bash

# build_target.sh
# This script builds a specific CMake target within the project.

# Check if a target name was provided
if [ -z "$1" ]; then
    echo "Usage: $0 <target_name_or_path> [build_configuration]"
    echo "Example: $0 utils Release"
    echo "Example: $0 src/utils/utils Release (also supported)"
    exit 1
fi

TARGET_NAME_INPUT="$1"
BUILD_CONFIG="${2:-Release}" # Default to 'Debug' if no config is provided

# Extract the actual target name (e.g., 'utils' from 'src/utils/utils')
# basename command handles both 'utils' and 'src/utils/utils' correctly.
TARGET_NAME=$(basename "${TARGET_NAME_INPUT}")

# Determine the project root directory
# This command finds the root of your Git repository, which is usually your project root.
# If your project is NOT a Git repo, you might need to adjust this.
PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd)

# Define the build directory path relative to the project root
BUILD_DIR="${PROJECT_ROOT}/build"

echo "--- CMake Build Script ---"
echo "Project Root: ${PROJECT_ROOT}"
echo "Build Directory: ${BUILD_DIR}"
echo "Original Target Input: ${TARGET_NAME_INPUT}"
echo "Parsed Target Name: ${TARGET_NAME}"
echo "Configuration: ${BUILD_CONFIG}"
echo "--------------------------"

# This is the command that CMake extension in VS Code runs:
# "C:\Program Files\CMake\bin\cmake.EXE" -DChrono_DIR:STRING=c:/Users/15309/Project_Chrono/chrono_build/cmake -Dpybind11_DIR:STRING=C://Users//15309//AppData//Local//Programs//Python//Python39//lib//site-packages//pybind11//share//cmake//pybind11 -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE --no-warn-unused-cli -S C:/Users/15309/Project_Chrono/chrono_simulink_cosim -B c:/Users/15309/Project_Chrono/chrono_simulink_cosim/build -G "Visual Studio 16 2019"

# Ensure the build directory exists and configure if needed
if [ ! -d "${BUILD_DIR}" ]; then
    echo "Build directory '${BUILD_DIR}' does not exist."
    echo "Attempting to create and configure the project..."
    mkdir -p "${BUILD_DIR}" # -p creates parent directories if they don't exist
    
    # Navigate to build directory, configure, then go back
    (cd "${BUILD_DIR}" && cmake ..)
    
    if [ $? -ne 0 ]; then
        echo "Error: CMake configuration failed. Exiting."
        exit 1
    fi
    echo "CMake configuration successful."
fi

# Navigate to the build directory and build the specified target
echo "Navigating to ${BUILD_DIR} and building target ${TARGET_NAME}..."
(cd "${BUILD_DIR}" && cmake --build . --target "${TARGET_NAME}" --config "${BUILD_CONFIG}")

# Check the exit status of the cmake --build command
if [ $? -eq 0 ]; then
    echo "Successfully built target: ${TARGET_NAME} (${BUILD_CONFIG})"
    echo "Output should be in ${BUILD_DIR}/bin or ${BUILD_DIR}/lib (or specific config subfolder within these)"
else
    echo "Error: Failed to build target: ${TARGET_NAME} (${BUILD_CONFIG})"
    exit 1
fi
