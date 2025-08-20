# cmake -DChrono_DIR:STRING=c:/Users/15309/Project_Chrono/chrono_build/cmake -Dpybind11_DIR:STRING=C://Users//15309//AppData//Local//Programs//Python//Python39//lib//site-packages//pybind11//share//cmake//pybind11 -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE --no-warn-unused-cli -S C:/Users/15309/Project_Chrono/chrono_simulink_cosim -B c:/Users/15309/Project_Chrono/chrono_simulink_cosim/build -G "Visual Studio 16 2019"
#!/bin/bash

# configure_project.sh
# Configures the CMake project, automatically detecting active Conda environment.

echo "--- CMake Configuration Script ---"

# Determine the project root directory
PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd)

# Define the build directory path relative to the project root
BUILD_DIR="${PROJECT_ROOT}/build"

echo "Project Root: ${PROJECT_ROOT}"
echo "Build Directory: ${BUILD_DIR}"
echo "----------------------------------"

# Ensure the build directory exists
mkdir -p "${BUILD_DIR}"

# --- Conda Environment Detection ---
# Check if CONDA_PREFIX is set, indicating an active Conda environment
if [ -n "${CONDA_PREFIX}" ]; then
    echo "Active Conda environment detected: ${CONDA_PREFIX}"
    # Set CMake variables to help find Python and Pybind11 within the Conda env
    # Python3_ROOT_DIR is often the most effective way to point CMake to a specific Python installation.
    # pybind11 might be found automatically by CMake's find_package if PYBIND11_DIR isn't set,
    # but explicitly setting Python3_ROOT_DIR helps guide it.
    CONDA_CMAKE_ARGS="-DPython3_ROOT_DIR=${CONDA_PREFIX}"
    echo "CMake will search for Python and Pybind11 in: ${CONDA_PREFIX}"
else
    echo "No active Conda environment detected. CMake will use system Python/Pybind11."
    CONDA_CMAKE_ARGS=""
fi

# --- CMake Configuration Command ---
echo "Running CMake configuration from ${BUILD_DIR}..."
echo "Command: cmake ${CONDA_CMAKE_ARGS} \\"
echo "         -DChrono_DIR:STRING=c:/Users/15309/Project_Chrono/chrono_build/cmake \\"
echo "         -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE --no-warn-unused-cli \\"
echo "         -S \"${PROJECT_ROOT}\" -B \"${BUILD_DIR}\" -G \"Visual Studio 16 2019\""

# Execute the CMake configuration
(cd "${BUILD_DIR}" && \
 cmake \
 "${CONDA_CMAKE_ARGS}" \
 -DChrono_DIR:STRING="c:/Users/15309/Project_Chrono/chrono_build/cmake" \
 -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE \
 --no-warn-unused-cli \
 -S "${PROJECT_ROOT}" \
 -B "${BUILD_DIR}" \
 -G "Visual Studio 16 2019" \
)

if [ $? -eq 0 ]; then
    echo "CMake configuration successful! You can now build your project."
else
    echo "Error: CMake configuration failed. Please check the output above."
    exit 1
fi
