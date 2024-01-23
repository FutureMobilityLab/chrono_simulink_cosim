# chrono_simulink_cosim
A minimal (in terms of source files and dependencies) project that connects [Project Chrono](https://github.com/projectchrono/chrono/tree/main) Vehicle simulation to Simulink using Project Chrono's Cosimulation module. This project aims to bring the power of Project Chrono's multi-body dynamics models of ground vehicles to interface with Matlab's Simulation easily.

## Dependencies
- [Project Chrono](https://github.com/projectchrono/chrono/tree/main) (Vehicle, Cosimulation, Irrlicht) (Tested on Release version 8.0.0)
  - please see [Project Chrono's Installation Guides](https://api.projectchrono.org/8.0.0/tutorial_table_of_content_install.html)
- Matlab/Simulink (Instrument Control Toolbox)
  - The instrument control toolbox is required because the Project Chrono executable has to talk to Simulink over TCP/IP.
- CMake (GUI is optional)
- A C++ Compiler for your system (ie. Visual Studio 2019)

## Getting Started
### Windows
#### Configuring and Building
If you are using VS Code, there is a `.vscode` directory already included for your convenience. This contains a `settings.json` and `c_cpp_properties.json`. To successfully configure this project you will have to tell CMake where to find the Chrono cmake directory. This is done in the `settings.json` shown below:
```
"cmake.configureSettings": {
    "Chrono_DIR":"C:\\Users\\Trevor\\Project Chrono\\chrono_build\\cmake"},
```
You will have to change this path to point to your Project Chrono's cmake directory. Note that `\\` is required in this path for Windows.

Once you change this you can configure the project in VS Code (if you have the necessary CMake and C++ extensions: CMake, CMake Tools, C/C++, C/C++ Extension Pack). To configure and build in VS Code refer to [Get started with CMake Tools on Linux](https://code.visualstudio.com/docs/cpp/cmake-linux) (I know it says Linux, but it also applies to Windows).

You can alternatively configure this project using the CMake GUI like you did for Project Chrono. I think you will only have to tell CMake where to find the Chrono cmake directory (whose instructions are the same as above). Make sure you set the variant to Release. Once configured, you can compile with Visual Studio just like you did for Project Chrono (if you used Visual Studio to do this).

#### Running
Before running, the generated executables need some data files. These files (and more) are included in the `data` directory, which are simply copied from the Project Chrono data files. The executable will look in a directory above it to find this directory, so it is recommended that you move (or copy) this directory to the `build` directory (or whichever directory you used to build this project). The structure of this project should then look like:
```
- chrono_simulink_cosim (where this repository is stored)
  - .vscode
  - build
    - data (where the model data is stored)
    - Release (where all the executables are stored)
      - cosim_forces.exe
      - cosim_simple.exe
      - ...
    - ...
  - data
  - Matlab
```

You can alternatively copy the entire Chrono Repository data directory to the `build` directory, but that contains a lot of extra files that you will not need.

Next, you will want to open one of the Simulink files in the `Matlab` directory. Each Simulink model corresponds to one executable (ie. `simple_cosimulation.slx` works with `cosim_simple.exe`). You will want to run the executable first since it will wait to make a connection with the Simulink Model.
