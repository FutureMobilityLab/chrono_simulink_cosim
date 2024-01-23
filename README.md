# chrono_simulink_cosim
A minimal (in terms of source files and dependencies) project that connects [Project Chrono](https://github.com/projectchrono/chrono/tree/main) Vehicle simulation to Simulink using Project Chrono's Cosimulation module. This project aims to bring the power of Project Chrono's multi-body dynamics models of ground vehicles to interface with Matlab's Simulation easily.

## Dependencies
- [Project Chrono](https://github.com/projectchrono/chrono/tree/main) (Vehicle, Cosimulation, Irrlicht) (Tested on Release version 8.0.0)
  - please see [Project Chrono's Installation Guides](https://api.projectchrono.org/8.0.0/tutorial_table_of_content_install.html)
  - To help you when installing Project Chrono, we only need the Vehicle, Cosimulation, and Irrlicht modules. For Release 8.0.0 you will need:
    - A C/C++ Compiler
      - For Windows the Visual Studio Community 2019 or newer is recommended.
      - You can find the link to the download [here](https://visualstudio.microsoft.com/downloads/)
    - The Eigen Library version 3.4.0
      - This requires no installation. All you need to do is download the zip file and extract it to some location you'll remember. I recommend keeping all of your Project Chrono files in one directory ie. `Project_Chrono`, and keeping the associated libraries in a subdirectory `libraries`
      - You can get the zip file from [here](https://eigen.tuxfamily.org/index.php?title=Main_Page)
      - Or you can download the zip directly from [here](https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip) is a link to the `3.4.0` version zip folder in case the previous link is more up-to-date.
      - If your Windows system is set to a language that doesn't support Unicode (ie. Chinese Traditional) you will have issues when compiling Chrono with Eigen3. The easiest way to resolve this is to change your Windows system to use a language that does support Unicode (ie. English (United States)). There are other alternatives such as setting compiler flags, but these may be more complicated.
    - The Irrlicht library version 1.8.5
      - This is an open source graphics rendering library. This requires no installation, just extract it to some location you'll remember.
      - You can get this version from the project's downloads webpage [here](https://irrlicht.sourceforge.io/?page_id=10)
      - Or you can click on this direct link from Source Forge [here](https://sourceforge.net/projects/irrlicht/files/Irrlicht%20SDK/1.8/1.8.4/irrlicht-1.8.4.zip/download?use_mirror=versaweb)
    - CMake
      - You can install CMake from [here](https://cmake.org/download/)
      - I recommend using the Windows x64 Installer for simplicity. You can get it directly from [here](https://github.com/Kitware/CMake/releases/download/v3.28.1/cmake-3.28.1-windows-x86_64.msi)
    - A GIT Client
      - I recommend installing Git and Git Bash if you want to work in a terminal (linux-like). You can install it from [here](https://gitforwindows.org/)
      - You can alternatively use [SourceTree](https://www.sourcetreeapp.com/) for a GUI.
- Matlab/Simulink (Instrument Control Toolbox)
  - The instrument control toolbox is required because the Project Chrono executable has to talk to Simulink over TCP/IP.
- CMake (GUI is optional)
- A C++ Compiler for your system (ie. Visual Studio 2019)

## Getting Started
### Windows
#### Configuring and Building
##### 1. Tell CMake where you installed Chrono

If you are using VS Code, there is a `.vscode` directory already included for your convenience. This contains a `settings.json` and `c_cpp_properties.json`. To successfully configure this project you will have to tell CMake where to find the Chrono cmake directory. This is done in the `settings.json` shown below:
```
"cmake.configureSettings": {
    "Chrono_DIR":"C:\\Users\\Trevor\\Project Chrono\\chrono_build\\cmake"},
```
You will have to change this path to point to your Project Chrono's cmake directory. Note that `\\` is required in this path for Windows.

##### 2. Configure project

To configure the project in VS Code you will need the necessary CMake and C++ extensions: CMake, CMake Tools, C/C++, C/C++ Extension Pack. You can install these by searching for them in the VS Code Extensions in the Markeplace. Learn more about how to do this [here](https://code.visualstudio.com/docs/editor/extension-marketplace). 

1. To configure the project press `CTRL+SHIFT+P` to open the VS Code Command Pallette (learn more [here](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette)) and run `CMake: Select a Kit`. The extension will scan your computer for compiler kits.
2. Select the compiler you want. (Typical Windows computers use x64 architecture)
   - ie. Visual Studio Community 2019 Community Release - amd64

You can also refer to the VS Code instructions for selecting a kit [here](https://code.visualstudio.com/docs/cpp/cmake-linux#_select-a-kit).

You can alternatively configure this project using the CMake GUI like you did for Project Chrono. I think you will only have to tell CMake where to find the Chrono cmake directory (whose instructions are the same as above). Make sure you set the variant to Release. Once configured, you can compile with Visual Studio just like you did for Project Chrono (if you used Visual Studio to do this).

##### 3. Build project

To build in VS Code you can simply click on the `Build` button in the lower toolbar. It is marked with a gear icon. Read how to build a project [here](https://code.visualstudio.com/docs/cpp/cmake-linux#_build-hello-world). 

For the full instructions on configuring and building in VS Code refer to [Get started with CMake Tools on Linux](https://code.visualstudio.com/docs/cpp/cmake-linux) (I know it says Linux, but it also applies to Windows).

##### 4. Move data director

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

#### Running

For user-friendliness each Simulink model (found in the `Matlab` directory) corresponds to a single executable (ie. `simple_cosimulation.slx` works with `cosim_simple.exe`). The Simulink model has a timeout associated with the TCP connection, and won't wait forever. So it is easier to run the executable first, and then run the Simulink model.
