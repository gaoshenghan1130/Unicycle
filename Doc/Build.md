# STM32WB55 Auto-Balance Unicycle Control - Dual Build System

This project can be built built and flashed **both in STM32CubeIDE** and **from VS Code** using CMake.

---

## Features
- **CUBEIDE integration**  
  - Native support for STM32CubeIDE build and debug  
   
- **VS Code integration**  
  - IntelliSense and code navigation  
  - Inline diagnostics  
  - Build within VS Code with CMake Tools extension  
  - CMake-based build for VS Code or command line 

---

## Notes

- VS Code CMake extension settings is included in `CMakerUserPresets.json`. You may need to adjust the paths according to your setup.
- In `run.sh`, adjust the path to `STM32_Programmer_CLI` according to your installation than you can flash the binary from command line.
- New file added to the projects should be included in `CMakeLists.txt` to be built in VS Code.