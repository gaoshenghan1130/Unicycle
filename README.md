# STM32WB55 Dual-Build Project (CubeIDE + VS Code)

This repository contains an STM32WB55RG project that can be built and flashed **both in STM32CubeIDE** and **from VS Code** using CMake.

---

## Features
- **Dual build system**  
  - Native support for STM32CubeIDE build and debug  
  - CMake-based build for VS Code or command line  
- **VS Code integration**  
  - IntelliSense and code navigation  
  - Inline diagnostics  
  - Build within VS Code with CMake Tools extension  

---

## Notes

- VS Code CMake extension settings is included in `CMakerUserPresets.json`. You may need to adjust the paths according to your setup.
- In `run.sh`, adjust the path to `STM32_Programmer_CLI` according to your installation than you can flash the binary from command line.
- New file added to the projects should be included in `CMakeLists.txt` to be built in VS Code.