cmake -S . -B Debug -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake

cd Debug
make

cd ..

/opt/ST/STM32CubeCLT_1.19.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
-c port=SWD -d Unicycle.elf