# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

set(MODEL_FOLDER .)
set(EI_SDK_FOLDER edge-impulse-sdk)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(gGlove C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} main.cpp)

include(${MODEL_FOLDER}/edge-impulse-sdk/cmake/utils.cmake)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_i2c)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${MODEL_FOLDER}
    ${MODEL_FOLDER}/classifer
    ${MODEL_FOLDER}/tflite-model
    ${MODEL_FOLDER}/model-parameters
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${EI_SDK_FOLDER}
    ${EI_SDK_FOLDER}/third_party/ruy
    ${EI_SDK_FOLDER}/third_party/gemmlowp
    ${EI_SDK_FOLDER}/third_party/flatbuffers/include
    ${EI_SDK_FOLDER}/third_party
    ${EI_SDK_FOLDER}/tensorflow
    ${EI_SDK_FOLDER}/dsp
    ${EI_SDK_FOLDER}/classifier
    ${EI_SDK_FOLDER}/anomaly
    ${EI_SDK_FOLDER}/CMSIS/NN/Include
    ${EI_SDK_FOLDER}/CMSIS/DSP/PrivateInclude
    ${EI_SDK_FOLDER}/CMSIS/DSP/Include
    ${EI_SDK_FOLDER}/CMSIS/Core/Include
)

include_directories(${INCLUDES})

# find model source files
RECURSIVE_FIND_FILE(MODEL_FILES "${MODEL_FOLDER}/tflite-model" "*.cpp")
RECURSIVE_FIND_FILE(SOURCE_FILES "${EI_SDK_FOLDER}" "*.cpp")
RECURSIVE_FIND_FILE(CC_FILES "${EI_SDK_FOLDER}" "*.cc")
RECURSIVE_FIND_FILE(S_FILES "${EI_SDK_FOLDER}" "*.s")
RECURSIVE_FIND_FILE(C_FILES "${EI_SDK_FOLDER}" "*.c")
list(APPEND SOURCE_FILES ${S_FILES})
list(APPEND SOURCE_FILES ${C_FILES})
list(APPEND SOURCE_FILES ${CC_FILES})
list(APPEND SOURCE_FILES ${MODEL_FILES})

# add all sources to the project
target_sources(${PROJECT_NAME} PRIVATE ${SOURCE_FILES})
# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})
