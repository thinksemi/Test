cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME VSC_Bootloader_AB_Test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

set(MBEDTLS_SOURCES
    Middlewares/mbedtls/library/sha256.c
    Middlewares/mbedtls/library/ecdsa.c
    Middlewares/mbedtls/library/ecp.c
    Middlewares/mbedtls/library/ecp_curves.c
    Middlewares/mbedtls/library/bignum.c
    Middlewares/mbedtls/library/bignum_core.c
    Middlewares/mbedtls/library/asn1parse.c
    Middlewares/mbedtls/library/asn1write.c
    Middlewares/mbedtls/library/platform_util.c
    Middlewares/mbedtls/library/platform.c
    Middlewares/mbedtls/library/constant_time.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MBEDTLS_SOURCES})


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/bootloader.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add mbedTLS sources and include paths (Middlewares/)
file(GLOB MBEDTLS_SRCS "${CMAKE_SOURCE_DIR}/Middlewares/library/*.c")
if(MBEDTLS_SRCS)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MBEDTLS_SRCS})
endif()
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Middlewares/mbedtls/include
    ${CMAKE_SOURCE_DIR}/Middlewares/mbedtls/include/mbedtls
    ${CMAKE_SOURCE_DIR}/Middlewares/mbedtls/library
)

# target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
#     Middlewares/mbedtls/include
# )



# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
