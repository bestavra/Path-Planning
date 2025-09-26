# GLAD - OpenGL Function Loader Generated from https://glad.dav1d.de/ OpenGL 3.3
# Core Profile

# Enable C language for this target
enable_language(C)

add_library(glad STATIC gl.c)

# Set the language property explicitly
set_source_files_properties(gl.c PROPERTIES LANGUAGE C)

target_include_directories(
  glad PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>)

# Set properties for the library
set_target_properties(glad PROPERTIES LINKER_LANGUAGE C
                                      POSITION_INDEPENDENT_CODE ON)

# Suppress warnings from third-party code
if(MSVC)
  target_compile_options(glad PRIVATE /w)
else()
  target_compile_options(glad PRIVATE -w)
endif()

# Create an alias for easier usage
add_library(third_party::glad ALIAS glad)
