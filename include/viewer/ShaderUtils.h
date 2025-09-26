#pragma once

#include <string>

namespace ShaderUtils {

/**
 * @brief Compiles a shader from source code.
 * @param shaderSource Null-terminated GLSL source code string.
 * @param shaderType OpenGL shader type (e.g., GL_VERTEX_SHADER, GL_FRAGMENT_SHADER).
 * @return OpenGL shader object ID, or 0 if compilation failed.
 */
unsigned int compileShader(const char* shaderSource, unsigned int shaderType);

/**
 * @brief Links vertex and fragment shaders into a program.
 * @param vertexShader Compiled vertex shader ID to link.
 * @param fragmentShader Compiled fragment shader ID to link.
 * @return OpenGL shader program ID, or 0 if linking failed.
 */
unsigned int linkProgram(unsigned int vertexShader, unsigned int fragmentShader);

/**
 * @brief Creates a complete shader program from vertex and fragment source.
 * @param vertexSource Null-terminated vertex shader source string.
 * @param fragmentSource Null-terminated fragment shader source string.
 * @return OpenGL shader program ID, or 0 if creation failed.
 */
unsigned int createShaderProgram(const char* vertexSource, const char* fragmentSource);

} // namespace ShaderUtils