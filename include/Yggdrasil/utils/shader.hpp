#pragma once
#ifndef SHADER_H
#define SHADER_H

#include "../glad/glad.h" /*Include glad to get all required opengl headers*/
#include "../glm/glm.hpp"
#include "../glm/ext.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

/*Creates a shader Program*/
class Shader {
    public: 
        //Program ID
        unsigned int ID;

        const char * vertexShaderSource;
        const char * fragmentShaderSource;


        //Constructor reads and builds the shader Object
        Shader(const char * VertexPath, const char* fragmentPath);

        Shader();
        //Use/activate the shader
        void use();

        //Utility uniform functions
        void setBool(const std::string &name, bool value) const;
        void setInt(const std::string &name, int value) const;
        void setFloat(const std::string &name, float value) const;
        void setMat4(const std::string &name, glm::mat4 value) const;
        void setVec3(const std::string &name, glm::vec3 value) const;

        /*return the C style source code for a shader type associated with this object.
        @param type The type of shader. v for Vertex and f for fragment*/

        const char* getshaderSource(const char type) const;
};

/*Creates a new Shader object. 
@param vertexPath The absolute file path for the vertex Shader glsl file
@param fragmentPath The absolute file path for the fragment shader glsl file*/
inline Shader::Shader(const char* vertexPath, const char* fragmentPath){
/*Retrieve vertex/fragment source code from file paths*/
std::string vertexCode;
std::string fragmentCode;
std::ifstream vShaderFile;
std::ifstream fShaderFile;

//Ensure ifstream objects can throw exceptions
vShaderFile.exceptions (std::ifstream:: failbit | std::ifstream::badbit);
fShaderFile.exceptions (std::ifstream:: failbit | std::ifstream::badbit);

//Read code from the files
try {
    /*Open files*/
    vShaderFile.open(vertexPath);
    fShaderFile.open(fragmentPath);
    std::stringstream vShaderStream, fShaderStream;

    //Read file's buffer contents into streams
    vShaderStream << vShaderFile.rdbuf();
    fShaderStream << fShaderFile.rdbuf();

    //Close file handlers
    vShaderFile.close();
    fShaderFile.close();

    //Convert stream to string
    vertexCode = vShaderStream.str();
    fragmentCode = fShaderStream.str();
} catch(std::ifstream::failure e) {
    std::cout<< "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ"<<std::endl;
}

//convert to C style strings 
const char *vShaderCode = vertexCode.c_str();
const char *fShaderCode = fragmentCode.c_str();


//Create vertex and fragment shaders
unsigned vertex, fragment;
int success;
char infoLog[512];
vertex = glCreateShader(GL_VERTEX_SHADER);
glShaderSource(vertex, 1, &vShaderCode, NULL);
glCompileShader(vertex);
//Print compile errors if any
glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
if(!success){
    glGetShaderInfoLog(vertex, 512, NULL, infoLog);
    std::cout<<"ERROR:SHADER::VERTEX::COMPILATION_FAILED\n"<<infoLog<<std::endl;
}


fragment = glCreateShader(GL_FRAGMENT_SHADER);
glShaderSource(fragment, 1, &fShaderCode, NULL);
glCompileShader(fragment);
//Print compile errors if any
glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
if(!success){
    glGetShaderInfoLog(fragment, 512, NULL, infoLog);
    std::cout<<"ERROR:SHADER::FRAGMENT::COMPILATION_FAILED\n"<<infoLog<<std::endl;
}

//Create Program
ID = glCreateProgram();
glAttachShader(ID, vertex);
glAttachShader(ID, fragment);
glLinkProgram(ID);
glGetProgramiv(ID, GL_LINK_STATUS, &success);
if(!success){
    glGetProgramInfoLog(ID, 512, NULL, infoLog);
    std::cout<<"ERROR::SHADER::PROGRAM::LINKING_FAILED\n"<<infoLog<<std::endl;
}

//Delete the shaders as they're linked into our program and no longer necessarily
glDeleteShader(vertex);
glDeleteShader(fragment);
}

inline Shader::Shader(){}

inline void Shader::use(){
    glUseProgram(ID);
}

//Set uniform values
inline void Shader::setBool(const std::string &name, bool value) const {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}

inline void Shader::setInt(const std::string &name, int value) const {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}

inline void Shader::setFloat(const std::string &name, float value) const {
    glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}

inline void Shader::setMat4(const std::string &name, glm::mat4 value) const{
    //Second arg is how many matrices we're sending, third is whether we want to transpose matrix, 4th is thematrix data
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
}

inline void Shader::setVec3(const std::string &name, glm::vec3 value) const{
    //Second arg is how many matrices we're sending, third is whether we want to transpose matrix, 4th is thematrix data
    glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(value));
}

inline const char* Shader::getshaderSource(const char type) const {
    if (type=='v') {
        return vertexShaderSource;
    }
    return fragmentShaderSource;
}


#endif