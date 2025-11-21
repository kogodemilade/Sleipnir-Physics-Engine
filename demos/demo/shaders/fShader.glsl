#version 330 core
// uniform Vec4 VertexColor;

uniform sampler2D myTexture;

in vec3 VertexColor;
// in vec3 pos;
in vec2 TexCoord;
out vec4 FragColor;

void main(){
    FragColor = vec4(VertexColor, 1.0f);
    // FragColor = texture(myTexture, TexCoord) * vec4(pos, 1.0);

}