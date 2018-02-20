#version 330 core

layout (location = 0) in vec2 texCoords;

uniform sampler2D texOutput;
uniform mat4 mvp;

void main() {
  gl_Position = vec4(10,10,10,1.0); // invalid NDC
  
  vec4 point = texture(texOutput, texCoords);
  if(point.w > 0.5) {
    gl_Position = mvp * point; 
  }
}