#version 330 core

layout (location = 0) in vec4 position_in;

uniform mat4 mvp;          // model-view-projection matrix. (applied p*v*m)

out vec4 pos;  // this will be linearly interpolated between the three triangle corners.

void main() {
  gl_Position = mvp * position_in;  // this must be in NDC.
  pos = position_in;
}
