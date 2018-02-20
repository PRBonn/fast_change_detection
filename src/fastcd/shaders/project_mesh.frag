#version 330

in vec4 pos;
out vec4 pos_out;

void main() {
  pos_out = vec4(pos.xyz, 1.0);  // that's it! position and valid flag.
}
