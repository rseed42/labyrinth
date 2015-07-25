#version 130
uniform vec4 vec_Color;

void main(){
//  gl_FragColor = vec4(0,0,1,0.7);
  gl_FragColor = vec_Color;
}
