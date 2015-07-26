#version 130
uniform mat4 mat_View;
uniform mat4 mat_ModelView;
uniform mat4 mat_Proj;
in vec4 Vertex;
void main(){
    gl_Position = mat_Proj * mat_ModelView * Vertex;
//    gl_Position = mat_Proj * mat_ModelView * mat_View * Vertex;
//    gl_Position = mat_View * mat_ModelView * mat_Proj *  Vertex;
}
