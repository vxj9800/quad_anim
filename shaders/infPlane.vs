#version 330

// Adapted from
// 1. https://gist.github.com/cmbruns/3c184d303e665ee2e987e4c1c2fe4b56
// 2. https://asliceofrendering.com/scene%20helper/2020/01/05/InfiniteGrid/

in vec3 vertexPosition;   // Vertex input attribute: position
in vec4 vertexColor;      // Vertex input attribute: color

out vec4 fragColor;   // To-fragment attribute: color
out vec4 fragVert;
out vec4 fragNdc;

uniform mat4 mvp;           // Model-View-Projection matrix
uniform mat4 matModel;          // Model matrix
uniform mat4 matView;          // View matrix
uniform mat4 matProjection;          // Projection matrix

vec3 dehomog(vec4 v)
{
    return v.xyz/v.w;
}

void main()
{
    // Convert the model plane into clip plane
    vec4 cpVertex = matModel*vec4(vertexPosition,1.0);
    gl_Position = vec4(cpVertex.xyz, 1.0);
    fragColor = vertexColor;

    vec3 planeOrgDist = dehomog(matView * mat4(1) * vec4(0, 0, 0, 1)); // Distance between plane and origin in view frame

    vec4 D = inverse(matProjection) * cpVertex; // Clip coordinates in view frame
    vec4 N = matView * mat4(1) * vec4(0, 0, 1, 0); // Plane normal in view frame
    float d = -dot(N.xyz, planeOrgDist); // Dot product between 

    fragVert = vec4(     // Point on the plane in view frame
        -d * D.xyz,  // xyz, numerator
        dot(D.xyz, N.xyz)  // w, denominator
    );

    fragNdc = matProjection * fragVert; // Put the point back into projection frame
}