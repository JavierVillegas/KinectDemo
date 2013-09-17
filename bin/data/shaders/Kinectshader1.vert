
//varying float ycord;
uniform int zebra;

void main()
{
 
    if (zebra!=0){
        int N = 20;
        float TH = .1;
        float OutZ =0.;
        OutZ = gl_Vertex.z/20.0;
        for (int k = 0; k < N; k++) {
          // up
            if (OutZ >TH) {
                OutZ = 2.*TH - OutZ;
            }
        
            // down
        
            if (OutZ< 0.){
                OutZ = -OutZ;
            }
        }
        OutZ *=8.;
        gl_FrontColor = vec4(OutZ,OutZ,OutZ,1.0);
    }
    else{
    
        //gl_FrontColor = gl_Color;
        gl_FrontColor = vec4(gl_Vertex.z/200.,gl_Vertex.z/200.,gl_Vertex.z/200.,1.0);
    }
    vec4 newcords = gl_Vertex;
  //  ycord = gl_Vertex.y;
    newcords.z = 4.0*newcords.z;
//    newcords.z = newcords.y;
    gl_Position = gl_ModelViewProjectionMatrix*newcords;
                    
    //ftransform();
} 
