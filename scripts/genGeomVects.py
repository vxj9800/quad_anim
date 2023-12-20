import sympy
import sympy.physics.mechanics
import sympy.utilities.codegen

## System Description
# THE QUADCOPTER MODEL IS MADE OF FIVE BODIES. THE MAIN BODY IS BODY A, WHICH INCLUDES QUADCOPTER FRAME, FILGHT CONTROLLER ELECTRONICS, ESCS AND THE STATORS OF THE FOUR MOTORS. THE OTHER FOUR BODIES ARE B, C, D AND E. EACH OF THESE BODIES REPRESENT THE ROTOR OF EACH MOTOR ALONG WITH THE RIGIDLY ATTACHED PROPELLER. THE QUADCOPTER IS ASSUMED TO HAVE AN X CONFIGURATION WITH "PROP-IN" MOTOR ROTATION AS SHOWN BELOW.
# 
#        -->             <--
#       | B               E |
#           \           /
#              \     /
#                 X
#              /     \
#           /           \
#       | C               D |
#        -->             <--

## Define Symbolic Variables
# Define generalized coordinates
q1, q2, q3 = sympy.physics.mechanics.dynamicsymbols("q1:4", real=True); # 3 coordinates for the position of Body A
e0, e1, e2, e3 = sympy.physics.mechanics.dynamicsymbols("e0:4", real=True); # 4 euler parameters for the orientation of Body A
q4, q5, q6, q7 = sympy.physics.mechanics.dynamicsymbols("q4:8", real=True); # 4 coordinates for the rotation of the motors w.r.t. Body A

## Define other constants
# Geometric Constants
A_P_AB = sympy.MatrixSymbol("A_P_AB", 3, 1); # Constants to define position of Body B = [lB; wB; hB]
A_P_AC = sympy.MatrixSymbol("A_P_AC", 3, 1); # Constants to define position of Body C = [lC; wC; hC]
A_P_AD = sympy.MatrixSymbol("A_P_AD", 3, 1); # Constants to define position of Body D = [lD; wD; hD]
A_P_AE = sympy.MatrixSymbol("A_P_AE", 3, 1); # Constants to define position of Body E = [lE; wE; hE]
pDia = sympy.symbols("pDIa", positive=True); # Propeller diameter

## Define frames and points
# Inertial Frame and point
N = sympy.physics.mechanics.ReferenceFrame('N');
No = sympy.physics.mechanics.Point('No');
No.set_vel(N,0);

# Body attached frames and points
A123 = N.orientnew('A123','Quaternion',(e0,e1,e2,e3));
Ao = No.locatenew('Ao', q1*N.x + q2*N.y + q3*N.z);

B123 = A123.orientnew('B123','Axis',(q4,A123.z));
Bo = Ao.locatenew('Bo', A_P_AB[0]*A123.x + A_P_AB[1]*A123.y + A_P_AB[2]*A123.z);

C123 = A123.orientnew('C123','Axis',(q5,A123.z));
Co = Ao.locatenew('Co', A_P_AC[0]*A123.x + A_P_AC[1]*A123.y + A_P_AC[2]*A123.z);

D123 = A123.orientnew('D123','Axis',(q6,A123.z));
Do = Ao.locatenew('Do', A_P_AD[0]*A123.x + A_P_AD[1]*A123.y + A_P_AD[2]*A123.z);

E123 = A123.orientnew('E123','Axis',(q7,A123.z));
Eo = Ao.locatenew('Eo', A_P_AE[0]*A123.x + A_P_AE[1]*A123.y + A_P_AE[2]*A123.z);

## Compute all the position vectors in inertial frame
N_P_NA = Ao.pos_from(No).to_matrix(N);

N_P_NB = Bo.pos_from(No).to_matrix(N);
N_P_NB_prop1 = Bo.locatenew('Bo_prop1',pDia/2*B123.x).pos_from(No).to_matrix(N);
N_P_NB_prop2 = Bo.locatenew('Bo_prop2',-pDia/2*B123.x).pos_from(No).to_matrix(N);
N_P_NB_base = Ao.locatenew('Bo_base', A_P_AB[0]*A123.x + A_P_AB[1]*A123.y).pos_from(No).to_matrix(N);

N_P_NC = Co.pos_from(No).to_matrix(N);
N_P_NC_prop1 = Co.locatenew('Co_prop1',pDia/2*C123.x).pos_from(No).to_matrix(N);
N_P_NC_prop2 = Co.locatenew('Co_prop2',-pDia/2*C123.x).pos_from(No).to_matrix(N);
N_P_NC_base = Ao.locatenew('Co_base', A_P_AC[0]*A123.x + A_P_AC[1]*A123.y).pos_from(No).to_matrix(N);

N_P_ND = Do.pos_from(No).to_matrix(N);
N_P_ND_prop1 = Do.locatenew('Do_prop1',pDia/2*D123.x).pos_from(No).to_matrix(N);
N_P_ND_prop2 = Do.locatenew('Do_prop2',-pDia/2*D123.x).pos_from(No).to_matrix(N);
N_P_ND_base = Ao.locatenew('Do_base', A_P_AD[0]*A123.x + A_P_AD[1]*A123.y).pos_from(No).to_matrix(N);

N_P_NE = Eo.pos_from(No).to_matrix(N);
N_P_NE_prop1 = Eo.locatenew('Eo_prop1',pDia/2*E123.x).pos_from(No).to_matrix(N);
N_P_NE_prop2 = Eo.locatenew('Eo_prop2',-pDia/2*E123.x).pos_from(No).to_matrix(N);
N_P_NE_base = Ao.locatenew('Eo_base', A_P_AE[0]*A123.x + A_P_AE[1]*A123.y).pos_from(No).to_matrix(N);

x = sympy.MatrixSymbol('x',12,2);
y = sympy.MatrixSymbol('y',12,2);
z = sympy.MatrixSymbol('z',12,2);

xCoords = sympy.Equality(x,sympy.Matrix([
    [N_P_NA[0], N_P_NB_base[0]],
    [N_P_NA[0], N_P_NC_base[0]],
    [N_P_NA[0], N_P_ND_base[0]],
    [N_P_NA[0], N_P_NE_base[0]],
    [N_P_NB_base[0], N_P_NB[0]],
    [N_P_NC_base[0], N_P_NC[0]],
    [N_P_ND_base[0], N_P_ND[0]],
    [N_P_NE_base[0], N_P_NE[0]],
    [N_P_NB_prop1[0], N_P_NB_prop2[0]],
    [N_P_NC_prop1[0], N_P_NC_prop2[0]],
    [N_P_ND_prop1[0], N_P_ND_prop2[0]],
    [N_P_NE_prop1[0], N_P_NE_prop2[0]],
]),evaluate=False);

yCoords = sympy.Equality(y,sympy.Matrix([
    [N_P_NA[1], N_P_NB_base[1]],
    [N_P_NA[1], N_P_NC_base[1]],
    [N_P_NA[1], N_P_ND_base[1]],
    [N_P_NA[1], N_P_NE_base[1]],
    [N_P_NB_base[1], N_P_NB[1]],
    [N_P_NC_base[1], N_P_NC[1]],
    [N_P_ND_base[1], N_P_ND[1]],
    [N_P_NE_base[1], N_P_NE[1]],
    [N_P_NB_prop1[1], N_P_NB_prop2[1]],
    [N_P_NC_prop1[1], N_P_NC_prop2[1]],
    [N_P_ND_prop1[1], N_P_ND_prop2[1]],
    [N_P_NE_prop1[1], N_P_NE_prop2[1]],
]),evaluate=False);

zCoords = sympy.Equality(z,sympy.Matrix([
    [N_P_NA[2], N_P_NB_base[2]],
    [N_P_NA[2], N_P_NC_base[2]],
    [N_P_NA[2], N_P_ND_base[2]],
    [N_P_NA[2], N_P_NE_base[2]],
    [N_P_NB_base[2], N_P_NB[2]],
    [N_P_NC_base[2], N_P_NC[2]],
    [N_P_ND_base[2], N_P_ND[2]],
    [N_P_NE_base[2], N_P_NE[2]],
    [N_P_NB_prop1[2], N_P_NB_prop2[2]],
    [N_P_NC_prop1[2], N_P_NC_prop2[2]],
    [N_P_ND_prop1[2], N_P_ND_prop2[2]],
    [N_P_NE_prop1[2], N_P_NE_prop2[2]],
]),evaluate=False);

## Substitute state variables
Q = sympy.MatrixSymbol('Q',11,1);
xCoords = xCoords.subs({q1:Q[0,0],q2:Q[1,0],q3:Q[2,0],e0:Q[3,0],e1:Q[4,0],e2:Q[5,0],e3:Q[6,0],q4:Q[7,0],q5:Q[8,0],q6:Q[9,0],q7:Q[10,0]});
yCoords = yCoords.subs({q1:Q[0,0],q2:Q[1,0],q3:Q[2,0],e0:Q[3,0],e1:Q[4,0],e2:Q[5,0],e3:Q[6,0],q4:Q[7,0],q5:Q[8,0],q6:Q[9,0],q7:Q[10,0]});
zCoords = zCoords.subs({q1:Q[0,0],q2:Q[1,0],q3:Q[2,0],e0:Q[3,0],e1:Q[4,0],e2:Q[5,0],e3:Q[6,0],q4:Q[7,0],q5:Q[8,0],q6:Q[9,0],q7:Q[10,0]});

## Generate Code
[(c_name, c_code), (h_name, c_header)] = sympy.utilities.codegen.codegen(
    name_expr=[('xCoords',xCoords)],
    language='C',
    project="quad_anim",
    to_files=False,
    header=True,
    argument_sequence=(Q,A_P_AB,A_P_AC,A_P_AD,A_P_AE,pDia,x)
);
cFile = open("./src/" + c_name,"w"); cFile.write(c_code); cFile.close();
hFile = open("./include/" + h_name,"w"); hFile.write(c_header); hFile.close();

[(c_name, c_code), (h_name, c_header)] = sympy.utilities.codegen.codegen(
    name_expr=[('yCoords',yCoords)],
    language='C',
    project="quad_anim",
    to_files=False,
    header=True,
    argument_sequence=(Q,A_P_AB,A_P_AC,A_P_AD,A_P_AE,pDia,y)
);
cFile = open("./src/" + c_name,"w"); cFile.write(c_code); cFile.close();
hFile = open("./include/" + h_name,"w"); hFile.write(c_header); hFile.close();

[(c_name, c_code), (h_name, c_header)] = sympy.utilities.codegen.codegen(
    name_expr=[('zCoords',zCoords)],
    language='C',
    project="quad_anim",
    to_files=False,
    header=True,
    argument_sequence=(Q,A_P_AB,A_P_AC,A_P_AD,A_P_AE,pDia,z)
);
cFile = open("./src/" + c_name,"w"); cFile.write(c_code); cFile.close();
hFile = open("./include/" + h_name,"w"); hFile.write(c_header); hFile.close();