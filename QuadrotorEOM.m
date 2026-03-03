function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

x_E = var(1);
y_E = var(2);
z_E = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
u_E = var(7);
v_E = var(8);
w_E = var(9);
p = var(10);
q = var(11);
r = var(12);

A1 = [cos(theta)*cos(psi), (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    cos(theta)*sin(psi), (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
    -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

A2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

A3 = [(r*v_E - q*w_E), (p*w_E - r*u_E), (q*u_E - p*v_E)]';

A4 = [-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)]';

A5 = [((Iy - Iz)/Ix)*q*r, ((Iz - Ix)/Iy)*p*r, ((Ix-Iy)/Iz)*p*q]';

f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

Z_C = -(f1 + f2 + f3 + f4);
L_C = (d/sqrt(2))*(-f1 -f2 +f3 +f4);
M_C = (d/sqrt(2))*(f1 -f2 -f3 +f4);
N_C = (km)*(f1 -f2 +f3 -f4);

var_dot_XYZ = A1*[u_E, v_E, w_E]';
var_dot_O = A2*[p, q, r]';
var_dot_UVW = A3 + g*A4 + (1/m)*[0,0,Z_C]';
var_dot_PQR = A5 + [(1/Ix)*L_C, (1/Iy)*M_C, (1/Iz)*N_C]';

var_dot = [var_dot_XYZ, var_dot_O, var_dot_UVW, var_dot_PQR]';

% aerodynamic forces/moments to be added
end