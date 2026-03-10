function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

I_principles = diag(I);
Ix = I_principles(1);
Iy = I_principles(2);
Iz = I_principles(3);

var_dot = zeros(12, 1);

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

% drag + moments, assuming 0 wind
Va_vec = [u_E, v_E, w_E]';
Drag = -nu*norm(Va_vec)*Va_vec;

omega_vec = [p, q, r]';
Moments = -mu*norm(omega_vec)*omega_vec;

var_dot_XYZ = A1*[u_E, v_E, w_E]';
var_dot_O = A2*[p, q, r]';
var_dot_UVW = A3 + g*A4 + (1/m)*[0,0,Z_C]' + (1/m)*Drag; % with aero drag
var_dot_PQR = A5 + [(1/Ix)*L_C, (1/Iy)*M_C, (1/Iz)*N_C]' + [(1/Ix)*Moments(1), (1/Iy)*Moments(2), (1/Iz)*Moments(3)]'; % with aero moment

var_dot(1) = var_dot_XYZ(1);
var_dot(2) = var_dot_XYZ(2);
var_dot(3) = var_dot_XYZ(3);

var_dot(4) = var_dot_O(1);
var_dot(5) = var_dot_O(2);
var_dot(6) = var_dot_O(3);

var_dot(7) = var_dot_UVW(1);
var_dot(8) = var_dot_UVW(2);
var_dot(9) = var_dot_UVW(3);

var_dot(10) = var_dot_PQR(1);
var_dot(11) = var_dot_PQR(2);
var_dot(12) = var_dot_PQR(3);

% aerodynamic forces/moments to be added
end
