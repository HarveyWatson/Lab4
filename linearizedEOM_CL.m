function var_dot = linearizedEOM_CL(t, var, g, m, I)

var_dot = zeros(12, 1);

% Call InnerLoopFeedback
[deltaFc, deltaGc] = InnerLoopFeedback(var);

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

var_dot(1) = u_E;
var_dot(2) = v_E;
var_dot(3) = w_E;
var_dot(4) = p;
var_dot(5) = q;
var_dot(6) = r;
var_dot(7) = -g*theta;
var_dot(8) =  g*phi; 
var_dot(9) = 0/m; % no perturbation in control force, set to 0
var_dot(10) = deltaGc(1)/ I(1,1); 
var_dot(11) = deltaGc(2)/ I(2,2); 
var_dot(12) = deltaGc(3)/ I(3,3);

end
