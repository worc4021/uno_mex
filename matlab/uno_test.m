options = uno_options("ipopt");

funcs.objective = @objective;
funcs.gradient = @gradient;
funcs.constraints = @constraints;
funcs.jacobian = @jacobian;
funcs.hessian = @hessian;
funcs.gradient_nonzeros = @()2;
funcs.jacobian_nonzeros = @()2;
funcs.hessian_nonzeros = @()4;

variableInfo.x0 = [3;2];
variableInfo.lb = [-5;-5];
variableInfo.ub = [5;5];
variableInfo.clBnds = -inf;
variableInfo.cuBnds = -2;

unostr.options = options;
unostr.funcs = funcs;
unostr.variableInfo = variableInfo;

uno_mex(unostr)

function fVal = objective(var)
x = var(1,:);
y = var(2,:);
fVal = (x.^2+y-11).^2+(x+y.^2-7).^2;
end
% ----------------------------------------------------------------------
function fGrad = gradient(var)
x = var(1);
y = var(2);

fGrad = [2*x + 4*x*(x^2 + y - 11) + 2*y^2 - 14;
         2*y + 4*y*(y^2 + x - 7) + 2*x^2 - 22];
fGrad = sparse(fGrad);

end

function c = constraints(var)
x = var(1,:);
y = var(2,:);

c = sin(x)-y;
end

function j = jacobian(var)
x = var(1);
j = sparse([cos(x),-1]);
end

% ----------------------------------------------------------------------
function hVal = hessian (var, sigma, lambda)

x = var(1);
y = var(2);
    
H = [12*x^2 + 4*y - 42,4*x + 4*y;
    4*x + 4*y,12*y^2 + 4*x - 26];
C = [sin(x), 0;
     0, 0];
hVal = sparse(sigma*H - lambda(1)*C);
end