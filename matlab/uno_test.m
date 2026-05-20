function uno_test()

problem = baseproblem("ipopt");
problem.options.linear_solver = "MUMPS";
assert(solveproblem(problem),"Failed with MUMPS");

problem.options.linear_solver = "MA27";
assert(solveproblem(problem),"Failed with MA27");

problem.options.linear_solver = "MA57";
assert(solveproblem(problem),"Failed with MA57");

problem.options.hessian_model = "LBFGS";
assert(solveproblem(problem),"Failed with LBFGS");

problem.options.hessian_model = "LSR1";
assert(solveproblem(problem),"Failed with LSR1");

problem = baseproblem("filtersqp");
problem.options.QP_solver = "BQPD";
assert(solveproblem(problem),"Failed with BQPD");

problem.options.QP_solver = "HiGHS";
assert(solveproblem(problem),"Failed with HiGHS");

problem.options.QP_solver = "BQPD";
problem.options.hessian_model = "LBFGS";
assert(solveproblem(problem),"Failed with LBFGS");

problem.options.hessian_model = "LSR1";
assert(solveproblem(problem),"Failed with LSR1");
end

function bSuccess = solveproblem(problem)
    res = uno_mex(problem);
    assert("Success" == res.termination_status, "Failed to converge, status %s", res.termination_status);
    % Use Uno-reported residuals (preset-consistent); manual KKT is exact-Hessian only.
    tol = 1e-5;
    if isfield(problem.options, "preset") && problem.options.preset == "ipopt"
        tol = 1e-7;
    end
    assert(res.primal_feasibility < tol, ...
        "Primal feasibility %g exceeds %g", res.primal_feasibility, tol);
    assert(res.stationarity < tol, ...
        "Stationarity %g exceeds %g", res.stationarity, tol);
    bSuccess = true;
end

function problem = baseproblem(preset)

options = uno_options(preset);

funcs.objective = @objective;
funcs.gradient = @gradient;
funcs.constraints = @constraints;
funcs.jacobian = @jacobian;
funcs.hessian = @hessian;

variableInfo.x0 = [3;2];
variableInfo.lBnds = [-5;-5];
variableInfo.uBnds = [5;5];
variableInfo.clBnds = -inf;
variableInfo.cuBnds = -2;
variableInfo.jacobianNnz = 2;
variableInfo.hessianNnz = 3;

callbacks = struct;
% callbacks.acceptable_iterate_callback = @(x,lambda,sigma)disp("acceptable Iterate");

problem.options = options;
problem.funcs = funcs;
problem.variableInfo = variableInfo;
problem.callbacks = callbacks;
end


% ----------------------------------------------------------------------
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
hVal = tril(sparse(sigma*H - lambda(1)*C));
end