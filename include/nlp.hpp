#pragma once

#include <vector>
#include <span>
#include <limits>
#include "model/Model.hpp"
#include "linear_algebra/SparseVector.hpp"
#include "linear_algebra/Vector.hpp"
#include "linear_algebra/RectangularMatrix.hpp"
#include "linear_algebra/SymmetricMatrix.hpp"
#include "optimization/Iterate.hpp"
#include "symbolic/CollectionAdapter.hpp"
#include "tools/UserCallbacks.hpp"
#include "tools/Timer.hpp"

namespace unomex
{

    struct Result
    {
        struct Solution {
            std::vector<double> primals;
            std::vector<double> duals_lb_x;
            std::vector<double> duals_ub_x;
            std::vector<double> duals_constraints;
            Solution(size_t number_variables, size_t number_constraints)
                : primals(number_variables)
                , duals_lb_x(number_variables)
                , duals_ub_x(number_variables)
                , duals_constraints(number_constraints)
            {
            }
            Solution() = default;
        } solution;
        size_t number_variables;
        size_t number_constraints;
        double cpu_time;
        uno::OptimizationStatus termination_status;
        
        Result(std::size_t number_variables, std::size_t number_constraints)
            : solution(number_variables, number_constraints)
            , number_variables(number_variables)
            , number_constraints(number_constraints)
            , cpu_time(0.)
            , termination_status(uno::OptimizationStatus::ITERATION_LIMIT)
        {
        }
        Result() = default;
    };

extern unomex::Result result;

    class DataModel : public uno::Model
    {
    protected:
        std::vector<double> _variable_lower_bounds;
        std::vector<double> _variable_upper_bounds;
        std::vector<double> _constraint_lower_bounds;
        std::vector<double> _constraint_upper_bounds;

        std::vector<uno::BoundType> _variable_status;    /*!< Status of the variables (EQUALITY, BOUNDED_LOWER, BOUNDED_UPPER, BOUNDED_BOTH_SIDES) */
        std::vector<uno::FunctionType> _constraint_type; /*!< Types of the constraints (LINEAR, QUADRATIC, NONLINEAR) */
        std::vector<uno::BoundType> _constraint_status;  /*!< Status of the constraints (EQUAL_BOUNDS, BOUNDED_LOWER, BOUNDED_UPPER, BOUNDED_BOTH_SIDES,UNBOUNDED) */
        std::vector<size_t> _linear_constraints;
        

        uno::SparseVector<size_t> _slacks{};

        uno::Timer _timer;
    private:
        // lists of variables and constraints + corresponding collection objects
        std::vector<size_t> _equality_constraints;
        std::vector<size_t> _inequality_constraints;
        uno::CollectionAdapter<std::vector<size_t> &> _equality_constraints_collection;
        uno::CollectionAdapter<std::vector<size_t> &> _inequality_constraints_collection;
        std::vector<size_t> _lower_bounded_variables;
        uno::CollectionAdapter<std::vector<size_t> &> _lower_bounded_variables_collection;
        std::vector<size_t> _upper_bounded_variables;
        uno::CollectionAdapter<std::vector<size_t> &> _upper_bounded_variables_collection;
        std::vector<size_t> _single_lower_bounded_variables; // indices of the single lower-bounded variables
        uno::CollectionAdapter<std::vector<size_t> &> _single_lower_bounded_variables_collection;
        std::vector<size_t> _single_upper_bounded_variables; // indices of the single upper-bounded variables
        uno::Vector<size_t> _fixed_variables;
        uno::CollectionAdapter<std::vector<size_t> &> _single_upper_bounded_variables_collection;
        uno::CollectionAdapter<std::vector<size_t> &> _linear_constraints_collection;

    public:
        

        DataModel(size_t number_variables, size_t number_constraints, const std::string &name = "DataModel")
            : uno::Model(name, number_variables, number_constraints, 1.)
            , _variable_lower_bounds(number_variables)
            , _variable_upper_bounds(number_variables)
            , _constraint_lower_bounds(number_constraints)
            , _constraint_upper_bounds(number_constraints)
            , _variable_status(number_variables)
            , _constraint_type(number_constraints)
            , _constraint_status(number_constraints)
            , _linear_constraints(0)
            , _slacks(0)
            , _timer()
            , _equality_constraints(0)
            , _inequality_constraints(0)
            , _equality_constraints_collection(_equality_constraints)
            , _inequality_constraints_collection(_inequality_constraints)
            , _lower_bounded_variables(0)
            , _lower_bounded_variables_collection(_lower_bounded_variables)
            , _upper_bounded_variables(0)
            , _upper_bounded_variables_collection(_upper_bounded_variables)
            , _single_lower_bounded_variables(0)
            , _single_lower_bounded_variables_collection(_single_lower_bounded_variables)
            , _single_upper_bounded_variables(0)
            , _fixed_variables(0)
            , _single_upper_bounded_variables_collection(_single_upper_bounded_variables)
            , _linear_constraints_collection(_linear_constraints)
        {
        }

        virtual ~DataModel() override = default;

        const uno::Collection<std::size_t> &get_equality_constraints() const override
        {
            return _equality_constraints_collection;
        }

        const uno::Collection<std::size_t> &get_inequality_constraints() const override
        {
            return _inequality_constraints_collection;
        }

        const uno::Collection<std::size_t> &get_linear_constraints() const override
        {
            return _linear_constraints_collection;
        }

        const uno::SparseVector<std::size_t> &get_slacks() const override
        {
            return _slacks;
        }

        const uno::Collection<std::size_t> &get_single_lower_bounded_variables() const override
        {
            return _single_lower_bounded_variables_collection;
        }

        const uno::Collection<std::size_t> &get_single_upper_bounded_variables() const override
        {
            return _single_upper_bounded_variables_collection;
        }

        const uno::Collection<std::size_t> &get_lower_bounded_variables() const override
        {
            return _lower_bounded_variables_collection;
        }

        const uno::Collection<std::size_t> &get_upper_bounded_variables() const override
        {
            return _upper_bounded_variables_collection;
        }

        const uno::Vector<std::size_t>& get_fixed_variables() const override {
            return _fixed_variables;
        }

        double variable_lower_bound(std::size_t variable_index) const override { 
            return _variable_lower_bounds[variable_index]; 
            }

        double variable_upper_bound(std::size_t variable_index) const override { 
            return _variable_upper_bounds[variable_index]; 
        }

        uno::BoundType get_variable_bound_type(std::size_t variable_index) const override {
            return _variable_status[variable_index];
        }

        double constraint_lower_bound(std::size_t constraint_index) const override {
            return _constraint_lower_bounds[constraint_index];
        }

        double constraint_upper_bound(std::size_t constraint_index) const override {
            return _constraint_upper_bounds[constraint_index];
        }
        
        uno::BoundType get_constraint_bound_type(std::size_t constraint_index) const override {
            return _constraint_status[constraint_index];
        }
        uno::FunctionType get_constraint_type(std::size_t constraint_index) const override {
            return _constraint_type[constraint_index];
        }

        void postprocess_solution([[maybe_unused]]uno::Iterate &iterate, [[maybe_unused]]uno::IterateStatus termination_status) const override {
            
        }

        void initialise_from_data()
        {
            for (std::size_t i = 0; i < number_variables; ++i)
            {
                if (_variable_lower_bounds[i] == _variable_upper_bounds[i])
                {
                    _variable_status[i] = uno::BoundType::EQUAL_BOUNDS;
                    _lower_bounded_variables.emplace_back(i);
                    _upper_bounded_variables.emplace_back(i);
                    _fixed_variables.emplace_back(i);
                }
                else if (std::isfinite(_variable_lower_bounds[i]) && std::isfinite(_variable_upper_bounds[i]))
                {
                    _variable_status[i] = uno::BoundType::BOUNDED_BOTH_SIDES;
                    _lower_bounded_variables.emplace_back(i);
                    _upper_bounded_variables.emplace_back(i);
                }
                else if (std::isfinite(_variable_lower_bounds[i]))
                {
                    _variable_status[i] = uno::BoundType::BOUNDED_LOWER;
                    _lower_bounded_variables.emplace_back(i);
                    _single_lower_bounded_variables.emplace_back(i);
                }
                else if (std::isfinite(_variable_upper_bounds[i]))
                {
                    _variable_status[i] = uno::BoundType::BOUNDED_UPPER;
                    _upper_bounded_variables.emplace_back(i);
                    _single_upper_bounded_variables.emplace_back(i);
                }
                else
                {
                    _variable_status[i] = uno::BoundType::UNBOUNDED;
                }
            }

            for (std::size_t i = 0; i<number_constraints; ++i) {
                if (_constraint_lower_bounds[i] == _constraint_upper_bounds[i])
                {
                    _constraint_status[i] = uno::BoundType::EQUAL_BOUNDS;
                    _equality_constraints.emplace_back(i);
                }
                else if (std::isfinite(_constraint_lower_bounds[i]) && std::isfinite(_constraint_upper_bounds[i]))
                {
                    _constraint_status[i] = uno::BoundType::BOUNDED_BOTH_SIDES;
                    _inequality_constraints.emplace_back(i);
                }
                else if (std::isfinite(_constraint_lower_bounds[i]))
                {
                    _constraint_status[i] = uno::BoundType::BOUNDED_LOWER;
                    _inequality_constraints.emplace_back(i);
                }
                else if (std::isfinite(_constraint_upper_bounds[i]))
                {
                    _constraint_status[i] = uno::BoundType::BOUNDED_UPPER;
                    _inequality_constraints.emplace_back(i);
                }
                else
                {
                    _constraint_status[i] = uno::BoundType::UNBOUNDED;
                }
            }
        }

    };

    class mexModel
        : public unomex::DataModel
    {
        friend class mexCallbacks;
        std::size_t _n{};
        std::size_t _m{};
        
        matlab::data::StructArray _funcs;
        matlab::data::StructArray _varInfo;
        bool _has_intermediate_callback{false};

        public:
        mexModel(std::size_t nVar, std::size_t nCon, matlab::data::StructArray &varInfo, matlab::data::StructArray &funcs) 
            : unomex::DataModel(nVar, nCon, "Matlab Model")
            , _n(nVar)
            , _m(nCon)
            , _funcs(std::move(funcs))
            , _varInfo(std::move(varInfo))
        {
            
            if (utilities::isfield(_varInfo, "lBnds")) {
                matlab::data::TypedArray<double> lBnd = std::move(_varInfo[0]["lBnds"]);
                if (_n != lBnd.getNumberOfElements()) 
                    utilities::error("Lower bounds on x must have {} entries but has {}", _n, lBnd.getNumberOfElements());
                std::copy(lBnd.cbegin(), lBnd.cend(), _variable_lower_bounds.begin());
            } else {
                utilities::error("Field 'lBnds' not supplied.");
            }

            if (utilities::isfield(_varInfo, "uBnds")) {
                matlab::data::TypedArray<double> uBnd = std::move(_varInfo[0]["uBnds"]);
                if (_n != uBnd.getNumberOfElements()) 
                    utilities::error("Upper bounds on x must have {} entries but has {}", _n, uBnd.getNumberOfElements());
                std::copy(uBnd.cbegin(), uBnd.cend(), _variable_upper_bounds.begin());
            } else {
                utilities::error("Field 'uBnds' not supplied.");
            }

            if (utilities::isfield(_varInfo, "clBnds")) {
                matlab::data::TypedArray<double> clBnd = std::move(_varInfo[0]["clBnds"]);
                if (_m != clBnd.getNumberOfElements()) 
                    utilities::error("Lower bounds on constraints must have {} entries but has {}", _m, clBnd.getNumberOfElements());
                std::copy(clBnd.cbegin(), clBnd.cend(), _constraint_lower_bounds.begin());
            } else {
                utilities::error("Field 'clBnds' not supplied.");
            }

            if (utilities::isfield(_varInfo, "cuBnds")) {
                matlab::data::TypedArray<double> cuBnd = std::move(_varInfo[0]["cuBnds"]);
                if (_m != cuBnd.getNumberOfElements()) 
                    utilities::error("Upper bounds on constraints must have {} entries but has {}", _m, cuBnd.getNumberOfElements());
                std::copy(cuBnd.cbegin(), cuBnd.cend(), _constraint_upper_bounds.begin());
            } else {
                utilities::error("Field 'cuBnds' not supplied.");
            }
            
            std::fill(_constraint_type.begin(), _constraint_type.end(), uno::FunctionType::NONLINEAR);
            if (utilities::isfield(_varInfo, "linearities")) {
                matlab::data::TypedArray<double> linearities = std::move(_varInfo[0]["linearities"]);
                for (std::size_t i = 0; i < linearities.getNumberOfElements(); ++i) {
                    _constraint_type[static_cast<std::size_t>(linearities[i])-1] = uno::FunctionType::LINEAR;
                    _linear_constraints.emplace_back(static_cast<std::size_t>(linearities[i])-1);
                }
            }
            
            initialise_from_data();
            
            if (utilities::isfield(_varInfo, "x0")) {
                matlab::data::TypedArray<double> x0 = utilities::getfield(_varInfo, "x0");
                if (_n != x0.getNumberOfElements()) 
                    utilities::error("Initial point x0 must have {} entries but has {}", _n, x0.getNumberOfElements());
            } else {
                utilities::error("Field 'x0' not supplied.");
            }

            if (utilities::isfield(_varInfo, "lambda0")) {
                matlab::data::TypedArray<double> lambda0 = utilities::getfield(_varInfo,"lambda0");
                if (_m != lambda0.getNumberOfElements()) 
                    utilities::error("Initial point lambda0 must have {} entries but has {}", _m, lambda0.getNumberOfElements());
            }

            
            if (utilities::isfield(_funcs, "objective")) {
                matlab::data::Array obj = utilities::getfield(_funcs, "objective");
                if (!utilities::ishandle(obj))
                    goto noobjective;
            } else {
noobjective:
                utilities::error("The objective field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(_funcs, "gradient")) {
                matlab::data::Array g = utilities::getfield(_funcs, "gradient");
                if (!utilities::ishandle(g))
                    goto nogradient;
            }
            else {
nogradient:
                utilities::error("The gradient field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(_funcs, "gradient_nonzeros")) {
                matlab::data::Array gnz = utilities::getfield(_funcs, "gradient_nonzeros");
                if (!utilities::ishandle(gnz))
                    goto nogradnz;
            }
            else {
nogradnz:
                utilities::error("The gradient_nonzeros field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(_funcs, "jacobian")) {
                matlab::data::Array j = utilities::getfield(_funcs, "jacobian");
                if (!utilities::ishandle(j))
                    goto nojac;
            }
            else {
nojac:
                utilities::error("The jacobian field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(_funcs, "jacobian_nonzeros")) {
                matlab::data::Array jnz = utilities::getfield(_funcs, "jacobian_nonzeros");
                if (!utilities::ishandle(jnz))
                    goto nojacnz;
            }
            else {
nojacnz:
                utilities::error("The jacobian_nonzeros field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(_funcs, "constraints")) {
                matlab::data::Array c = utilities::getfield(_funcs, "constraints");
                if (!utilities::ishandle(c))
                    goto nocons;
            }
            else {
nocons:
                utilities::error("The constraints field on funcs must be a function handle taking one vector intput");
            }

                
            if (utilities::isfield(_funcs, "hessian")) {
                matlab::data::Array h = utilities::getfield(_funcs, "hessian");
                if (!utilities::ishandle(h))
                    goto nohes;
            }
            else {
nohes:
                utilities::error("The hessian field on funcs must be a function handle taking x, sigma and lambda as inputs");
            }
            
            if (utilities::isfield(_funcs, "hessian_nonzeros")) {
                matlab::data::Array hnz = utilities::getfield(_funcs, "hessian_nonzeros");
                if (!utilities::ishandle(hnz))
                    goto nohesnz;
            }
            else {
nohesnz:
                utilities::error("The hessian_nonzeros field on funcs must be a function handle taking x, sigma and lambda as inputs");
            }
        }
        

        double evaluate_objective(const uno::Vector<double> &x) const override { 
            matlab::data::ArrayDimensions dims = {_n,1};
            matlab::data::ArrayFactory f;
            matlab::data::TypedArray<double> _x = f.createArray<double>(dims);
            std::copy(x.begin(), x.end(), _x.begin());
            auto retval = utilities::feval(_funcs[0]["objective"], 1, {_x});
            return utilities::getscalar<double>(retval[0]);
        }

        void evaluate_objective_gradient(const uno::Vector<double> &x, uno::SparseVector<double> &gradient) const override
        {
            matlab::data::ArrayDimensions dims = {_n,1};
            matlab::data::ArrayFactory f;
            matlab::data::TypedArray<double> _x = f.createArray<double>(dims);
            std::copy(x.begin(), x.end(), _x.begin());
            auto retval = utilities::feval(_funcs[0]["gradient"], 1, {_x});

            matlab::data::SparseArray<double> grad = std::move(retval[0]);
            matlab::data::SparseIndex idx;
            for (auto it = grad.cbegin(); it != grad.cend(); ++it) 
            {
                idx = grad.getIndex(it);
                gradient.insert(idx.first, *it);
            }
            
        }

        void evaluate_constraints(const uno::Vector<double> &x, std::vector<double> &constraints) const override
        {
            matlab::data::ArrayDimensions dims = {_n,1};
            matlab::data::ArrayFactory f;
            matlab::data::TypedArray<double> _x = f.createArray<double>(dims);
            std::copy(x.begin(), x.end(), _x.begin());
            auto retval = utilities::feval(_funcs[0]["constraints"], 1, {_x});
            matlab::data::TypedArray<double> cons = std::move(retval[0]);
            std::copy(cons.begin(), cons.end(), constraints.begin());
        }

        void evaluate_constraint_gradient(const uno::Vector<double> &x, size_t constraint_index, uno::SparseVector<double> &gradient) const override
        {
            matlab::data::ArrayDimensions dims = {_n,1};
            matlab::data::ArrayFactory f;
            matlab::data::TypedArray<double> _x = f.createArray<double>(dims);
            std::copy(x.begin(), x.end(), _x.begin());
            auto retval = utilities::feval(_funcs[0]["jacobian"], 1, {_x});
            matlab::data::SparseArray<double> jacobian = std::move(retval[0]);
            matlab::data::SparseIndex idx;
            
            for (auto it = jacobian.cbegin(); it != jacobian.cend(); ++it) 
            {
                idx = jacobian.getIndex(it);
                if (idx.first == constraint_index)
                    gradient.insert(idx.second, *it);
            }
        }
        void evaluate_constraint_jacobian(const uno::Vector<double> &x, uno::RectangularMatrix<double> &constraint_jacobian) const override
        {
            matlab::data::ArrayDimensions dims = {_n,1};
            matlab::data::ArrayFactory f;
            matlab::data::TypedArray<double> _x = f.createArray<double>(dims);
            std::copy(x.begin(), x.end(), _x.begin());
            auto retval = utilities::feval(_funcs[0]["jacobian"], 1, {_x});
            matlab::data::SparseArray<double> jacobian = std::move(retval[0]);
            matlab::data::SparseIndex idx;
            for (auto it = jacobian.cbegin(); it != jacobian.cend(); ++it) 
            {
                idx = jacobian.getIndex(it);
                constraint_jacobian[idx.first].insert(idx.second, *it);
            }
            
        }
        void evaluate_lagrangian_hessian(const uno::Vector<double> &x, double objective_multiplier, const uno::Vector<double> &multipliers,
                                         uno::SymmetricMatrix<size_t, double> &hessian) const override
        {
            matlab::data::ArrayFactory f;
            matlab::data::TypedArray<double> _x = f.createArray<double>({_n,1});
            matlab::data::TypedArray<double> _sigma = f.createScalar(objective_multiplier);
            matlab::data::TypedArray<double> _lambda = f.createArray<double>({_m,1});
            std::copy(x.begin(), x.end(), _x.begin());
            std::copy(multipliers.begin(), multipliers.end(), _lambda.begin());

            auto retval = utilities::feval(_funcs[0]["hessian"], 1, {_x, _sigma, _lambda});
            
            matlab::data::SparseArray<double> hess = std::move(retval[0]);

            hessian.reset();
            matlab::data::SparseIndex idx;
            std::size_t jCol = hess.getIndex(hess.cbegin()).second;

            for (auto it = hess.cbegin(); it != hess.cend(); ++it) 
            {
                idx = hess.getIndex(it);
                if (idx.second != jCol)
                {
                    hessian.finalize_column(jCol);
                    jCol = idx.second;
                }

                hessian.insert(*it, idx.first, idx.second);
            }
            hessian.finalize_column(jCol);

        }

        void initial_primal_point(uno::Vector<double> &x) const override {
            matlab::data::TypedArray<double> _x0 = std::move(_varInfo[0]["x0"]);
            std::copy(_x0.cbegin(), _x0.cend(), x.begin());
        }

        void initial_dual_point(uno::Vector<double> &multipliers) const override {
            if (utilities::isfield(_varInfo, "lambda0")) {
                matlab::data::TypedArray<double> _lambda0 = std::move(_varInfo[0]["lambda0"]);
                std::copy(_lambda0.cbegin(), _lambda0.cend(), multipliers.begin());
            } else {
                std::fill(multipliers.begin(), multipliers.end(), 0.);
            }
        }

        std::size_t number_objective_gradient_nonzeros() const override { 
            if (utilities::isfield(_funcs, "gradient_nonzeros")){
                auto retval = utilities::feval(_funcs[0]["gradient_nonzeros"], 1, {});
                return static_cast<std::size_t>(utilities::getscalar<double>(retval[0]));
            } else {
                return _n;
            }
        }

        std::size_t number_jacobian_nonzeros() const override {    
            auto retval = utilities::feval(_funcs[0]["jacobian_nonzeros"], 1, {});
            return static_cast<std::size_t>(utilities::getscalar<double>(retval[0]));
        }

        std::size_t number_hessian_nonzeros() const override { 
            auto retval = utilities::feval(_funcs[0]["hessian_nonzeros"], 1, {});
            return static_cast<std::size_t>(utilities::getscalar<double>(retval[0]));
        }

    };


    class mexCallbacks
        : public uno::UserCallbacks
    {
        matlab::data::StructArray _callbacks;
        bool _has_acceptable_iterate_callback{false};
        bool _has_new_primals_callback{false};
        bool _has_new_multipliers_callback{false};
        public:
        mexCallbacks(matlab::data::StructArray &callbacks) : _callbacks(std::move(callbacks)) {
            if (utilities::isfield(_callbacks, "acceptable_iterate_callback")) {
                matlab::data::Array icb = utilities::getfield(_callbacks, "acceptable_iterate_callback");
                if (!utilities::ishandle(icb))
                    utilities::error("The acceptable_iterate_callback field on funcs must be a function handle taking x, sigma and lambda as inputs");
                _has_acceptable_iterate_callback = true;
            }
            
            if (utilities::isfield(_callbacks, "new_primals_callback")) {
                matlab::data::Array npc = utilities::getfield(_callbacks, "new_primals_callback");
                if (!utilities::ishandle(npc))
                    utilities::error("The new_primals_callback field on funcs must be a function handle taking x, sigma and lambda as inputs");
                _has_new_primals_callback = true;
            }

            if (utilities::isfield(_callbacks, "new_multipliers_callback")) {
                matlab::data::Array nmc = utilities::getfield(_callbacks, "new_multipliers_callback");
                if (!utilities::ishandle(nmc))
                    utilities::error("The new_multipliers_callback field on funcs must be a function handle taking x, sigma and lambda as inputs");
                _has_new_multipliers_callback = true;
            }
        }
            

        void notify_acceptable_iterate(const uno::Vector<double>& primals, const uno::Multipliers& multipliers, double objective_multiplier) override {
            
            if (_has_acceptable_iterate_callback) {
                
                matlab::data::ArrayDimensions dimsX = {primals.size(),1};
                matlab::data::ArrayDimensions dimsLambda = {multipliers.constraints.size(),1};
                matlab::data::ArrayFactory f;
                matlab::data::TypedArray<double> _x = f.createArray<double>(dimsX);
                matlab::data::TypedArray<double> _lambda = f.createArray<double>(dimsLambda);            
                std::copy(primals.begin(), primals.end(), _x.begin());
                std::copy(multipliers.constraints.begin(), multipliers.constraints.end(), _lambda.begin());
                utilities::feval(_callbacks[0]["acceptable_iterate_callback"], 0, {_x, _lambda});
            }
        }
        void notify_new_primals(const uno::Vector<double>& primals) override {
            if (_has_new_primals_callback) {
                matlab::data::ArrayDimensions dimsX = {primals.size(),1};
                matlab::data::ArrayFactory f;
                matlab::data::TypedArray<double> _x = f.createArray<double>(dimsX);
                std::copy(primals.begin(), primals.end(), _x.begin());
                utilities::feval(_callbacks[0]["new_primals_callback"], 0, {_x});
            }
        }
        
        void notify_new_multipliers(const uno::Multipliers& multipliers) override {
            if (_has_new_multipliers_callback) {
                matlab::data::ArrayDimensions dimsLambda = {multipliers.constraints.size(),1};
                matlab::data::ArrayFactory f;
                matlab::data::TypedArray<double> _lambda = f.createArray<double>(dimsLambda);            
                std::copy(multipliers.constraints.begin(), multipliers.constraints.end(), _lambda.begin());
                utilities::feval(_callbacks[0]["new_multipliers_callback"], 0, {_lambda});
            }
        }
    };
} // namespace unomex