#pragma once

#include <vector>
#include <span>
#include <limits>
#include "model/Model.hpp"
#include "linear_algebra/SparseVector.hpp"
#include "linear_algebra/Vector.hpp"
#include "linear_algebra/RectangularMatrix.hpp"
#include "linear_algebra/SymmetricMatrix.hpp"
#include "optimization/Multipliers.hpp"
#include "symbolic/CollectionAdapter.hpp"

namespace unomex
{

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
            : uno::Model(name, number_variables, number_constraints, 1.),
              _variable_lower_bounds(number_variables),
              _variable_upper_bounds(number_variables),
              _constraint_lower_bounds(number_constraints),
              _constraint_upper_bounds(number_constraints),
              _variable_status(number_variables),
              _constraint_type(number_constraints),
              _constraint_status(number_constraints),
              _linear_constraints(0),
              _slacks(0),
              _equality_constraints(0),
              _inequality_constraints(0),
              _equality_constraints_collection(_equality_constraints),
              _inequality_constraints_collection(_inequality_constraints),
              _lower_bounded_variables(0),
              _lower_bounded_variables_collection(_lower_bounded_variables),
              _upper_bounded_variables(0),
              _upper_bounded_variables_collection(_upper_bounded_variables),
              _single_lower_bounded_variables(0),
              _single_lower_bounded_variables_collection(_single_lower_bounded_variables),
              _single_upper_bounded_variables(0),
              _fixed_variables(0),
              _single_upper_bounded_variables_collection(_single_upper_bounded_variables),
              _linear_constraints_collection(_linear_constraints) {}

        virtual ~DataModel() override = default;

        const uno::Collection<size_t> &get_equality_constraints() const override
        {
            return _equality_constraints_collection;
        }

        const uno::Collection<size_t> &get_inequality_constraints() const override
        {
            return _inequality_constraints_collection;
        }

        const uno::Collection<size_t> &get_linear_constraints() const override
        {
            return _linear_constraints_collection;
        }

        const uno::SparseVector<size_t> &get_slacks() const override
        {
            return _slacks;
        }

        const uno::Collection<size_t> &get_single_lower_bounded_variables() const override
        {
            return _single_lower_bounded_variables_collection;
        }

        const uno::Collection<size_t> &get_single_upper_bounded_variables() const override
        {
            return _single_upper_bounded_variables_collection;
        }

        const uno::Collection<size_t> &get_lower_bounded_variables() const override
        {
            return _lower_bounded_variables_collection;
        }

        const uno::Collection<size_t> &get_upper_bounded_variables() const override
        {
            return _upper_bounded_variables_collection;
        }

        const uno::Vector<size_t>& get_fixed_variables() const override {
            return _fixed_variables;
        }

        double variable_lower_bound(size_t variable_index) const override { 
            return _variable_lower_bounds[variable_index]; 
            }

        double variable_upper_bound(size_t variable_index) const override { 
            return _variable_upper_bounds[variable_index]; 
        }

        uno::BoundType get_variable_bound_type(size_t variable_index) const override {
            return _variable_status[variable_index];
        }

        double constraint_lower_bound(size_t constraint_index) const override {
            return _constraint_lower_bounds[constraint_index];
        }

        double constraint_upper_bound(size_t constraint_index) const override {
            return _constraint_upper_bounds[constraint_index];
        }
        
        uno::BoundType get_constraint_bound_type(size_t constraint_index) const override {
            return _constraint_status[constraint_index];
        }
        uno::FunctionType get_constraint_type(size_t constraint_index) const override {
            return _constraint_type[constraint_index];
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
        std::size_t _n{};
        std::size_t _m{};
        
        matlab::data::TypedArray<double> _x0;
        matlab::data::TypedArray<double> _lambda0;
        matlab::data::StructArray _funcs;

        matlab::data::ArrayFactory factory;
        
        public:
        mexModel(std::size_t nVar, std::size_t nCon) 
            : unomex::DataModel(nVar, nCon, "Matlab Model")
            , _n(nVar)
            , _m(nCon)
            , _x0(factory.createArray<double>({nVar, 1}))
            , _lambda0(factory.createArray<double>({nCon, 1}))
            , _funcs(factory.createStructArray({0, 0},{}))
        {
            
        }
        
        void setVariableInfo(matlab::data::StructArray &varInfo) {
            if (!utilities::isfield(varInfo, "lBnds") || _n != varInfo[0]["lBnds"].getNumberOfElements()) {
                utilities::error("Lower bounds on x must have {} entries", _n);
            }
            if (!utilities::isfield(varInfo, "uBnds") || _n != varInfo[0]["uBnds"].getNumberOfElements()) {
                utilities::error("Upper bounds on x must have {} entries", _n);
            }
            if (!utilities::isfield(varInfo, "x0") || _n != varInfo[0]["x0"].getNumberOfElements()) {
                utilities::error("Initial point x0 must have {} entries", _n);
            }
            if (!utilities::isfield(varInfo, "clBnds") || _m != varInfo[0]["clBnds"].getNumberOfElements()) {
                utilities::error("Lower bound on constraints must have {} entries", _m);
            }
            if (!utilities::isfield(varInfo, "cuBnds") || _m != varInfo[0]["cuBnds"].getNumberOfElements()) {
                utilities::error("Upper bound on constraints must have {} entries", _m);
            }
            
            matlab::data::TypedArray<double> lBnds = std::move(varInfo[0]["lBnds"]);
            matlab::data::TypedArray<double> uBnds = std::move(varInfo[0]["uBnds"]);
            matlab::data::TypedArray<double> x0 = std::move(varInfo[0]["x0"]);
            matlab::data::TypedArray<double> clBnds = std::move(varInfo[0]["clBnds"]);
            matlab::data::TypedArray<double> cuBnds = std::move(varInfo[0]["cuBnds"]);

            std::copy(lBnds.cbegin(), lBnds.cend(), _variable_lower_bounds.begin());
            std::copy(uBnds.cbegin(), uBnds.cend(), _variable_upper_bounds.begin());
            std::copy(x0.cbegin(), x0.cend(), _x0.begin());
            std::copy(clBnds.cbegin(), clBnds.cend(), _constraint_lower_bounds.begin());
            std::copy(cuBnds.cbegin(), cuBnds.cend(), _constraint_upper_bounds.begin());

            std::fill(_constraint_type.begin(), _constraint_type.end(), uno::FunctionType::NONLINEAR);

            if (utilities::isfield(varInfo, "linearities")) {
                matlab::data::TypedArray<double> linearities = std::move(varInfo[0]["linearities"]);
                for (std::size_t i = 0; i < linearities.getNumberOfElements(); ++i) {
                    _constraint_type[static_cast<std::size_t>(linearities[i])-1] = uno::FunctionType::LINEAR;
                    _linear_constraints.emplace_back(static_cast<std::size_t>(linearities[i])-1);
                }
            }

            if (utilities::isfield(varInfo, "lambda0")) {
                matlab::data::TypedArray<double> lambda0 = std::move(varInfo[0]["lambda0"]);
                std::copy(lambda0.cbegin(), lambda0.cend(), _lambda0.begin());
            }
            else
                std::fill(_lambda0.begin(), _lambda0.end(), 0.0);
            
            initialise_from_data();
        }

        void setFunctionHandles(matlab::data::StructArray &funcs) {
            if (utilities::isfield(funcs, "objective")) {
            matlab::data::Array obj = utilities::getfield(funcs, "objective");
            if (!utilities::ishandle(obj))
                goto noobjective;
            }
            else {
noobjective:
                utilities::error("The objective field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(funcs, "gradient")) {
                matlab::data::Array g = utilities::getfield(funcs, "gradient");
                if (!utilities::ishandle(g))
                    goto nogradient;
            }
            else {
nogradient:
                utilities::error("The gradient field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(funcs, "gradient_nonzeros")) {
                matlab::data::Array gnz = utilities::getfield(funcs, "gradient_nonzeros");
                if (!utilities::ishandle(gnz))
                    goto nogradnz;
            }
            else {
nogradnz:
                utilities::error("The gradient_nonzeros field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(funcs, "jacobian")) {
                matlab::data::Array j = utilities::getfield(funcs, "jacobian");
                if (!utilities::ishandle(j))
                    goto nojac;
            }
            else {
nojac:
                utilities::error("The jacobian field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(funcs, "jacobian_nonzeros")) {
                matlab::data::Array jnz = utilities::getfield(funcs, "jacobian_nonzeros");
                if (!utilities::ishandle(jnz))
                    goto nojacnz;
            }
            else {
nojacnz:
                utilities::error("The jacobian_nonzeros field on funcs must be a function handle taking one vector intput");
            }

            if (utilities::isfield(funcs, "constraints")) {
                matlab::data::Array c = utilities::getfield(funcs, "constraints");
                if (!utilities::ishandle(c))
                    goto nocons;
            }
            else {
nocons:
                utilities::error("The constraints field on funcs must be a function handle taking one vector intput");
            }

                
            if (utilities::isfield(funcs, "hessian")) {
                matlab::data::Array h = utilities::getfield(funcs, "hessian");
                if (!utilities::ishandle(h))
                    goto nohes;
            }
            else {
nohes:
                utilities::error("The hessian field on funcs must be a function handle taking x, sigma and lambda as inputs");
            }
            
            if (utilities::isfield(funcs, "hessian_nonzeros")) {
                matlab::data::Array hnz = utilities::getfield(funcs, "hessian_nonzeros");
                if (!utilities::ishandle(hnz))
                    goto nohesnz;
            }
            else {
nohesnz:
                utilities::error("The hessian_nonzeros field on funcs must be a function handle taking x, sigma and lambda as inputs");
            }

            _funcs = std::move(funcs);
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
            std::copy(_x0.cbegin(), _x0.cend(), x.begin());
        }

        void initial_dual_point(uno::Vector<double> &multipliers) const override {
            std::copy(_lambda0.begin(), _lambda0.end(),multipliers.begin());
        }

        void postprocess_solution([[maybe_unused]]uno::Iterate &iterate, [[maybe_unused]]uno::TerminationStatus termination_status) const override {

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
} // namespace unomex