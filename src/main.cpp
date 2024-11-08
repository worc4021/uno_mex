#include "mex.hpp"
#include "mexAdapter.hpp"
#include "utilities.hpp"
#include "ingredients/globalization_mechanisms/GlobalizationMechanism.hpp"
#include "ingredients/globalization_mechanisms/GlobalizationMechanismFactory.hpp"
#include "ingredients/constraint_relaxation_strategies/ConstraintRelaxationStrategy.hpp"
#include "ingredients/constraint_relaxation_strategies/ConstraintRelaxationStrategyFactory.hpp"
#include "Uno.hpp"
#include "model/ModelFactory.hpp"
#include "options/Options.hpp"
#include "options/DefaultOptions.hpp"
#include "tools/Logger.hpp"
#include "nlp.hpp"

class MexFunction 
    : public matlab::mex::Function 
{

public:
    MexFunction()  {
        matlabPtr = getEngine();
    }
    ~MexFunction() = default;
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        

        if (inputs.size() != 1 || !utilities::isstruct(inputs[0]))
            utilities::error("Pass a struct with the fields 'variableInfo', 'funcs' and 'options'");
        
        matlab::data::StructArray problem = std::move(inputs[0]);
        
        if (!utilities::isfield(problem, "variableInfo"))
            utilities::error("Field 'variableInfo' not supplied.");
        if (!utilities::isfield(problem, "funcs"))
            utilities::error("Field 'funcs' not supplied.");
        if (!utilities::isfield(problem, "options"))
            utilities::warning("Field 'options' not supplied.");


        matlab::data::StructArray varInfo = std::move(problem[0]["variableInfo"]);
        matlab::data::StructArray funcs = std::move(problem[0]["funcs"]);

        if (!utilities::isfield(varInfo, "x0"))
            utilities::error("Field 'x0' not supplied.");
        
        std::size_t nVar = varInfo[0]["x0"].getNumberOfElements();

        if (!utilities::isfield(varInfo,"clBnds"))
            utilities::error("Lower bound on constraints not supplied.");
        
        std::size_t nCon = varInfo[0]["clBnds"].getNumberOfElements();

        
        uno::Options options = uno::DefaultOptions::load();
        uno::Options solvers_options = uno::DefaultOptions::determine_solvers_and_preset();
        if (utilities::isfield(problem, "options")) {
            matlab::data::StructArray opts = std::move(problem[0]["options"]);
            for (const auto &field : opts.getFieldNames()) {
                solvers_options[field] = utilities::getstringvalue(opts[0][field]);
            }
        }
        options.overwrite_with(solvers_options);

        utilities::printf("Creating model with {} variables and {} constraints\n", nVar, nCon);
        std::unique_ptr<unomex::mexModel> mex_model = std::make_unique<unomex::mexModel>(nVar,nCon,varInfo,funcs);
        
        std::unique_ptr<uno::Model> model = uno::ModelFactory::reformulate(std::move(mex_model), options);
        
        uno::Iterate initial_iterate(model->number_variables, model->number_constraints);
        model->initial_primal_point(initial_iterate.primals);
        model->project_onto_variable_bounds(initial_iterate.primals);
        model->initial_dual_point(initial_iterate.multipliers.constraints);
        initial_iterate.feasibility_multipliers.reset();

        auto constraint_relaxation_strategy = uno::ConstraintRelaxationStrategyFactory::create(*model, options);
        auto globalization_mechanism = uno::GlobalizationMechanismFactory::create(*constraint_relaxation_strategy, options);
        uno::Uno uno = uno::Uno(*globalization_mechanism, options);

        // // // solve the instance
        uno.solve(*model, initial_iterate, options);
        matlab::data::ArrayFactory factory;
        outputs[0] = factory.createScalar(0);
    }
};