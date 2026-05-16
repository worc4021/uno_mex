#include <exception>

#include "mex.hpp"
#include "mexAdapter.hpp"
#include "utilities.hpp"
#include "Uno_C_API.h"
#include "mex_problem.hpp"

class MexFunction : public matlab::mex::Function {
public:
   MexFunction() {
      matlabPtr = getEngine();
   }

   void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
      if (inputs.size() != 1 || !utilities::isstruct(inputs[0])) {
         utilities::errWithId("invalidInput",
            "Pass a struct with the fields 'variableInfo', 'funcs' and 'options'.");
      }

      matlab::data::StructArray problem = std::move(inputs[0]);
      if (!utilities::isfield(problem, "variableInfo")) {
         utilities::errWithId("missingField", "Field 'variableInfo' not supplied.");
      }
      if (!utilities::isfield(problem, "funcs")) {
         utilities::errWithId("missingField", "Field 'funcs' not supplied.");
      }
      if (!utilities::isfield(problem, "options")) {
         utilities::warnWithId("missingField", "Field 'options' not supplied.");
      }
      if (!utilities::isfield(problem, "callbacks")) {
         utilities::warnWithId("missingField", "Field 'callbacks' not supplied.");
      }

      matlab::data::StructArray var_info = std::move(problem[0]["variableInfo"]);
      matlab::data::StructArray funcs = std::move(problem[0]["funcs"]);
      matlab::data::ArrayFactory factory;
      matlab::data::StructArray callbacks = factory.createStructArray({1, 1}, {});
      if (utilities::isfield(problem, "callbacks")) {
         callbacks = matlab::data::StructArray(problem[0]["callbacks"]);
      }

      const std::size_t n_var = var_info[0]["x0"].getNumberOfElements();
      const std::size_t n_con = var_info[0]["clBnds"].getNumberOfElements();

      utilities::printf("Creating model with {} variables and {} constraints\n", n_var, n_con);

      unomex::MexProblemContext context(n_var, n_con, std::move(var_info), std::move(funcs));
      void* model = unomex::build_uno_model(context);

      void* solver = uno_create_solver();
      if (solver == nullptr) {
         uno_destroy_model(model);
         utilities::errWithId("cApiFailure", "uno_create_solver failed.");
      }

      const unomex::MatlabLoggerGuard matlab_logger;
      if (utilities::isfield(problem, "options")) {
         unomex::apply_solver_options(solver, problem[0]["options"]);
      }
      unomex::validate_solver_options(solver);

      unomex::MexCallbackContext callback_context{std::move(callbacks)};
      if (utilities::isfield(callback_context.callbacks, "acceptable_iterate_callback")) {
         if (!utilities::ishandle(callback_context.callbacks[0]["acceptable_iterate_callback"])) {
            uno_destroy_solver(solver);
            uno_destroy_model(model);
            utilities::errWithId("invalidHandle", "acceptable_iterate_callback must be a function handle.");
         }
         callback_context.has_acceptable_iterate = true;
      }
      if (utilities::isfield(callback_context.callbacks, "new_primals_callback")) {
         callback_context.has_new_primals = utilities::ishandle(callback_context.callbacks[0]["new_primals_callback"]);
      }
      if (utilities::isfield(callback_context.callbacks, "new_multipliers_callback")) {
         callback_context.has_new_multipliers =
            utilities::ishandle(callback_context.callbacks[0]["new_multipliers_callback"]);
      }
      unomex::apply_solver_callbacks(solver, callback_context);

      try {
         uno_optimize(solver, model);
      }
      catch (const std::exception& ex) {
         uno_destroy_solver(solver);
         uno_destroy_model(model);
         utilities::errWithId("solveFailure", "uno_optimize failed: {}", ex.what());
      }
      catch (...) {
         uno_destroy_solver(solver);
         uno_destroy_model(model);
         utilities::errWithId("solveFailure", "uno_optimize failed with an unknown exception.");
      }

      outputs[0] = unomex::pack_solution(solver, n_var, n_con, factory);

      uno_destroy_solver(solver);
      uno_destroy_model(model);
   }
};

#include "mex_problem.cpp"
#include "mex_problem_mex.cpp"
