#include "mex.hpp"
#include "mexAdapter.hpp"
#include "utilities.hpp"
#include "Uno_C_API.h"
#include "mex_problem.hpp"

class MexFunction : public matlab::mex::Function {
   matlab::data::ArrayFactory factory;

public:
   MexFunction() {
      matlabPtr = getEngine();
   }

   void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
      void* solver = uno_create_solver();
      if (solver == nullptr) {
         utilities::errWithId("cApiFailure", "uno_create_solver failed.");
      }

      if (inputs.size() > 0) {
         if (!utilities::isstring(inputs[0])) {
            uno_destroy_solver(solver);
            utilities::errWithId("invalidInput", "Pass a string with the preset name.");
         }
         const std::string preset = utilities::getstringvalue(inputs[0]);
         if (!uno_set_solver_preset(solver, preset.c_str())) {
            uno_destroy_solver(solver);
            utilities::errWithId("invalidPreset", "Failed to set preset '{}'.", preset);
         }
      }

      outputs[0] = unomex::export_solver_options(solver, factory);
      uno_destroy_solver(solver);
   }
};

#include "mex_problem.cpp"
