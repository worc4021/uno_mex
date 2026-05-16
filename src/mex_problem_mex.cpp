// Included only from main.cpp (uno_mex). Requires Uno C++ headers under uno/ (not in the install prefix).
#include "mex_problem.hpp"

#include <string_view>

#include "utilities.hpp"

#include "ingredients/subproblem_solvers/LPSolverFactory.hpp"
#include "ingredients/subproblem_solvers/QPSolverFactory.hpp"
#include "ingredients/subproblem_solvers/SymmetricIndefiniteLinearSolverFactory.hpp"
#include "tools/Logger.hpp"

namespace unomex {

namespace {

#ifdef __cplusplus
extern "C" {
#endif
uno_int matlab_logger_stream_callback(const char* buffer, uno_int length, void* /*user_data*/) {
   if (length <= 0) {
      return 0;
   }
   utilities::printf("{}", std::string_view(buffer, static_cast<std::size_t>(length)));
   return length;
}
#ifdef __cplusplus
}
#endif

template<typename Allowed>
void require_allowed_string_option(const char* option_name, const std::string& value, const Allowed& allowed) {
   for (const auto& choice : allowed) {
      if (value == choice) {
         return;
      }
   }
   std::string allowed_list;
   for (const auto& choice : allowed) {
      if (!allowed_list.empty()) {
         allowed_list.append(", ");
      }
      allowed_list.append(choice);
   }
   utilities::errWithId("invalidOption", "Option '{}' value '{}' is invalid. Allowed values: {}.", option_name, value,
      allowed_list);
}

} // namespace

void validate_solver_options(void* solver) {
   if (uno_get_solver_option_type(solver, "logger") != UNO_OPTION_TYPE_NOT_FOUND) {
      try {
         uno::Logger::set_logger(uno_get_solver_string_option(solver, "logger"));
      }
      catch (const std::exception& exception) {
         utilities::errWithId("invalidOption", "{}", exception.what());
      }
   }

   if (uno_get_solver_option_type(solver, "linear_solver") != UNO_OPTION_TYPE_NOT_FOUND) {
      require_allowed_string_option("linear_solver", uno_get_solver_string_option(solver, "linear_solver"),
         uno::SymmetricIndefiniteLinearSolverFactory::available_solvers());
   }

   if (uno_get_solver_option_type(solver, "QP_solver") != UNO_OPTION_TYPE_NOT_FOUND) {
      require_allowed_string_option("QP_solver", uno_get_solver_string_option(solver, "QP_solver"),
         uno::QPSolverFactory::available_solvers);
   }

   if (uno_get_solver_option_type(solver, "LP_solver") != UNO_OPTION_TYPE_NOT_FOUND) {
      require_allowed_string_option("LP_solver", uno_get_solver_string_option(solver, "LP_solver"),
         uno::LPSolverFactory::available_solvers);
   }

   static constexpr const char* kHessianModels[] = {"exact", "LBFGS", "LSR1", "identity", "zero"};
   if (uno_get_solver_option_type(solver, "hessian_model") != UNO_OPTION_TYPE_NOT_FOUND) {
      require_allowed_string_option("hessian_model", uno_get_solver_string_option(solver, "hessian_model"), kHessianModels);
   }

   static constexpr const char* kInequalityHandlingMethods[] = {"inequality_constrained", "interior_point"};
   if (uno_get_solver_option_type(solver, "inequality_handling_method") != UNO_OPTION_TYPE_NOT_FOUND) {
      require_allowed_string_option("inequality_handling_method",
         uno_get_solver_string_option(solver, "inequality_handling_method"), kInequalityHandlingMethods);
   }

   static constexpr const char* kBarrierFunctions[] = {"log"};
   if (uno_get_solver_option_type(solver, "barrier_function") != UNO_OPTION_TYPE_NOT_FOUND) {
      require_allowed_string_option("barrier_function", uno_get_solver_string_option(solver, "barrier_function"),
         kBarrierFunctions);
   }

   static constexpr const char* kInertiaCorrectionStrategies[] = {"none", "primal", "primal_dual"};
   if (uno_get_solver_option_type(solver, "inertia_correction_strategy") != UNO_OPTION_TYPE_NOT_FOUND) {
      require_allowed_string_option("inertia_correction_strategy",
         uno_get_solver_string_option(solver, "inertia_correction_strategy"), kInertiaCorrectionStrategies);
   }
}

MatlabLoggerGuard::MatlabLoggerGuard() {
   if (!uno_set_logger_stream_callback(matlab_logger_stream_callback, nullptr)) {
      utilities::errWithId("cApiFailure", "uno_set_logger_stream_callback failed.");
   }
}

MatlabLoggerGuard::~MatlabLoggerGuard() {
   uno_reset_logger_stream();
}

} // namespace unomex
