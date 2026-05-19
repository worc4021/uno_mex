#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "MatlabDataArray.hpp"
#include "mex.hpp"
#include "sparse.hpp"
#include "Uno_C_API.h"

namespace unomex {

struct MexCallbackContext {
   matlab::data::StructArray callbacks;
   bool has_acceptable_iterate{false};
   bool has_new_primals{false};
   bool has_new_multipliers{false};
};

class MexProblemContext {
public:
   std::size_t number_variables{};
   std::size_t number_constraints{};
   matlab::data::StructArray funcs;
   matlab::data::StructArray var_info;
   std::vector<double> x0;
   std::vector<double> variable_lower_bounds;
   std::vector<double> variable_upper_bounds;
   std::vector<double> constraint_lower_bounds;
   std::vector<double> constraint_upper_bounds;
   utilities::Sparse<double> jacobian_sparsity;
   std::optional<utilities::Sparse<double>> hessian_sparsity;
   std::vector<uno_int> jacobian_row;
   std::vector<uno_int> jacobian_col;
   std::vector<uno_int> hessian_row;
   std::vector<uno_int> hessian_col;

   MexProblemContext(std::size_t n_var, std::size_t n_con, matlab::data::StructArray var_info_in,
      matlab::data::StructArray funcs_in);

   void validate_handles() const;
   void build_sparsity_patterns();
};

const char* optimization_status_message(uno_int status);

void* build_uno_model(MexProblemContext& context);
void apply_solver_options(void* solver, const matlab::data::StructArray& options);
void validate_solver_options(void* solver);
void apply_solver_callbacks(void* solver, MexCallbackContext& callbacks);

// Route Uno log output to the MATLAB command window for the lifetime of this guard.
class MatlabLoggerGuard {
public:
   MatlabLoggerGuard();
   ~MatlabLoggerGuard();
   MatlabLoggerGuard(const MatlabLoggerGuard&) = delete;
   MatlabLoggerGuard& operator=(const MatlabLoggerGuard&) = delete;
};
matlab::data::StructArray pack_solution(void* solver, std::size_t number_variables, std::size_t number_constraints,
   matlab::data::ArrayFactory& factory);

matlab::data::StructArray export_solver_options(void* solver, matlab::data::ArrayFactory& factory);

} // namespace unomex
