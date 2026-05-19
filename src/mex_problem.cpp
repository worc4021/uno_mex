#include "mex_problem.hpp"

#include <algorithm>
#include <cstring>
#include <span>
#include <stdexcept>

#include "mex.hpp"
#include "utilities.hpp"
#include "uno_option_names.hpp"

namespace unomex {

namespace {

matlab::data::TypedArray<double> make_x_vector(const double* x, std::size_t n) {
   matlab::data::ArrayFactory factory;
   matlab::data::TypedArray<double> mx = factory.createArray<double>({n, 1});
   std::copy(x, x + n, mx.begin());
   return mx;
}

void fill_coo_from_sparse(const utilities::Sparse<double>& sparse, std::vector<uno_int>& row, std::vector<uno_int>& col) {
   const std::size_t nnz = sparse.getNumberOfNonZeroElements();
   row.resize(nnz);
   col.resize(nnz);
   if (nnz == 0) {
      return;
   }
   std::vector<std::size_t> rows(nnz);
   std::vector<std::size_t> cols(nnz);
   sparse.iRow(std::span(rows));
   sparse.jCol(std::span(cols));
   for (std::size_t k = 0; k < nnz; ++k) {
      row[k] = static_cast<uno_int>(rows[k]);
      col[k] = static_cast<uno_int>(cols[k]);
   }
}

void emplace_lower_triangle_pattern(std::optional<utilities::Sparse<double>>& sparse, const matlab::data::Array& array) {
   if (!utilities::issparse(array)) {
      utilities::errWithId("expectedSparse", "Expected a sparse matrix.");
   }
   utilities::Sparse<double> full;
   full.set(array);
   const std::size_t m = full.getNumberOfRows();
   const std::size_t n = full.getNumberOfColumns();
   const std::size_t nnz = full.getNumberOfNonZeroElements();
   std::vector<std::size_t> rows;
   std::vector<std::size_t> cols;
   std::vector<double> values;
   rows.reserve(nnz);
   cols.reserve(nnz);
   values.reserve(nnz);
   if (nnz > 0) {
      std::vector<std::size_t> all_rows(nnz);
      std::vector<std::size_t> all_cols(nnz);
      std::vector<double> all_values(nnz);
      full.iRow(std::span(all_rows));
      full.jCol(std::span(all_cols));
      full.val(std::span(all_values));
      for (std::size_t k = 0; k < nnz; ++k) {
         if (all_rows[k] >= all_cols[k]) {
            rows.push_back(all_rows[k]);
            cols.push_back(all_cols[k]);
            values.push_back(all_values[k]);
         }
      }
   }
   sparse.emplace(m, n);
   sparse->set(std::span(rows), std::span(cols), std::span(values));
}

void require_sparse(const matlab::data::Array& array, const char* callback_name) {
   if (!utilities::issparse(array)) {
      utilities::errWithId("expectedSparse", "The {} callback must return a sparse matrix.", callback_name);
   }
}

void update_pattern_values(utilities::Sparse<double>& pattern, const matlab::data::Array& array, const char* callback_name) {
   require_sparse(array, callback_name);
   matlab::data::SparseArray<double> sparse(array);
   pattern.updateValues(sparse);
}

void copy_pattern_values(const utilities::Sparse<double>& pattern, double* values) {
   const std::size_t nnz = pattern.getNumberOfNonZeroElements();
   if (nnz == 0) {
      return;
   }
   std::vector<double> buffer(nnz);
   pattern.val(std::span(buffer));
   std::copy(buffer.begin(), buffer.end(), values);
}

void scatter_sparse_into_dense(const matlab::data::Array& array, double* dense, std::size_t length, const char* callback_name) {
   if (utilities::issparse(array)) {
      utilities::Sparse<double> sparse;
      sparse.set(array);
      const std::size_t nnz = sparse.getNumberOfNonZeroElements();
      if (nnz > 0) {
         std::vector<std::size_t> rows(nnz);
         std::vector<double> values(nnz);
         sparse.iRow(std::span(rows));
         sparse.val(std::span(values));
         for (std::size_t k = 0; k < nnz; ++k) {
            if (rows[k] >= length) {
               utilities::errWithId("dimensionMismatch",
                  "The {} callback returned an index outside the primal vector (length {}).", callback_name, length);
            }
            dense[rows[k]] = values[k];
         }
      }
      return;
   }
   if (!utilities::isvector(array)) {
      utilities::errWithId("expectedVector", "The {} callback must return a sparse or dense vector.", callback_name);
   }
   matlab::data::TypedArray<double> dense_array(array);
   if (dense_array.getNumberOfElements() != length) {
      utilities::errWithId("dimensionMismatch", "The {} callback must return {} elements but returned {}.", callback_name,
         length, dense_array.getNumberOfElements());
   }
   std::copy(dense_array.cbegin(), dense_array.cend(), dense);
}

} // namespace

const char* optimization_status_message(uno_int status) {
   switch (status) {
      case UNO_SUCCESS:
         return "Success";
      case UNO_ITERATION_LIMIT:
         return "Iteration limit";
      case UNO_TIME_LIMIT:
         return "Time limit";
      case UNO_EVALUATION_ERROR:
         return "Evaluation error";
      case UNO_ALGORITHMIC_ERROR:
         return "Algorithmic error";
      case UNO_USER_TERMINATION:
         return "User termination";
      default:
         return "Unknown";
   }
}

MexProblemContext::MexProblemContext(std::size_t n_var, std::size_t n_con, matlab::data::StructArray var_info_in,
   matlab::data::StructArray funcs_in)
   : number_variables(n_var)
   , number_constraints(n_con)
   , funcs(std::move(funcs_in))
   , var_info(std::move(var_info_in)) {
   if (!utilities::isfield(var_info, "x0")) {
      utilities::errWithId("missingField", "Field 'x0' not supplied.");
   }
   matlab::data::TypedArray<double> x0_arr = var_info[0]["x0"];
   if (x0_arr.getNumberOfElements() != number_variables) {
      utilities::errWithId("dimensionMismatch", "Initial point x0 must have {} entries but has {}.", number_variables,
         x0_arr.getNumberOfElements());
   }
   x0.assign(x0_arr.cbegin(), x0_arr.cend());
   matlab::data::TypedArray<double> l_bnds = var_info[0]["lBnds"];
   matlab::data::TypedArray<double> u_bnds = var_info[0]["uBnds"];
   matlab::data::TypedArray<double> cl_bnds = var_info[0]["clBnds"];
   matlab::data::TypedArray<double> cu_bnds = var_info[0]["cuBnds"];
   variable_lower_bounds.assign(l_bnds.cbegin(), l_bnds.cend());
   variable_upper_bounds.assign(u_bnds.cbegin(), u_bnds.cend());
   constraint_lower_bounds.assign(cl_bnds.cbegin(), cl_bnds.cend());
   constraint_upper_bounds.assign(cu_bnds.cbegin(), cu_bnds.cend());
   validate_handles();
   build_sparsity_patterns();
}

void MexProblemContext::validate_handles() const {
   const char* required[] = {"objective", "gradient", "constraints", "jacobian", "jacobian_nonzeros", "hessian",
      "hessian_nonzeros"};
   for (const char* name : required) {
      if (!utilities::isfield(funcs, name)) {
         utilities::errWithId("missingField", "Field '{}' not supplied on funcs.", name);
      }
      if (!utilities::ishandle(utilities::getfield(funcs, name))) {
         utilities::errWithId("invalidHandle", "The {} field on funcs must be a function handle.", name);
      }
   }
   if (!utilities::isfield(var_info, "lBnds")) {
      utilities::errWithId("missingField", "Field 'lBnds' not supplied.");
   }
   if (!utilities::isfield(var_info, "uBnds")) {
      utilities::errWithId("missingField", "Field 'uBnds' not supplied.");
   }
   if (!utilities::isfield(var_info, "clBnds")) {
      utilities::errWithId("missingField", "Lower bound on constraints not supplied.");
   }
   if (!utilities::isfield(var_info, "cuBnds")) {
      utilities::errWithId("missingField", "Upper bound on constraints not supplied.");
   }
   const std::size_t n = number_variables;
   const std::size_t m = number_constraints;
   if (var_info[0]["lBnds"].getNumberOfElements() != n) {
      utilities::errWithId("dimensionMismatch", "Lower bounds on x must have {} entries.", n);
   }
   if (var_info[0]["uBnds"].getNumberOfElements() != n) {
      utilities::errWithId("dimensionMismatch", "Upper bounds on x must have {} entries.", n);
   }
   if (var_info[0]["clBnds"].getNumberOfElements() != m) {
      utilities::errWithId("dimensionMismatch", "Lower bounds on constraints must have {} entries.", m);
   }
   if (var_info[0]["cuBnds"].getNumberOfElements() != m) {
      utilities::errWithId("dimensionMismatch", "Upper bounds on constraints must have {} entries.", m);
   }
   if (utilities::isfield(var_info, "lambda0") && var_info[0]["lambda0"].getNumberOfElements() != m) {
      utilities::errWithId("dimensionMismatch", "Initial point lambda0 must have {} entries.", m);
   }
   if (utilities::isfield(var_info, "linearities")) {
      utilities::warnWithId("ignoredField",
         "Field 'linearities' is ignored; all constraints are treated as nonlinear in the C API path.");
   }
}

void MexProblemContext::build_sparsity_patterns() {
   matlab::data::TypedArray<double> x = make_x_vector(x0.data(), number_variables);
   if (number_constraints > 0) {
      auto jac_out = utilities::feval(funcs[0]["jacobian"], 1, {x});
      require_sparse(jac_out[0], "jacobian");
      jacobian_sparsity.set(jac_out[0]);
      fill_coo_from_sparse(jacobian_sparsity, jacobian_row, jacobian_col);
      const auto expected_nnz = static_cast<std::size_t>(
         utilities::getscalar<double>(utilities::feval(funcs[0]["jacobian_nonzeros"], 1, {})[0]));
      if (jacobian_row.size() != expected_nnz) {
         utilities::errWithId("sparsityMismatch",
            "Jacobian nonzero count from jacobian_nonzeros ({}) does not match pattern at x0 ({}).", expected_nnz,
            jacobian_row.size());
      }
   }
   {
      matlab::data::ArrayFactory factory;
      matlab::data::TypedArray<double> sigma = factory.createScalar(1.);
      matlab::data::TypedArray<double> lambda = factory.createArray<double>({number_constraints, 1});
      std::fill(lambda.begin(), lambda.end(), 0.);
      auto hess_out = utilities::feval(funcs[0]["hessian"], 1, {x, sigma, lambda});
      emplace_lower_triangle_pattern(hessian_sparsity, hess_out[0]);
      fill_coo_from_sparse(*hessian_sparsity, hessian_row, hessian_col);
      const auto expected_nnz = static_cast<std::size_t>(
         utilities::getscalar<double>(utilities::feval(funcs[0]["hessian_nonzeros"], 1, {})[0]));
      if (hessian_row.size() != expected_nnz) {
         utilities::warnWithId("hessianSparsityMismatch",
            "hessian_nonzeros reports {} but the lower-triangle pattern at x0 has {}; using the pattern at x0.",
            expected_nnz, hessian_row.size());
      }
   }
}

#ifdef __cplusplus
extern "C" {
#endif

static uno_int objective_callback(uno_int number_variables, const double* x, double* objective_value, void* user_data) {
   auto* ctx = static_cast<MexProblemContext*>(user_data);
   try {
      matlab::data::TypedArray<double> mx = make_x_vector(x, static_cast<std::size_t>(number_variables));
      auto retval = utilities::feval(ctx->funcs[0]["objective"], 1, {mx});
      *objective_value = utilities::getscalar<double>(retval[0]);
      return 0;
   }
   catch (...) {
      return 1;
   }
}

static uno_int gradient_callback(uno_int number_variables, const double* x, double* gradient, void* user_data) {
   auto* ctx = static_cast<MexProblemContext*>(user_data);
   try {
      const auto n = static_cast<std::size_t>(number_variables);
      std::memset(gradient, 0, n * sizeof(double));
      matlab::data::TypedArray<double> mx = make_x_vector(x, n);
      auto retval = utilities::feval(ctx->funcs[0]["gradient"], 1, {mx});
      scatter_sparse_into_dense(retval[0], gradient, n, "gradient");
      return 0;
   }
   catch (...) {
      return 1;
   }
}

static uno_int constraints_callback(uno_int number_variables, uno_int number_constraints, const double* x,
   double* constraint_values, void* user_data) {
   auto* ctx = static_cast<MexProblemContext*>(user_data);
   try {
      matlab::data::TypedArray<double> mx = make_x_vector(x, static_cast<std::size_t>(number_variables));
      auto retval = utilities::feval(ctx->funcs[0]["constraints"], 1, {mx});
      matlab::data::TypedArray<double> cons = std::move(retval[0]);
      if (cons.getNumberOfElements() != static_cast<std::size_t>(number_constraints)) {
         return 1;
      }
      std::copy(cons.cbegin(), cons.cend(), constraint_values);
      return 0;
   }
   catch (...) {
      return 1;
   }
}

static uno_int jacobian_callback(uno_int /*number_variables*/, uno_int /*number_jacobian_nonzeros*/, const double* x,
   double* jacobian_values, void* user_data) {
   auto* ctx = static_cast<MexProblemContext*>(user_data);
   try {
      matlab::data::TypedArray<double> mx = make_x_vector(x, ctx->number_variables);
      auto retval = utilities::feval(ctx->funcs[0]["jacobian"], 1, {mx});
      update_pattern_values(ctx->jacobian_sparsity, retval[0], "jacobian");
      copy_pattern_values(ctx->jacobian_sparsity, jacobian_values);
      return 0;
   }
   catch (...) {
      return 1;
   }
}

static uno_int hessian_callback(uno_int number_variables, uno_int number_constraints, uno_int /*number_hessian_nonzeros*/,
   const double* x, double objective_multiplier, const double* multipliers, double* hessian_values, void* user_data) {
   auto* ctx = static_cast<MexProblemContext*>(user_data);
   try {
      matlab::data::ArrayFactory factory;
      matlab::data::TypedArray<double> mx = make_x_vector(x, static_cast<std::size_t>(number_variables));
      matlab::data::TypedArray<double> sigma = factory.createScalar(objective_multiplier);
      matlab::data::TypedArray<double> lambda = factory.createArray<double>({static_cast<std::size_t>(number_constraints), 1});
      std::copy(multipliers, multipliers + number_constraints, lambda.begin());
      auto retval = utilities::feval(ctx->funcs[0]["hessian"], 1, {mx, sigma, lambda});
      update_pattern_values(*ctx->hessian_sparsity, retval[0], "hessian");
      copy_pattern_values(*ctx->hessian_sparsity, hessian_values);
      return 0;
   }
   catch (...) {
      return 1;
   }
}

static void notify_acceptable_iterate_callback(uno_int number_variables, uno_int number_constraints, const double* primals,
   const double* /*lower_bound_multipliers*/, const double* /*upper_bound_multipliers*/,
   const double* constraint_multipliers, double /*objective_multiplier*/, double /*primal_feasibility_residual*/,
   double /*stationarity_residual*/, double /*complementarity_residual*/, void* user_data) {
   auto* ctx = static_cast<MexCallbackContext*>(user_data);
   if (!ctx->has_acceptable_iterate) {
      return;
   }
   matlab::data::ArrayFactory factory;
   matlab::data::TypedArray<double> x = factory.createArray<double>({static_cast<std::size_t>(number_variables), 1});
   matlab::data::TypedArray<double> lambda = factory.createArray<double>({static_cast<std::size_t>(number_constraints), 1});
   std::copy(primals, primals + number_variables, x.begin());
   std::copy(constraint_multipliers, constraint_multipliers + number_constraints, lambda.begin());
   utilities::feval(ctx->callbacks[0]["acceptable_iterate_callback"], 0, {x, lambda});
}

#ifdef __cplusplus
}
#endif

void* build_uno_model(MexProblemContext& context) {
   void* model = uno_create_model(UNO_PROBLEM_NONLINEAR, static_cast<uno_int>(context.number_variables),
      context.variable_lower_bounds.data(), context.variable_upper_bounds.data(), UNO_ZERO_BASED_INDEXING);
   if (model == nullptr) {
      utilities::errWithId("cApiFailure", "uno_create_model failed.");
   }
   if (!uno_set_user_data(model, &context)) {
      utilities::errWithId("cApiFailure", "uno_set_user_data failed.");
   }
   if (!uno_set_objective(model, UNO_MINIMIZE, objective_callback, gradient_callback)) {
      utilities::errWithId("cApiFailure", "uno_set_objective failed.");
   }
   if (context.number_constraints > 0) {
      if (!uno_set_constraints(model, static_cast<uno_int>(context.number_constraints), constraints_callback,
            context.constraint_lower_bounds.data(), context.constraint_upper_bounds.data(),
            static_cast<uno_int>(context.jacobian_row.size()), context.jacobian_row.data(),
            context.jacobian_col.data(), jacobian_callback)) {
         utilities::errWithId("cApiFailure", "uno_set_constraints failed.");
      }
   }
   if (context.hessian_sparsity.has_value()) {
      if (!uno_set_lagrangian_hessian(model, static_cast<uno_int>(context.hessian_row.size()), UNO_LOWER_TRIANGLE,
            context.hessian_row.data(), context.hessian_col.data(), hessian_callback)) {
         utilities::errWithId("cApiFailure", "uno_set_lagrangian_hessian failed.");
      }
   }
   if (!uno_set_lagrangian_sign_convention(model, UNO_MULTIPLIER_NEGATIVE)) {
      utilities::errWithId("cApiFailure", "uno_set_lagrangian_sign_convention failed.");
   }
   if (!uno_set_initial_primal_iterate(model, context.x0.data())) {
      utilities::errWithId("cApiFailure", "uno_set_initial_primal_iterate failed.");
   }
   if (utilities::isfield(context.var_info, "lambda0")) {
      matlab::data::TypedArray<double> lambda0 = context.var_info[0]["lambda0"];
      std::vector<double> lambda0_vec(lambda0.cbegin(), lambda0.cend());
      if (!uno_set_initial_dual_iterate(model, lambda0_vec.data())) {
         utilities::errWithId("cApiFailure", "uno_set_initial_dual_iterate failed.");
      }
   }
   return model;
}

static bool set_option_from_matlab(void* solver, const std::string& name, const matlab::data::Array& value) {
   if (name == "preset") {
      if (!utilities::isstring(value)) {
         utilities::errWithId("invalidOption", "Option 'preset' must be a string.");
      }
      return uno_set_solver_preset(solver, utilities::getstringvalue(value).c_str());
   }
   const uno_int option_type = uno_get_solver_option_type(solver, name.c_str());
   if (option_type == UNO_OPTION_TYPE_NOT_FOUND) {
      utilities::warnWithId("unknownOption", "Unknown option '{}'; skipping.", name);
      return true;
   }
   if (option_type == UNO_OPTION_TYPE_STRING) {
      if (!utilities::isstring(value)) {
         utilities::errWithId("invalidOption", "Option '{}' must be a string.", name);
      }
      return uno_set_solver_string_option(solver, name.c_str(), utilities::getstringvalue(value).c_str());
   }
   if (option_type == UNO_OPTION_TYPE_BOOL) {
      if (value.getType() == matlab::data::ArrayType::LOGICAL) {
         matlab::data::TypedArray<bool> logical_value(value);
         return uno_set_solver_bool_option(solver, name.c_str(), logical_value[0]);
      }
      if (value.getType() == matlab::data::ArrayType::DOUBLE) {
         return uno_set_solver_bool_option(solver, name.c_str(), utilities::getscalar<double>(value) != 0.);
      }
      utilities::errWithId("invalidOption", "Option '{}' must be logical or numeric.", name);
   }
   if (option_type == UNO_OPTION_TYPE_INTEGER) {
      uno_int int_value = 0;
      if (value.getType() == matlab::data::ArrayType::INT32) {
         matlab::data::TypedArray<int32_t> int_array(value);
         int_value = static_cast<uno_int>(int_array[0]);
      }
      else if (value.getType() == matlab::data::ArrayType::INT64) {
         matlab::data::TypedArray<int64_t> int_array(value);
         int_value = static_cast<uno_int>(int_array[0]);
      }
      else {
         int_value = static_cast<uno_int>(utilities::getscalar<double>(value));
      }
      return uno_set_solver_integer_option(solver, name.c_str(), int_value);
   }
   if (option_type == UNO_OPTION_TYPE_DOUBLE) {
      return uno_set_solver_double_option(solver, name.c_str(), utilities::getscalar<double>(value));
   }
   return false;
}

void apply_solver_options(void* solver, const matlab::data::StructArray& options) {
   for (const auto& field : options.getFieldNames()) {
      const std::string field_name(field);
      if (!set_option_from_matlab(solver, field_name, options[0][field])) {
         utilities::errWithId("invalidOption", "Failed to set option '{}'.", field_name);
      }
   }
}

void apply_solver_callbacks(void* solver, MexCallbackContext& callbacks) {
   if (callbacks.has_new_primals) {
      utilities::warnWithId("unsupportedCallback", "new_primals_callback is not supported by the Uno C API; ignored.");
   }
   if (callbacks.has_new_multipliers) {
      utilities::warnWithId("unsupportedCallback",
         "new_multipliers_callback is not supported by the Uno C API; ignored.");
   }
   uno_notify_acceptable_iterate_callback notify_cb = callbacks.has_acceptable_iterate ? notify_acceptable_iterate_callback : nullptr;
   if (!uno_set_solver_callbacks(solver, notify_cb, nullptr, &callbacks)) {
      utilities::errWithId("cApiFailure", "uno_set_solver_callbacks failed.");
   }
}

matlab::data::StructArray pack_solution(void* solver, std::size_t number_variables, std::size_t number_constraints,
   matlab::data::ArrayFactory& factory) {
   matlab::data::StructArray ret_val =
      factory.createStructArray({1, 1}, {"solution", "cpu_time", "termination_status"});
   matlab::data::StructArray sol = factory.createStructArray({1, 1},
      {"primals", "duals_lb_x", "duals_ub_x", "duals_constraints"});

   matlab::data::TypedArray<double> x = factory.createArray<double>({number_variables, 1});
   matlab::data::TypedArray<double> z_l = factory.createArray<double>({number_variables, 1});
   matlab::data::TypedArray<double> z_u = factory.createArray<double>({number_variables, 1});
   matlab::data::TypedArray<double> z_c = factory.createArray<double>({number_constraints, 1});
   uno_get_primal_solution(solver, &(*x.begin()));
   uno_get_lower_bound_dual_solution(solver, &(*z_l.begin()));
   uno_get_upper_bound_dual_solution(solver, &(*z_u.begin()));
   if (number_constraints > 0) {
      uno_get_constraint_dual_solution(solver, &(*z_c.begin()));
   }
   sol[0]["primals"] = std::move(x);
   sol[0]["duals_lb_x"] = std::move(z_l);
   sol[0]["duals_ub_x"] = std::move(z_u);
   sol[0]["duals_constraints"] = std::move(z_c);
   ret_val[0]["solution"] = std::move(sol);
   ret_val[0]["cpu_time"] = factory.createScalar(uno_get_cpu_time(solver));
   ret_val[0]["termination_status"] = factory.createScalar(optimization_status_message(uno_get_optimization_status(solver)));
   return ret_val;
}

matlab::data::StructArray export_solver_options(void* solver, matlab::data::ArrayFactory& factory) {
   std::vector<std::string> names;
   for (std::size_t i = 0; i < kUnoOptionNamesCount; ++i) {
      const char* name = kUnoOptionNames[i];
      if (uno_get_solver_option_type(solver, name) != UNO_OPTION_TYPE_NOT_FOUND) {
         names.emplace_back(name);
      }
   }
   matlab::data::StructArray options = factory.createStructArray({1, 1}, names);
   for (const std::string& name : names) {
      const uno_int option_type = uno_get_solver_option_type(solver, name.c_str());
      if (option_type == UNO_OPTION_TYPE_INTEGER) {
         options[0][name] =
            factory.createScalar(static_cast<double>(uno_get_solver_integer_option(solver, name.c_str())));
      }
      else if (option_type == UNO_OPTION_TYPE_DOUBLE) {
         options[0][name] = factory.createScalar(uno_get_solver_double_option(solver, name.c_str()));
      }
      else if (option_type == UNO_OPTION_TYPE_BOOL) {
         options[0][name] = factory.createScalar(uno_get_solver_bool_option(solver, name.c_str()));
      }
      else if (option_type == UNO_OPTION_TYPE_STRING) {
         options[0][name] = factory.createScalar(uno_get_solver_string_option(solver, name.c_str()));
      }
   }
   return options;
}

} // namespace unomex
