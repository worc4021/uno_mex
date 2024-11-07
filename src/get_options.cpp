#include "mex.hpp"
#include "mexAdapter.hpp"
#include "utilities.hpp"
#include "options/Options.hpp"
#include "options/DefaultOptions.hpp"

class MexFunction 
    : public matlab::mex::Function 
{

public:
    MexFunction()  {
        matlabPtr = getEngine();
    }
    ~MexFunction() = default;
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        
        uno::Options solvers_options = uno::DefaultOptions::determine_solvers_and_preset();
        if (inputs.size())
        {
            if (!utilities::isstring(inputs[0]))
            {
                utilities::error("Pass a string with the preset name.");
            }
            std::string preset = utilities::getstringvalue(inputs[0]);
            uno::Options::set_preset(solvers_options, preset);
        }

    }
};