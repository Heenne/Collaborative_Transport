#pragma once
#include<exception>
#include<string>

/** \addtogroup MultiRobotController 
 *  @{
 */
class NecessaryParamException:public std::runtime_error
{
    public:
        NecessaryParamException():runtime_error("Not specified")
        {
        }   
        NecessaryParamException(std::string param_name_resolved):runtime_error(param_name_resolved)
        {            
        }
};