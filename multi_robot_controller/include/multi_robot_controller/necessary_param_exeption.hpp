#pragma once
#include<exception>
#include<string>

class NecessaryParamException:public std::exception
{
    public:
        NecessaryParamException()
        {
            this->param_name_="Not specified";
        }   
        NecessaryParamException(std::string param_name_resolved)
        {
            this->param_name_=param_name_resolved;
        }
        virtual const char* what() const throw() override
        {
            std::stringstream str;
            str<<"Could not find parameter with name: "<<this->param_name_;
            return str.str().c_str();
        }
    private:
        std::string param_name_;
};