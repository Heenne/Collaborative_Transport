#include<exception>
#include<string>

class NecessaryParamException:public std::exception
{
    public:
        NecessaryParamException(std::string param_name_resolved)
        {
            this->param_name_=param_name_resolved;
        }
        virtual const char* what() const throw()
        {
            std::string str;
            str="Could not find parameter with name: ";
            str+=this->param_name_;
            return str.c_str();
        }
    private:
        std::string param_name_;
};