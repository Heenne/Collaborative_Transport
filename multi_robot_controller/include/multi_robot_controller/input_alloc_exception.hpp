#pragma once
#include<exception>

class InputAllocException:public std::exception
{
    public:
        const char * what () const throw () {
        return "Could not alloc a proper input source!";
   }
};