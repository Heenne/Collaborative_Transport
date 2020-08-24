#pragma once
#include<exception>
/** \addtogroup MultiRobotController 
 *  @{
 */
class InputAllocException:public std::exception
{
    public:
        const char * what () const throw () {
        return "Could not alloc a proper input source!";
   }
};