%{
#include "initialization.h"
%}

%include std_string.i
%include std_vector.i

namespace std 
{
   %template(V_string) vector<string>;
}

%include "initialization.h"

%init %{

%}

