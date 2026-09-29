#include "astribot_s1_navigation_policy_native/policy_numeric.hpp"
#include <iostream>
int main(){double p[3];while(std::cin.read(reinterpret_cast<char*>(p),sizeof p)){double o[2]={astribot::navigation::policy::euclidean_norm(p[0],p[1]),astribot::navigation::policy::euclidean_norm(p[0],p[1],p[2])};std::cout.write(reinterpret_cast<char*>(o),sizeof o);}}
