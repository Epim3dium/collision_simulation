#include "debug.hpp"
namespace epi {

std::vector<std::ostream*> LogIt::_output = {&std::cerr};
LogLevel LogIt::_priority = LogLevel::DEBUG;

};
