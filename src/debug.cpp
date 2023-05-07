#include "debug.hpp"
namespace epi {

std::vector<std::ostream*> LogIt::_output = {&std::cerr};
LogLevel LogIt::_priority = LogLevel::DEBUG;
std::map<size_t, LogIt::timePoint> LogIt::_saved_times;

};
