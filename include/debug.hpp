#ifndef EPI_DEBUG_H
#define EPI_DEBUG_H
#include "collider.hpp"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
namespace epi {

/* consider adding boost thread id since we'll want to know whose writting and
 * won't want to repeat it for every single call */

/* consider adding policy class to allow users to redirect logging to specific
 * files via the command line
 */

#if _DEBUG
#define DEBUG_CALL(x) x 
#else 
#define DEBUG_CALL(x) ;
#endif
enum class LogLevel { 
    ERROR,
    WARNING,
    INFO,
    DEBUG,
};
#if _DEBUG

#define Log1(loglevel) LogIt(loglevel)
#define Log2(loglevel, duration) Log3Helper(loglevel, duration, __LINE__)
#define Log3Helper(loglevel, duration, line) Log3(loglevel, duration, line)

#define Log3(loglevel, duration, line)\
static auto epi_time_at##line= std::chrono::steady_clock::now();\
bool result##line = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - epi_time_at##line).count() > duration * 1000.0;\
if(result##line) epi_time_at##line= std::chrono::steady_clock::now();\
if(result##line)\
LogIt(loglevel)

#else

#define Log1(loglevel) if(false)LogIt(loglevel)
#define Log2(loglevel, duration) if(false)LogIt(loglevel)
#endif



#define GET_MACRO(_0, _1, _2, NAME, ...) NAME
#define Log(...) GET_MACRO(_0, ##__VA_ARGS__, Log2, Log1, Log0)(__VA_ARGS__)


class LogIt
{
public:
    LogIt(LogLevel _loglevel) {
        switch(_loglevel) {
            case LogLevel::ERROR:
                _buffer << "\x1B[31m[ERROR]:\x1B[0m\t";
            break;
            case LogLevel::WARNING:
                _buffer << "\x1B[33m[WARNING]:\x1B[0m\t";
            break;
            case LogLevel::INFO:
                _buffer << "\x1B[36m[INFO]:\x1B[0m\t";
            break;
            case LogLevel::DEBUG:
                _buffer << "\x1B[34m[DEBUG]:\x1B[0m\t";
            break;
        }
    }

    template <typename T>
    LogIt & operator<<(T const & value)
    {
        if(_isActive)
            _buffer << value;
        return *this;
    }
    static void addOutput(std::ostream& out) {
        _output.push_back(&out);
    }

    ~LogIt() {
        if(!_isActive)
            return;
        _buffer << std::endl;
        for(auto& o : _output) {
            *o << _buffer.str();
        }
    }

private:
    bool _isActive = true;
    std::ostringstream _buffer;

    static std::vector<std::ostream*> _output;
    static LogLevel _priority;
};
}
#endif
