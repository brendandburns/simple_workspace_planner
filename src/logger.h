#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <string>
#include <memory>

class Logger {
    public:
        Logger() = default;
        ~Logger() = default;

        virtual void info(const std::string& msg);
        virtual void warning(const std::string& msg);
        virtual void error(const std::string& msg);
    
        static std::unique_ptr<Logger> instance;
};

#endif // __LOGGER_H__