#include "logger.h"

#include <ostream>
#include <iostream>

void Logger::info(const std::string& msg)
{
    std::cout << msg << std::endl;
}

void Logger::warning(const std::string& msg)
{
    std::cerr << msg << std::endl;
}

void Logger::error(const std::string& msg)
{
    std::cerr << msg << std::endl;
}

std::unique_ptr<Logger> Logger::instance;