#include "logger.h"

Logger::Logger(std::string fileName)
{
    FileName = fileName;
    MyFile.open(FileName);
}

Logger::~Logger()
{
    MyFile.close();
}

void Logger::Log(std::vector<float> data)
{
    for (size_t i = 0; i < data.size(); i++)
    {
        MyFile << data[i] << std::endl;
    }
}