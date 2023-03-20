#include "logger.h"

Logger::Logger()
{
}

Logger::~Logger()
{
}

void Logger::Log(const std::vector<float> &data, const std::string &fileName)
{
    FileName = fileName;

    MyFile.open(FileName);

    for (size_t i = 0; i < data.size(); i++)
    {
        MyFile << data[i] << std::endl;
    }
    
    MyFile.close();
}