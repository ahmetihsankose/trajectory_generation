#pragma once
#include <vector>
#include <fstream>

class Logger
{
private:
    std::string FileName;
    std::ofstream MyFile;

public:
    Logger(std::string fileName);
    ~Logger();
    void Log(std::vector<float> data);

};