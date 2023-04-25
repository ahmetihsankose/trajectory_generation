/*
    Written by Ahmet Ihsan KOSE, Istanbul, Turkey
    Contact koseahmetihsan@gmail.com
*/

#pragma once
#include <vector>
#include <fstream>

class Logger
{
private:
    std::string FileName;
    std::ofstream MyFile;

public:
    Logger();
    ~Logger();
    void Log(const std::vector<float> &data, const std::string &fileName);

};