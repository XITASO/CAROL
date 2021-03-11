//
// Created by andreas on 11.11.20.
//

#ifndef GRASPITCPP_LOGFILE_H
#define GRASPITCPP_LOGFILE_H

#include "string"
#include "vector"
#include <chrono>
#include <ctime>
class LogFile
{
public:
    void open(char * acFilename);
    void write(std::string sText, double dValue);
    void write(std::string sText, int iValue);
    void write(std::string sText, std::string sValue);
    void write(std::string sText, bool bLineFeed = false);
    void close();
private:
    char * acFilename = NULL; /* intern variable for filename */
    std::vector<std::string> fileContent; /* vector with all lines to write in the file */
};


#endif //GRASPITCPP_LOGFILE_H
