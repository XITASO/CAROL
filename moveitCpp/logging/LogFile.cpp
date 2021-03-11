//
// Created by andreas on 11.11.20.
//

#include <locale>
#include "LogFile.h"

#ifdef LOG_RESULTS
LogFile::LogFile()
{

}

void LogFile::open(char *acFilename)
{
    this->acFilename = acFilename;
    auto start = std::chrono::system_clock::now();
    std::time_t timeNow = std::chrono::system_clock::to_time_t(start);
    std::string sString = BEGIN_EXPERIMENT;
    sString.append(std::ctime(&timeNow));
    write(sString, false);
}


void LogFile::close()
{
    write(END_EXPERIMENT, true);
    FILE * pFile = fopen(acFilename, "a");
    flockfile(pFile);
    for(int i = 0; i < fileContent.size(); i++)
    {
        fwrite(fileContent.at(i).c_str(), fileContent.at(i).size(), 1, pFile);
    }
    funlockfile(pFile);
    fclose(pFile);
}

void LogFile::write(std::string sText, std::string sValue)
{
    std::string sString = sText + sValue;
    write(sString, true);
}

void LogFile::write(std::string sText, double dValue)
{
    std::string sString = sText + std::to_string(dValue);
    int pos = sString.find(",");
    if(pos < sString.size() && pos >= 0)
    {
        sString.replace(sString.find(","), 1, ".");
    }
    write(sString, true);
}

void LogFile::write(std::string sText, int iValue)
{
    std::string sString = sText + std::to_string(iValue);
    write(sString, true);
}

void LogFile::write(std::string sText, bool bLineFeed)
{
    if(bLineFeed)
    {
        sText += "\n";
    }
    fileContent.push_back(sText);
}
#endif /*LOG_RESULTS*/