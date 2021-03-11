//
// Created by andreas on 11.11.20.
//

#include <locale>
#include <iostream>
#include "LogFile.h"

/**
 * @fn void LogFile::open(char *acFilename)
 * @brief stores the filename and the datetime of the experiment
 * @param acFilename name of log file to write results
 */
void LogFile::open(char *acFilename)
{
    this->acFilename = acFilename;
    auto start = std::chrono::system_clock::now();
    std::time_t timeNow = std::chrono::system_clock::to_time_t(start);
    std::string sString = "----";
    sString.append(std::ctime(&timeNow));
    write(sString, false);
}

/**
 * @fn void LogFile::close()
 * @brief opens locks and writes the file with all lines stored
 */
void LogFile::close()
{
    FILE * pFile = fopen(acFilename, "a");
    flockfile(pFile);
    for(int i = 0; i < fileContent.size(); i++)
    {
        fwrite(fileContent.at(i).c_str(), fileContent.at(i).size(), 1, pFile);
    }
    funlockfile(pFile);
    fclose(pFile);
}

/**
 * @fn void LogFile::write(std::string sText, std::string sValue)
 * @brief used when one of the strings is declared via a preprocessor define and you don't want to declare
 * a new string object to append the \refitem sValue to it
 * @param sText Basic text to store
 * @param sValue second string to store in file
 */
void LogFile::write(std::string sText, std::string sValue)
{
    std::string sString = sText + sValue;
    write(sString, true);
}

/**
 * @fn void LogFile::write(std::string sText, double dValue)
 * @brief stores a string and double value in the file
 * @param sText string to write
 * @param dValue double value to write in file
 */
void LogFile::write(std::string sText, double dValue)
{
    std::string sString = sText + std::to_string(dValue);
    sString.replace(sString.find(","), 1, ".");
    write(sString, true);
}

/**
 * @fn void LogFile::write(std::string sText, int iValue)
 * @brief stores a string and int value in the file
 * @param sText string to write
 * @param iValue value to store in file
 */
void LogFile::write(std::string sText, int iValue)
{
    std::string sString = sText + std::to_string(iValue);
    write(sString, true);
}

/**
 * @fn void LogFile::write(std::string sText, bool lineFeed)
 * @brief writes a single string in file and appends a linefeed character if needed
 * @param sText string to store in file
 * @param bLineFeed true when linefeed should be added
 */
void LogFile::write(std::string sText, bool bLineFeed)
{
    if(bLineFeed)
    {
        sText += "\n";
    }
    fileContent.push_back(sText);
}
