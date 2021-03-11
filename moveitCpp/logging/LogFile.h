//
// Created by andreas on 11.11.20.
//

#ifndef GRASPITCPP_LOGFILE_H
#define GRASPITCPP_LOGFILE_H

#include "../config.h"

#ifdef LOG_RESULTS
#include "string"
#include "vector"
#include <chrono>
#include <ctime>
class LogFile
{
public:

    /**
     * @fn LogFile()
     * @brief default constructor
     */
    LogFile();

    /**
     * @fn void open(char * acFilename)
     * @brief stores the filename and current timestamp of this experiment
     * @param acFilename filename with path of logfile
     */
    void open(char * acFilename);

    /**
     * @fn void write(std::string sText, double dValue)
     * @brief stores one more line in vector to write logfile later
     * @param sText
     * @param dValue
     */
    void write(std::string sText, double dValue);

    /**
     * @fn void write(std::string sText, int iValue)
     * @brief stores a string with int value in vector
     * @param sText basic string
     * @param iValue value to be stored
     */
    void write(std::string sText, int iValue);

    /**
     * @fn void LogFile::write(std::string sText, std::string sValue)
     * @brief used when one of the strings is declared via a preprocessor define and you don't want to declare
     * a new string object to append the \refitem sValue to it
     * @param sText Basic text to store
     * @param sValue second string to store in file
     */
    void write(std::string sText, std::string sValue);

    /**
     * @fn void write(std::string sText, bool bLineFeed = false)
     * @param sText string to be stored in file
     * @param bLineFeed true if line feed should be added later
     */
    void write(std::string sText, bool bLineFeed = false);

    /**
     * @fn void close()
     * @brief opens, locks, writes strings in file and closes file finally again
     */
    void close();
private:
    char * acFilename = NULL; /*!< intern memory for filename */
    std::vector<std::string> fileContent; /*!< vector that contains all lines to be written in log file*/
};
#endif /*LOG_RESULTS*/

#endif //GRASPITCPP_LOGFILE_H
