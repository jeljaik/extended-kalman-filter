/*
 * This class has been taught specifically for parsing data from a dataDumper-style log file.
 * It uses the Boost library for memory mapping the logged data to increase I/O performance.
 * This is a much faster way of parsing when using large files (as is the case with dumped data),
 * and definitely a better option with respect to traditional C methods.
 * 
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Jorhabib Eljaik
 * email:  jorhabib.eljaik@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * 
 */

#include "dataDumperParser.h"


dataDumperParser::dataDumperParser(std::string srcFile): m_srcFile(srcFile)
{
//     Cannot do the following in the constructor as m_pFirst and m_pLast would not be accessible.
//     boost::iostreams::mapped_file m_mmap(srcFile, boost::iostreams::mapped_file::readonly);
//     // Retrieves pointer to the first byte of data in the mapping associated with mmap
//     m_pFirst = m_mmap.const_data();
//     // Pointer to last byte of data in the mapping
//     m_pLast = m_pFirst + m_mmap.size();
//     
    m_pLast = 0;
    m_pFirst = 0;
    m_pCurrent = 0;
    numlines = 0;
}
dataDumperParser::~dataDumperParser()
{
    if (m_mmap) {
        delete m_mmap;
        m_mmap = NULL;
    }
}

void dataDumperParser::parseFile() { 
    // Maps full file in mmap
    m_mmap = new boost::iostreams::mapped_file(m_srcFile, boost::iostreams::mapped_file::readonly);
    // Retrieves pointer to the first byte of data in the mapping associated with mmap
    m_pFirst = m_mmap->const_data();
    // Pointer to last byte of data in the mapping
    m_pLast = m_pFirst + m_mmap->size();
    m_pCurrent = m_pFirst;
}

bool dataDumperParser::parseFileistream() { 
    //     2. Wrapping the source device in a istream
    //     
    //     This gives you all the usual stream-based operations of c++ standard streams, so you can detect the end of the file like you would always:
    //     
    //     #include <boost/iostreams/device/mapped_file.hpp> // for mmap
    //     #include <boost/iostreams/stream.hpp>             // for stream
    //     #include <algorithm>                              // for std::find
    //     #include <iostream>                               // for std::cout
    //     #include <cstring>
    //     
    //     int main()
    //     {
    //         using boost::iostreams::mapped_file_source;
    //         using boost::iostreams::stream;
    //         mapped_file_source mmap("test.cpp");
    //         stream<mapped_file_source> is(mmap, std::ios::binary);
    //         
    //         std::string line;
    //         
    //         uintmax_t m_numLines = 0;
    //         while (std::getline(is, line))
    //         {
    //             m_numLines++;
    //         }
    //         
    //         std::cout << "m_numLines = " << m_numLines << "\n";
    //     }
        
    m_mmap2.open(m_srcFile);
    stream<mapped_file_source> m_file_stream(m_mmap2, std::ios::binary);
    std::string line;
    std::getline(m_file_stream, line);
}

bool dataDumperParser::parseLine(currentData &currData)
{
    long double currentTime = 0;
    const char* eol;
    
    // Find end of line from position of m_pCurrent
    if (m_pCurrent && m_pCurrent!=m_pLast) {
        eol = static_cast<const char*>(memchr(m_pCurrent, '\n', m_pLast - m_pFirst));
    
    
    // Find Length of the line
    char del[] = "\n";
    int lengthLine = strcspn(m_pCurrent, del);
    
    // Copy the current line
    char tmpLine[(size_t) lengthLine];
    strncpy(tmpLine, m_pCurrent, lengthLine);
    
    // Null character manually added
    tmpLine[lengthLine] = '\0';
    //TODO Make this print optional if some VERBOSE flag is activated
//     yInfo(" [dataDumperParser::parseLine] Current line:\n %s", tmpLine);
    
    // Divide line  in tokens
    char* tmp;
    tmp = strtok(tmpLine, " ");
    unsigned int col = 0;
    
    // Pass data to currData
    // TODO This can't  be hardcoded to 12 as this can vary according to the sensor that was dumped
    currData.measurement.resize(12);
    double tmpIdx;
    while (tmp != NULL) {
        if (col == 1) {
            currData.time = atof(tmp);
        }
        else
            if (col > 1) {
                currData.measurement.insert_element(col - 2, atof(tmp));
            }
        col++;
        
        // A null pointer may be specified to strtok, in which case the function continues 
        // scanning where a previous successful call to the function ended
        tmp = strtok (NULL, " ");
    }

//     Printing content of currData.measurement with iterator
//     for (boost::numeric::ublas::vector<double>::iterator i = currData.measurement.begin(); i!=currData.measurement.end(); ++i)
//         std::cout << *i << " ";
//     std::cout << std::endl;
    
    // Leave m_pCurrent at the beginning of the following line
    eol++;
    m_pCurrent = eol;
    return true;
    } else {
        yError("[dataDumperParser::parseLine] No line to parse! ");
        return false;
    }
}

bool dataDumperParser::countLines()
{
    bool ans = false;
    const char* pFirst = m_pFirst;
    const char* pLast  = m_pLast;
    while ( pFirst && pFirst!=pLast ) {
        if ((pFirst = static_cast<const char*>(memchr(pFirst, '\n', pLast-pFirst))))
            numlines++, pFirst++;
    }
    yInfo(" [dataDumperParser::countLines] Number of lines in %s: %i", m_srcFile.c_str(), (int) numlines);
    ans = true;
    return ans;
}

bool dataDumperParser::closeFile()
{

}
