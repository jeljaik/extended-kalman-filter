/*
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
 */
#ifndef __DATADUMPERPARSER_H__
#define __DATADUMPERPARSER_H__

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/iostreams/stream.hpp>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <string>
#include <stdlib.h>
#include <yarp/os/LogStream.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>

using boost::iostreams::mapped_file_source;
using boost::iostreams::stream;

struct currentData {
    long double time;
    MatrixWrapper::ColumnVector measurement;
};

class dataDumperParser{
    boost::iostreams::mapped_file* m_mmap;
    mapped_file_source             m_mmap2;
    stream<mapped_file_source>     m_file_stream;
    std::string                    m_srcFile;
    const char*                    m_pFirst;
    const char*                    m_pLast;
    const char*                    m_pCurrent;
    uintmax_t                      numlines;
public:
    dataDumperParser(std::string srcFile);
    ~dataDumperParser();
    bool openFile();
    void parseFile();
    bool parseFileistream();
    bool countLines();
    bool parseLine(currentData &currData);
    bool closeFile();
    
};

#endif