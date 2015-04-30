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

#include "quaternionEKFThread.h"

quaternionEKFThread::quaternionEKFThread ( int period, 
                                           yarp::os::Property &filterParams)
    : RateThread ( m_period ),
      m_filterParams( filterParams )
{
    

}

void quaternionEKFThread::run()
{
    
}

bool quaternionEKFThread::threadInit()
{
    return true;
}

void quaternionEKFThread::threadRelease()
{
    delete m_parser;
    m_parser = NULL;
}

