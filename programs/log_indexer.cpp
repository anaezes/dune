//***************************************************************************
// Copyright 2007-2018 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Ana Santos                                                       *
//***************************************************************************

// ISO C++ headers
#include <iostream>
#include <sstream>
#include <cstdio>

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Database.hpp>

using DUNE_NAMESPACES;

struct Log
{
    std::string name;
    std::string vehicle;
    //std::vector<std::string> sensors;
    double distance;
    double latStart;
    double lonStart;
    std::time_t date;
    //std::vector<std::string> errors;
    //std::vector<std::string> warnings;
    double duration;

    Log(const std::string& a_name, const std::string& a_vehicle, double a_distance, double a_latStart, double a_lonStart, std::time_t a_date, double a_duration):
            name(a_name),
            vehicle(a_vehicle),
            distance(a_distance),
            latStart(a_latStart),
            lonStart(a_lonStart),
            date(a_date),
            duration(a_duration)
           /* sensors(0),
            errors(0),
            warnings(0) */
    { }
};

void testDB(Database::Connection connection);

// Minimum rpm before starting to assume that the vehicle is moving
const float c_min_rpm = 400.0;
// Maximum speed between to consider when integrating
const float c_max_speed = 6.0;
// Timestep
const float c_timestep = 0.5;

Log
getLog(char* file) {
    std::istream* is = 0;
    Compression::Methods method = Compression::Factory::detect(file);
    if (method == METHOD_UNKNOWN)
        is = new std::ifstream(file, std::ios::binary);
    else
        is = new Compression::FileInput(file, method);

    IMC::Message* msg = NULL;

    uint16_t curr_rpm = 0;

    bool got_state = false;
    IMC::EstimatedState estate;
    double last_lat, last_lon;
    double latStart = 0.0;
    double lonStart = 0.0;
    std::time_t date;

    // Accumulated travelled distance
    double distance = 0.0;
    // Accumulated travelled time
    double duration = 0.0;

    bool got_name_log = false;
    std::string log_name = "unknown";

    bool ignore = false;
    uint16_t sys_id = 0xffff;

    bool got_name_veh = false;
    std::string vehicle_name = "unknown";

    try
    {
        bool first = true;
        while ((msg = IMC::Packet::deserialize(*is)) != 0)
        {
            if (msg->getId() == DUNE_IMC_ANNOUNCE)
            {
                IMC::Announce* ptr = static_cast<IMC::Announce*>(msg);
                if (sys_id == ptr->getSource())
                {
                    vehicle_name = ptr->sys_name;
                    got_name_veh = true;
                }
            }
            else if (msg->getId() == DUNE_IMC_LOGGINGCONTROL)
            {
                if (!got_name_log)
                {
                    IMC::LoggingControl* ptr = static_cast<IMC::LoggingControl*>(msg);

                    if (ptr->op == IMC::LoggingControl::COP_STARTED)
                    {
                        sys_id = ptr->getSource();
                        log_name = ptr->name;
                        got_name_log = true;
                    }
                }
            }
            else if (msg->getId() == DUNE_IMC_ESTIMATEDSTATE)
            {
                if (msg->getTimeStamp() - estate.getTimeStamp() > c_timestep)
                {
                    IMC::EstimatedState* ptr = static_cast<IMC::EstimatedState*>(msg);

                    if (!got_state)
                    {
                        estate = *ptr;
                        Coordinates::toWGS84(*ptr, last_lat, last_lon);

                        got_state = true;
                    }
                    else if (curr_rpm > c_min_rpm)
                    {

                        double lat, lon;
                        date = estate.getTimeStamp();
                        Coordinates::toWGS84(*ptr, lat, lon);

                        double dist = Coordinates::WGS84::distance(last_lat, last_lon, 0.0,
                                                                   lat, lon, 0.0);

                        // Not faster than maximum considered speed
                        if (dist / (ptr->getTimeStamp() - estate.getTimeStamp()) < c_max_speed)
                        {
                            distance += dist;
                            duration += msg->getTimeStamp() - estate.getTimeStamp();
                        }

                        estate = *ptr;
                        last_lat = lat;
                        last_lon = lon;

                        if(first)
                        {
                            latStart = DUNE::Math::Angles::degrees(lat);
                            lonStart = DUNE::Math::Angles::degrees(lon);
                            first = false;
                        }
                    }
                }
            }
            else if (msg->getId() == DUNE_IMC_RPM)
            {
                IMC::Rpm* ptr = static_cast<IMC::Rpm*>(msg);
                curr_rpm = ptr->value;
            }
/*            else if (msg->getId() == DUNE_IMC_SIMULATEDSTATE)
            {
                // since it has simulated state let us ignore this log
                ignore = true;
                delete msg;
                std::cerr << "this is a simulated log";
                break;
            }*/

            delete msg;

            // ignore idles
            // either has the string _idle or has only the time.
 /*           if (log_name.find("_idle") != std::string::npos ||
                log_name.size() == 15)
            {
                ignore = true;
                std::cerr << "this is an idle log";
                break;
            }*/
        }
    }
    catch (std::runtime_error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    delete is;

    std::cout << std::endl;
    std::cout << "::: LOG ::::" << std::endl;
    std::cout << "log_name : " << log_name << std::endl;
    std::cout << "vehicle_name : " << vehicle_name << std::endl;
    std::cout << "distance : " << distance << std::endl;
    std::cout << "last_lat : " << latStart << std::endl;
    std::cout << "last_lon : " << lonStart << std::endl;
    std::cout << "date : " << date << std::endl;
    std::cout << "duration : " << duration << std::endl;

    return Log(log_name, vehicle_name, distance, latStart, lonStart, date, duration);
}


int
main(int32_t argc, char** argv) {
    if (argc <= 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_log_1/Data.lsf[.gz]> ... <path_database/database.db>"
                  << std::endl;
        return 1;
    }


    // get log information
    Log log = getLog(argv[1]);

    // add to database (...)
    DUNE::Database::Connection db(argv[2], DUNE::Database::Connection::CF_CREATE);

    db.beginTransaction();
    std::cout << std::endl << "Connection::beginTransaction()" << std::endl;

    DUNE::Database::Statement insertion("INSERT INTO LOG VALUES(?,?,?,?,?,?,?,?,?)", db);
    insertion << log.name
              << log.vehicle
              << log.distance
              << log.latStart
              << log.lonStart
              << log.date
              << "no"
              << "yes"
              << log.duration;

    insertion.execute();

    //db.commit();

    testDB(db);

}

void testDB(Database::Connection db) {
    DUNE::Database::Statement query("SELECT * FROM LOG", db);

    while (query.execute()) {
        std::string name;
        std::string vehicle;
        double distance;
        double latStart;
        double lonStart;
        long date;
        double duration;
        std::string errors;
        std::string warnings;


        query >> name
              >> vehicle
              >> distance
              >> latStart
              >> lonStart
              >> date
              >> errors
              >> warnings
              >> duration;

        std::cout << "::: LOG ::::" << std::endl;
        std::cout << "log_name : " << name << std::endl;
        std::cout << "vehicle_name : " << vehicle << std::endl;
        std::cout << "distance : " << distance << std::endl;
        std::cout << "last_lat : " << latStart << std::endl;
        std::cout << "last_lon : " << lonStart << std::endl;
        std::cout << "date : " << date << std::endl;
        std::cout << "errors : " << errors << std::endl;
        std::cout << "warnings : " << warnings << std::endl;
        std::cout << "duration : " << duration << std::endl << std::endl;
    }
}

