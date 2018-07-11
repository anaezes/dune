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
#include <vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Database.hpp>

using DUNE_NAMESPACES;

struct Log
{
    std::string name;
    std::string vehicle;
    std::vector<std::string> sensors;
    double distance;
    double latStart;
    double lonStart;
    std::time_t date;
    //std::vector<std::string> errors;
    //std::vector<std::string> warnings;
    double duration;

    Log(const std::string& a_name, const std::string& a_vehicle, std::vector<std::string> sensors, double a_distance, double a_latStart, double a_lonStart, std::time_t a_date, double a_duration):
            name(a_name),
            vehicle(a_vehicle),
            distance(a_distance),
            latStart(a_latStart),
            lonStart(a_lonStart),
            date(a_date),
            duration(a_duration),
            sensors(sensors)
     /*errors(0),
     warnings(0) */
    { }
};

// Minimum rpm before starting to assume that the vehicle is moving
const float c_min_rpm = 400.0;
// Maximum speed between to consider when integrating
const float c_max_speed = 6.0;
// Timestep
const float c_timestep = 0.5;

static const char* c_log_table_stmt =
        "CREATE TABLE IF NOT EXISTS log ( "
        "name text PRIMARY KEY,"
        "vehicle text NOT NULL,"
        "distTravelled REAL NOT NULL,"
        "startLat REAL NOT NULL,"
        "startLon REAL NOT NULL,"
        "\"date\" INTEGER NOT NULL,"
        "errors text,"
        "warnings text,"
        "duration REAL NOT NULL"
        ");";

static const char* c_sensor_table_stmt =
        "CREATE TABLE IF NOT EXISTS sensor ( "
        "sensorName text NOT NULL"
        ");";

static const char* c_log_sensor_table_stmt =
        "CREATE TABLE IF NOT EXISTS log_sensor ( "
        "logName text NOT NULL REFERENCES log ON DELETE CASCADE,"
        "sensorName text NOT NULL REFERENCES sensor ON DELETE CASCADE"
        ");";

static const char* c_insert_sensor_stmt = "INSERT OR IGNORE INTO SENSOR VALUES(?)";
static const char* c_insert_log_sensor_stmt = "INSERT OR IGNORE INTO LOG_SENSOR VALUES(?,?)";
static const char* c_insert_log_stmt = "INSERT OR IGNORE INTO LOG VALUES(?,?,?,?,?,?,?,?,?)";

const char * const sensorsList[] = {"Ctd", "Sidescan", "Imu", "Multibeam", "Camara", "Ranger"};

const int NSENSORS = 6;


Log
getLog(std::string file) {
    std::istream* is = 0;
    Compression::Methods method = Compression::Factory::detect(file.c_str());
    if (method == METHOD_UNKNOWN)
        is = new std::ifstream(file.c_str(), std::ios::binary);
    else
        is = new Compression::FileInput(file.c_str(), method);

    IMC::Message* msg = NULL;

    uint16_t curr_rpm = 0;

    bool got_state = false;
    IMC::EstimatedState estate;
    double last_lat, last_lon;
    double latStart = 0.0;
    double lonStart = 0.0;
    std::time_t date = 0;

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

    std::set<std::string> sensors_tmp;

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
            else if(msg->getId() == DUNE_IMC_PLANSPECIFICATION)
            {
               IMC::PlanSpecification* pspec = static_cast<IMC::PlanSpecification*>(msg);
                const IMC::SetEntityParameters* t_sep;

                IMC::MessageList<IMC::PlanManeuver>::const_iterator it = pspec->maneuvers.begin();
                for (; it != pspec->maneuvers.end(); it++ )
                {
                    IMC::MessageList<IMC::Message>::const_iterator it1 = (*it)->start_actions.begin();
                    for (; it1 != (*it)->start_actions.end(); it1++ )
                    {
                        if((*it1)->getId() == DUNE_IMC_SETENTITYPARAMETERS)
                        {
                            t_sep = static_cast<const IMC::SetEntityParameters *>(*it1);
                            sensors_tmp.insert(t_sep->name);
                        }
                    }
                }

            }
            else if(msg->getId() == DUNE_IMC_SETENTITYPARAMETERS)
            {
                IMC::SetEntityParameters* etparam = static_cast<IMC::SetEntityParameters*>(msg);
                sensors_tmp.insert(etparam->name);
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

    std::vector<std::string> sensors;
    std::copy(sensors_tmp.begin(), sensors_tmp.end(), std::back_inserter(sensors));


    /* TEST LOG */
    std::cout << std::endl;
    std::cout << "::: LOG ::::" << std::endl;
    std::cout << "log_name : " << log_name << std::endl;
    std::cout << "vehicle_name : " << vehicle_name << std::endl;
    std::cout << "Sensors: " << std::endl;

    for(size_t i = 0; i < sensors.size();i++)
        std::cout << i << " - " << sensors[i] << std::endl;

    std::cout << "distance : " << distance << std::endl;
    std::cout << "last_lat : " << latStart << std::endl;
    std::cout << "last_lon : " << lonStart << std::endl;
    std::cout << "date : " << date << std::endl;
    std::cout << "duration : " << duration << std::endl;
    
    return Log(log_name, vehicle_name, sensors, distance, latStart, lonStart, date, duration);
}

void
getDataFiles(const char* directory, std::vector<std::string> &result) {

    try
    {
        Directory dir(directory);
        const char* fname = 0;

        std::string str = directory;
        str += "/Data.lsf.gz";
        const char* fileName =  str.c_str();

        while ((fname = dir.readEntry(Directory::RD_FULL_NAME)))
        {
            struct stat s;
            if(stat(fname,&s) == 0)
            {
                if( s.st_mode & S_IFDIR )
                {
                    //it's a directory
                    getDataFiles(fname, result);
                }
                else if( s.st_mode & S_IFREG )
                {
                    //it's a file
                    if (std::strcmp(fname, fileName) == 0) {
                        //append to result
                        result.push_back(fname);
                        continue;
                    }
                }
            }
            else
            {
                //error
            }
        }
    }
    catch (...)
    { }
}

void
addToDataBase(Database::Connection* db, Log log) {
    //DUNE::Database::Connection db(database, DUNE::Database::Connection::CF_CREATE);
    //db.beginTransaction();
    DUNE::Database::Statement insertionLog(c_insert_log_stmt, *db);
    insertionLog << log.name
              << log.vehicle
              << log.distance
              << log.latStart
              << log.lonStart
              << log.date
              << "no"
              << "yes"
              << log.duration;
    insertionLog.execute();

    for(size_t i = 0; i < log.sensors.size(); i++) {

        DUNE::Database::Statement insertionLogSensor(c_insert_log_sensor_stmt, *db);
        insertionLogSensor << log.name
                           << log.sensors[i];
        insertionLogSensor.execute();
    }

    //db.commit();

    //test database
    DUNE::Database::Statement query("SELECT * FROM LOG", *db);

    while (query.execute()) {
        std::string name;
        std::string vehicle;
        double distance;
        double latStart;
        double lonStart;
        std::time_t date;
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

        std::cout << std::endl;
        std::cout << "::: LOG DATA BASE ::::" << std::endl;
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

    DUNE::Database::Statement query1("SELECT * FROM SENSOR", *db);
    while (query1.execute()) {
        std::string name;
        query1 >> name;
        std::cout << "sensor name : " << name << std::endl;
    }

    DUNE::Database::Statement query2("SELECT * FROM LOG_SENSOR", *db);
    while (query2.execute()) {
        std::string log_name, sensor;
        query2 >> log_name >> sensor;
        std::cout << "log name : " << log_name << std::endl;
        std::cout << "sensor name : " << sensor << std::endl;
    }
}

int
prepareDatabase(Database::Connection* db) {

    // Create log table
    db->execute(c_log_table_stmt);

    // Create sensor table and initialize them
    db->execute(c_sensor_table_stmt);

    for(size_t i = 0; i < NSENSORS; i++)
    {
        Database::Statement sensor_insert(c_insert_sensor_stmt, *db);
        sensor_insert << sensorsList[i] ;
        sensor_insert.execute();
    }

    // Create log_sensor table
    db->execute(c_log_sensor_table_stmt);

    db->commit();
}



int
main(int32_t argc, char** argv) {

    // home/ana/workspace/lsts/build/log/lauv-noptilus-2/20180709/142145_cmd-lauv-noptilus-2/Data.lsf.gz /home/ana/workspace/lsts/database.db
    if (argc <= 2) {
        std::cerr << "Usage: " << argv[0] << " <path_directory> " << "<path_database/database.db>" << std::endl;
        return 1;
    }

    //prepare database
    Database::Connection* db = new Database::Connection(argv[2], Database::Connection::CF_CREATE);
    db->beginTransaction();

    if (prepareDatabase(db) == 1) {
        std::cerr << "Error while preparing the database" << std::endl;
        return -1;
    }
    std::vector<std::string> result;
    getDataFiles(argv[1], result);

    for (size_t i = 0; i < result.size(); i++) {
        std::cout << std::endl << result[i] << std::endl;

        // get log information
        Log log = getLog(result[i]);

        // add to database (...)
        addToDataBase(db, log);
    }

    return 0;
}

