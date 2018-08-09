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
#include <map>

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Database.hpp>

using DUNE_NAMESPACES;

struct Log
{
    std::string name;
    std::string vehicle;
    int year;
    std::vector<std::string> sensors;
    double distance;
    double latStart;
    double lonStart;
    std::time_t date;
    std::string errors;
    std::string warnings;
    double duration;
    double maxDepth;

    Log(const std::string& a_name, const std::string& a_vehicle, const int& year, const std::vector<std::string>& sensors,
        const double& a_distance, const  double& a_latStart, const double &a_lonStart,const  std::time_t& a_date,
        const double& a_duration, const double& a_depth, const std::string& errors, const std::string& warnings):
            name(a_name),
            vehicle(a_vehicle),
            year(year),
            distance(a_distance),
            latStart(a_latStart),
            lonStart(a_lonStart),
            date(a_date),
            duration(a_duration),
            maxDepth(a_depth),
            sensors(sensors),
            errors(errors),
            warnings(warnings)
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
        "year INTEGER NOT NULL,"
        "distTravelled REAL NOT NULL,"
        "startLat REAL NOT NULL,"
        "startLon REAL NOT NULL,"
        "\"date\" INTEGER NOT NULL,"
        "errors text,"
        "warnings text,"
        "duration REAL NOT NULL,"
        "maxDepth REAL NOT NULL"
        ");";

static const char* c_sensor_table_stmt =
        "CREATE TABLE IF NOT EXISTS sensor ( "
        "sensorName text PRIMARY KEY"
        ");";

static const char* c_log_sensor_table_stmt =
        "CREATE TABLE IF NOT EXISTS log_sensor ( "
        "logName text NOT NULL REFERENCES log ON DELETE CASCADE,"
        "sensorName text NOT NULL REFERENCES sensor ON DELETE CASCADE,"
        "PRIMARY KEY (logName, sensorName)"
        ");";

static const char* c_insert_sensor_stmt = "INSERT OR IGNORE INTO sensor VALUES(?)";
static const char* c_insert_log_sensor_stmt = "INSERT OR IGNORE INTO log_sensor VALUES(?,?)";
static const char* c_insert_log_stmt = "INSERT OR IGNORE INTO log VALUES(?,?,?,?,?,?,?,?,?,?,?)";

static const std::string sensorsList[] = {"Ctd",
                                          "Sidescan",
                                          "Imu",
                                          "Multibeam",
                                          "Camera"};

std::set<std::string> sensorsSet(sensorsList, sensorsList + sizeof(sensorsList) / sizeof(sensorsList[0]));


static const std::string vehicles[] = {"lauv-noptilus-1",
                                          "lauv-noptilus-2",
                                          "lauv-noptilus-3",
                                          "lauv-xplore-1",
                                          "lauv-xplore-2",
                                          "lauv-xplore-2",
                                          "lauv-xplore-3",
                                          "lauv-xplore-4",
                                          "lauv-xplore-5",
                                          "lauv-nemo-1",
                                          "lauv-xtreme-2",
                                          "x8-05",
                                          "x8-06",
                                          "x8-07",
                                          "vtol-02"
                                          };

static const int n_vehicles = sizeof(vehicles) / sizeof(vehicles[0]);

static const char* file_logs = "/Data.lsf.gz" ;

std::string
getErrors(std::map<int,std::string > entity_map, std::multimap<int,std::pair<std::string,std::string> > errors_map)
{
    std::string messageError = "";
    std::set<std::string> errors_set;

    for(std::map<int, std::string>::iterator it = entity_map.begin(); it != entity_map.end(); ++it)
    {
        std::pair<std::multimap<int, std::pair<std::string,std::string> >::iterator, std::multimap<int,
                std::pair<std::string,std::string> >::iterator> r = errors_map.equal_range(it->first);

        for (std::multimap<int, std::pair<std::string,std::string> >::iterator it2 = r.first; it2 != r.second; ++it2) {
            std::string message =  it->second + "(" +  it2->second.first + "): " + it2->second.second + "; ";
            errors_set.insert(message);
        }
    }

    for(std::set<std::string>::iterator it = errors_set.begin(); it != errors_set.end(); ++it)
    {
        messageError += *it;
    }

    return messageError;
}


std::string
getWarnings(std::map<int,std::string > entity_map, std::multimap<int,std::string> warnings_map) {
    std::string messageWarning = "";
    std::set<std::string> warnings_set;

    for(std::map<int, std::string>::iterator it = entity_map.begin(); it != entity_map.end(); ++it)
    {
        std::pair<std::multimap<int, std::string>::iterator, std::multimap<int,
                std::string> ::iterator> r = warnings_map.equal_range(it->first);

        for (std::multimap<int, std::string> ::iterator it2 = r.first; it2 != r.second; ++it2) {
            std::string message =  it->second + ": " +  it2->second + "; ";
            warnings_set.insert(message);
        }
    }

    for(std::set<std::string>::iterator it = warnings_set.begin(); it != warnings_set.end(); ++it)
    {
        messageWarning += *it;
    }

    return messageWarning;
}

std::string
getLogName(std::string path, std::string nameFile) {
    std::string result = nameFile;
    result.erase (0,path.size());

    return result;
}

std::string
getVehicleName(std::string logName) {

   int i = 0;
   while(i < n_vehicles) {
        if (logName.find(vehicles[i]) !=std::string::npos)
            return vehicles[i];
        i++;
    }

    return "unknown";
}


Log*
getLog(std::string file, std::string logName) {

    //open file
    std::istream* is = 0;
    Compression::Methods method = Compression::Factory::detect(file.c_str());
    if (method == METHOD_UNKNOWN)
        is = new std::ifstream(file.c_str(), std::ios::binary);
    else
        is = new Compression::FileInput(file.c_str(), method);

    IMC::Message* msg = NULL;

    // current rpms of vehicle
    uint16_t curr_rpm = 0;

    bool got_state = false;
    IMC::EstimatedState estate;

    // latitude and longitude
    double last_lat;
    double last_lon;
    double latStart = 0.0;
    double lonStart = 0.0;

    // Accumulated travelled distance
    double distance = 0.0;

    // Accumulated travelled time
    double duration = 0.0;

    // date
    std::time_t date = 0;
    int year = 0;

    // vehicle name
    uint16_t sys_id = 0xffff;
    std::string vehicle_name = "";

    // sensors
    std::set<std::string> sensors_set;

    // all entity's
    std::map<int,std::string > entity_map;

    // errors
    std::multimap<int,std::pair<std::string,std::string> > errors_map;

    // sensors
    std::multimap<int,std::string > warnings_map;

    double depth = 0.0;

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
                            if(sensorsSet.find((t_sep->name)) != sensorsSet.end())
                                sensors_set.insert(t_sep->name);
                        }
                    }
                }
            }
            else if(msg->getId() == DUNE_IMC_SETENTITYPARAMETERS)
            {
                IMC::SetEntityParameters* etparam = static_cast<IMC::SetEntityParameters*>(msg);

                if(sensorsSet.find((etparam->name)) != sensorsSet.end())
                    sensors_set.insert(etparam->name);
            }
            else if(msg->getId() == DUNE_IMC_DEPTH)
            {
                IMC::Depth* currDepth = static_cast<IMC::Depth*>(msg);
                if(currDepth->value > depth){
                    depth = currDepth->value;
                }
            }
            else if(msg->getId() == DUNE_IMC_ENTITYSTATE)
            {
                IMC::EntityState* entState = static_cast<IMC::EntityState*>(msg);

                if( entState->state == IMC::EntityState::ESTA_NORMAL || entState->state == IMC::EntityState::ESTA_BOOT)
                    continue;

                std::string error;
                std::pair <std::string,std::string> errorDescription;

                if( entState->state == IMC::EntityState::ESTA_FAULT)
                {
                    errorDescription = std::make_pair(std::string("fault"),entState->description);
                }
                else if( entState->state == IMC::EntityState::ESTA_ERROR)
                {
                    errorDescription = std::make_pair(std::string("error"),entState->description);
                }
                else if(entState->state == IMC::EntityState::ESTA_FAILURE)
                {
                    errorDescription = std::make_pair(std::string("failure"),entState->description);
                }

                errors_map.insert(std::pair<int,std::pair<std::string,std::string> >(entState->getSourceEntity(),errorDescription));
            }
            else if(msg->getId() == DUNE_IMC_ENTITYINFO)
            {
                IMC::EntityInfo* entityInfo = static_cast<IMC::EntityInfo*>(msg);
                entity_map.insert(std::pair<int,std::string>(entityInfo->id , entityInfo->label));
            }
            else if(msg->getId() == DUNE_IMC_LOGBOOKENTRY)
            {
                IMC::LogBookEntry* entry= static_cast<IMC::LogBookEntry*>(msg);

                if(entry->type == IMC::LogBookEntry::LBET_WARNING) {
                    warnings_map.insert(std::pair<int,std::string >(entry->getSourceEntity(),entry->text));
                }

            }
            else if (msg->getId() == DUNE_IMC_SIMULATEDSTATE)
            {
                // since it has simulated state let us ignore this log
                delete msg;
                std::cerr << "this is a simulated log";
                return NULL;
            }

            delete msg;
        }
    }
    catch (std::runtime_error& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    delete is;


    // get sensors list
    std::vector<std::string> sensors;
    std::copy(sensors_set.begin(), sensors_set.end(), std::back_inserter(sensors));

    // get errors list
    std::string errors = getErrors(entity_map, errors_map);

    // get warnings list
    std::string warnings = getWarnings(entity_map, warnings_map);

    // get year
    tm utc_tm = *gmtime(&date);
    tm local_tm = *localtime(&date);

    if(date == 0)
        std::istringstream(logName.substr(0,4)) >> year;
    else
         year = local_tm.tm_year + 1900;

    // get vehicle name if don't have already
    if(vehicle_name == "")
        vehicle_name = getVehicleName(logName);


    return new Log(logName, vehicle_name, year, sensors, distance, latStart, lonStart, date, duration, depth, errors, warnings);
}


int
getDataFiles(const char* directory, std::vector<std::string> &result) {

    try
    {
        Directory dir(directory);
        const char* fname = 0;

        std::string str = directory;
        str += file_logs;
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
                    }
                }
            }
            else
            {
                std::cerr << "ERROR: to searching file." << std::endl;
                return -1;
            }
        }
    }
    catch (...) // file
    {
        int len = strlen(directory);
        const char* fileName = &directory[len-12];

        if(std::strcmp(fileName, file_logs) == 0)
            result.push_back(directory);
    }

    return 0;
}


int
addToDataBase(Database::Connection* db, Log* log) {
    try {
        db->beginTransaction();
        DUNE::Database::Statement insertionLog(c_insert_log_stmt, *db);
        insertionLog << log->name
                     << log->vehicle
                     << log->year
                     << log->distance
                     << log->latStart
                     << log->lonStart
                     << log->date
                     << log->errors
                     << log->warnings
                     << log->duration
                     << log->maxDepth;
        insertionLog.execute();

        for (size_t i = 0; i < log->sensors.size(); i++) {

            DUNE::Database::Statement insertionLogSensor(c_insert_log_sensor_stmt, *db);
            insertionLogSensor << log->name
                               << log->sensors[i];
            insertionLogSensor.execute();
        }

        db->commit();
    }
    catch (...) {
        std::cerr << "Error to add statement in database." << std::endl;
        return -1;
    }

     return 0;
}


void
prepareDatabase(Database::Connection* db) {

    // Create log table
    db->execute(c_log_table_stmt);

    // Create sensor table and initialize them
    db->execute(c_sensor_table_stmt);

    for (size_t i = 0; i < sensorsSet.size(); i++) {
       Database::Statement sensor_insert(c_insert_sensor_stmt, *db);
       sensor_insert << sensorsList[i];
       sensor_insert.execute();
    }

    // Create log_sensor table
    db->execute(c_log_sensor_table_stmt);

    db->commit();
}


int
main(int32_t argc, char** argv) {

    if (argc <= 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <path_directory>"
                  << " <path_database/database.db>"
                  << " base to extract log names"
                  << std::endl;
        return 1;
    }

    Database::Connection *db;

    // Prepare database
    try {
        db = new Database::Connection(argv[2], Database::Connection::CF_CREATE);
        db->beginTransaction();
        prepareDatabase(db);
    }
    catch(...) {
        std::cerr << "Error to prepare database." << std::endl;
        return 1;
    }

    std::vector<std::string> result;
    if(getDataFiles(argv[1], result) == -1) {
        std::cerr << "Error while searching files." << std::endl;
        return 1;
    }

    for (size_t i = 0; i < result.size(); i++) {

        std::string logName = getLogName(argv[3], result[i]);
        std::cout << std::endl << std::endl << "logName: " << logName << std::endl;

        // get log information
        Log* log = getLog(result[i], logName);

        if(log == NULL || log->name == "unknown") {
            delete log;
            continue;
        }

        if(addToDataBase(db, log) == -1) {
            return 1;
        }

        delete log;
    }

    return 0;
}

