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
// Utility to compute distance travelled from LSF log files.                *
//***************************************************************************

// ISO C++ 98 headers.
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <map>

// DUNE headers.
#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

//! Size of 872 frame.
const unsigned c_ping_size = 4096;
//! Data points channel
const uint16_t c_data_points_channel = 1000;
//! GPS string file offset
const uint16_t c_gps_string_file_offset = 3200;
//! Number of bytes to previous ping
const uint16_t c_bytes_previous_ping = 8192;
//! File name of data log
const std::string file_logs = "/Data.lsf.gz" ;
//! File name of 872 file
const std::string name_file = "/Data.872";


void
getConstInfo(std::vector<uint8_t> &data) {

    data[0] = '8';
    data[1] = '7';
    data[2] = '2';
    data[3] = '0';

    //Number of bytes that are written to the disk
    data[8] = c_ping_size >> 8;
    data[9] = c_ping_size & 0xff;

    //Data points per channel
    data[10] = c_data_points_channel >> 8;
    data[11] = c_data_points_channel & 0xff;

    //Bytes per data point - always 1
    data[12] = 1;

    //Data point bit depth - always 8
    data[13] = 8;

    //Gps type (GPRMC) and number of strings (1)  - 00100001
    data[14] = 0x21;

    //GPS string file offset - 3200
    data[15] = c_gps_string_file_offset >> 8;
    data[16] = c_gps_string_file_offset & 0xff;

    //Event/Annotation counter
    data[17] = 0;
    data[18] = 0;

    //Exec freq - (medium by default)
    data[45] = 1;

    // Data Gain - (by default)
    data[47] = 40;

    // Channel balance - (by default)
    data[48] = 30;

    // Reserved always 0
    data[66] = 0;
    data[67] = 0;
    data[68] = 0;
    data[69] = 0;

    //Sonar type
    data[70] = 0;

    // 0's fill 1
    char zeroFill1[928];
    memset(zeroFill1, '\0', 928);
    std::memcpy(&data[72], &zeroFill1, 928);

    // 0's fill 2
    char zeroFill2[993];
    memset(zeroFill2, '\0' , 993);
    std::memcpy(&data[3100], &zeroFill2, 993);

    // previous ping
    data[4094] = c_bytes_previous_ping >> 8;
    data[4095] = c_bytes_previous_ping & 0xFF;
}

void
setTimeInfo(std::vector<uint8_t> &data, const uint64_t timestamp){

    // date
    time_t time_stamp = timestamp / 1000;
    struct tm *tm = localtime(&time_stamp);
    char date[12];
    strftime(date, sizeof(date), "%d-%b-%Y", tm);
    std::memcpy(&data[19], &date, 12);
    data[30] = '\0';

    // time
    char time[9];
    strftime(time, sizeof(time), "%H:%M:%S", tm);
    std::memcpy(&data[31], &time, 9);
    data[39] = '\0';

    // Thousandths of seconds
    char buffer[4];
    sprintf(buffer,"%03d",  (int) (timestamp % 1000));

    data[40] = '.';
    data[41] = buffer[0];
    data[42] = buffer[1];
    data[43] = buffer[2];
    data[44] = '\0';
}

std::string
createRMC(const IMC::EstimatedState state)
{
    double time_reference = Math::round(state.getTimeStamp());
    time_t secs = (time_t)time_reference;
    double fraction = time_reference - secs;
    unsigned fsec = fraction * 100;
    Time::BrokenDown bdt(secs);
    std::string stn_str;

    double lat = state.lat;
    double lon = state.lon;

    Coordinates::toWGS84(state, lat, lon);

    std::string lat_nmea = latitudeToNMEA(lat);
    std::string lon_nmea = longitudeToNMEA(lon);

    double vel = Math::norm(state.vx, state.vy);

    NMEAWriter stn("GPRMC");
    stn << String::str("%02u%02u%02u.%02u", bdt.hour, bdt.minutes, bdt.seconds, fsec)
        << "A"
        << lat_nmea
        << lon_nmea
        << vel * DUNE::Units::c_ms_to_knot
        << 0 // azimuth.
        << String::str("%02u%02u%02u", bdt.day, bdt.month, bdt.year - 2000)
        << ""
        << ""
        << "A";

    return stn.sentence();
}


void
setRangeIndex(std::vector<uint8_t> &data, const uint32_t range) {

    uint8_t index = 0x07;

    switch (range)
    {
        case 10:
            index = 0x05;
            break;
        case 20:
            index = 0x06;
            break;
        case 30:
            index = 0x07;
            break;
        case 40:
            index = 0x08;
            break;
        case 50:
            index = 0x09;
            break;
        case 60:
            index = 0x0a;
            break;
        case 80:
            index = 0x0b;
            break;
        case 100:
            index = 0x0c;
            break;
        case 125:
            index = 0x0d;
            break;
        case 150:
            index = 0x0f;
            break;
        case 200:
            index = 0x10;
            break;
    }

    //Range index
    data[46] = index;
}

// GPS Strings
void
setGpsString(std::vector<uint8_t> &data, const IMC::EstimatedState state) {
    char gpsString[100];
    memset(gpsString, 0, 100);
    std::string sentence = createRMC(state);
    sprintf(gpsString, "%s", sentence.c_str());
    std::memcpy(&data[3000], &gpsString, 100);
}

std::vector<uint8_t>
getFileHeader(uint32_t range) {

    std::vector<uint8_t> header;
    header.resize(12, 0);

    header[0] = 'I';
    header[1] = 'V';
    header[2] = 'X';

    // Serial status
    header[3] = 0;

    // Range
    header[4] = range;

    // Frequency
    header[5] = 1;

    // Firmware Version
    header[6] = 0x00;

    //Reserved
    header[7] = 0;
    header[8] = 0;
    header[9] = 0;

    // Number of data bytes
    header[10] = c_data_points_channel >> 8;
    header[11] = c_data_points_channel & 0xFF;

    return header;
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

        if(std::strcmp(fileName, file_logs.c_str()) == 0)
            result.push_back(directory);
    }

    return 0;
}

int
main(int32_t argc, char** argv) {

    if (argc <= 1)
    {
        std::cerr << "Usage: " << argv[0] << " <directory>"
                  << std::endl;
        return 1;
    }

    std::vector<std::string> result;
    if(getDataFiles(argv[1], result) == -1) {
        std::cerr << "Error while searching files." << std::endl;
        return 1;
    }

    IMC::Message* msg = NULL;
    std::vector<uint8_t> data;
    data.resize(c_ping_size, 0);
    getConstInfo(data);

    uint32_t range = 30;
    uint32_t frequency = 770;
    uint32_t time_between_pings = 0;
    double last_time = 0;
    short sound_speed = 1500;

    for (size_t i = 0; i < result.size() ; ++i)
    {
        std::istream* is = 0;
        Compression::Methods method = Compression::Factory::detect(result[i].c_str());
        if (method == METHOD_UNKNOWN)
            is = new std::ifstream(result[i].c_str(), std::ios::binary);
        else
            is = new Compression::FileInput(result[i].c_str(), method);


        std::string directory = result[i].substr(0, result[i].find(file_logs));
        std::cout << "File: " << directory << std::endl;


        // Raw log file
        std::ofstream data_file;
        std::string file_872 = directory + name_file;
        data_file.open(file_872.c_str(), std::ofstream::app | std::ios::binary);

        bool received_sonar_data = false;
        bool received_set_entity_parameters = false;
        bool received_estimated_state = false;

        try {

            while ((msg = IMC::Packet::deserialize(*is)) != 0) {

                if (msg->getId() == DUNE_IMC_SONARDATA)
                {
                    received_sonar_data = true;
                    IMC::SonarData* ptr = static_cast<IMC::SonarData*>(msg);

                    if(ptr->type == IMC::SonarData::ST_SIDESCAN) {

                        // Ping data
                        std::vector<char> ping_data = ptr->data;
                        std::memcpy(&data[1000], &ping_data.at(0), c_data_points_channel * 2);

                        // Frequency
                        frequency = ptr->frequency / 1000;
                        data[49] = frequency >> 8;
                        data[50] = frequency & 0xff;

                        // Timestamp
                       double time = ptr->getTimeStamp();
                       long time_ms = time * 1000;
                       setTimeInfo(data, time_ms);

                       // Repetition rate (time between pings)
                       time_between_pings = time_ms - last_time;
                       data[49] = time_between_pings >> 8;
                       data[50] = time_between_pings & 0xff;

                       last_time = time_ms;
                    }
                }
                else if(msg->getId() == DUNE_IMC_SETENTITYPARAMETERS)
                {
                    received_set_entity_parameters = true;

                    IMC::SetEntityParameters* ptr = static_cast<IMC::SetEntityParameters*>(msg);

                    if(std::strcmp(ptr->name.c_str(), "Sidescan") == 0) {

                        IMC::MessageList<IMC::EntityParameter>::const_iterator it = ptr->params.begin();

                        for (; it != ptr->params.end(); it++ )
                        {
                            if(std::strcmp((*it)->name.c_str(),"Range") == 0) {
                                std::istringstream((*it)->value) >> range;
                                setRangeIndex(data, range);
                                data[71] = range;

                            }
                        }
                    }
                }
                else if (msg->getId() == DUNE_IMC_ESTIMATEDSTATE)
                {
                    if(received_estimated_state)
                        continue;

                    received_estimated_state = true;

                    IMC::EstimatedState* ptr = static_cast<IMC::EstimatedState*>(msg);
                    setGpsString(data, *ptr);
                }
                else if (msg->getId() == DUNE_IMC_SOUNDSPEED)
                {
                    IMC::SoundSpeed* ptr = static_cast<IMC::SoundSpeed*>(msg);
                    sound_speed = ptr->value * 10;
                }

                if(!received_sonar_data || !received_estimated_state)
                    continue;

                data[51] = sound_speed >> 8;
                data[52] = sound_speed & 0xff;

                std::vector<uint8_t> header = getFileHeader(range);
                std::memcpy(&data[53], &header[0], 12);

                // Write to file
                for(size_t i = 0; i < c_ping_size; i++)
                    data_file << data[i] ;

                received_sonar_data = false;
            }

            data_file.close();
        }
        catch (std::runtime_error& e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl;
        }
    }

}