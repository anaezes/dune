//***************************************************************************
// Copyright 2007-2019 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Ana Santos                                                        *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Vision
{
  //! Device Driver for the FLIR Duo camera
  namespace Flir
  {
    using DUNE_NAMESPACES;

     enum Index
     {
      //! Identification code.
      ID_CODE = 0,
      //! Status code.
      STATUS_CODE = 1,
      //! Instruction number.
      INST_NUMBER = 2,
      //! Instruction number.
      INST_LENGTH = 4,
      //! CRC check (checksum).
      CRC1 = 6,
      //! Body
      BODY = 8
    };

    enum Status_code
    {
      //! Success.
      SUCCESS = 0x00,
      //! Operation error.
      OP_ERROR = 0x02,
      //! Instruction number is incorrect.
      INSTRUCTION_INCORRECT = 0x03,
      //! Parameter is illegal.
      ILLEGAL_PARAM = 0x04,
      //! CRC1 check code error.
      CRC1_CHECK_ERROR = 0x05,
      //! CRC2 check code error.
      CRC2_CHECK_ERROR = 0x06,
      //! File could not be found.
      FILE_NOT_FOUND = 0x07,
      //! Device is busy and can not respond.
      BUSY = 0x08
    };

    enum Instruction
    {
      //! Camera status request.
      HEARTBEAT_REQ = 0x0000,
      //! Camera status response.
      HEARTBEAT_RES = 0x0001,
      //! Time synchronization request.
      TIME_SYNC_REQ = 0x0002,
      //! Time synchronization response.
      TIME_SYNC_RES = 0x0003,
      //! Set camera GPS information.
      GPS_INF_REQ = 0x0004,
      //! Recording status request.
      REC_STATUS_REQ = 0x1004,
      //! Recording status response.
      REC_STATUS_RES = 0x1005,
      //! Take pictures start.
      START_PICTURES_REQ = 0x2000,
      //! Take pictures start.
      START_PICTURES_RES = 0x2001,
      //! Take pictures stop.
      STOP_PICTURES_REQ = 0x2002,
      //! Take pictures stop.
      STOP_PICTURES_RES = 0x2003,
      //! Query radiation coefficient of temperature measurement.
      TEMP_QUERY_REQ = 0x5000,
      //! Query radiation coefficient of temperature measurement.
      TEMP_QUERY_RES = 0x5001,
      //! Set radiation coefficient of temperature measurement.
      TEMP_SET_REQ = 0x5002,
      //! Set radiation coefficient of temperature measurement.
      TEMP_SET_RES = 0x5003,
      //! Notification for capture start events.
      CAPTURE_NOTIFICATION = 0xE000,
      //! Recording or capturing start notification.
      START_NOTIFICATION = 0xE008,
      //! Recording or capturing stop notification.
      STOP_NOTIFICATION = 0xE006,
      //! Recording or capturing stop notification.
      FILE_DOWNLOAD_REQ = 0xF004,
      //! Recording or capturing stop notification.
      STOP_NOTIF_RES = 0xF005
    };

    struct Arguments
    {
      //! IPv4 address.
      Address addr;

      //! TCP port control (short connection for camara control and parameter configuration).
      unsigned port_control;

      //! TCP port notification (long connection for camera event notification).
      unsigned port_notification;
    };

    //! Return identification command code.
    static const uint8_t c_identification_code = 0x64;
    //! Return request max size.
    static const int c_request_size = 80;
    //! Return response max size.
    static const int c_response_size = 100;
    //! Return heartbeat body request/response size.
    static const uint8_t c_heartbeat_body_size = 0x00;
    //! Return heartbeat command / response size
    static const int c_heartbeat_size = 10;
    //! Return take pictures start body request size.
    static const uint16_t c_pic_start_req_body_size = 0x08;
    //! Return take pictures start body request size.
    static const uint8_t c_pic_start_req_size = 18;
    //! Return take pictures start body response size.
    static const uint8_t c_pic_start_res_size = 14;
    //! Return take pictures stop body request size.
    static const uint8_t c_pic_stop_req_body_size = 0x04;
    //! Return take pictures stop request size.
    static const uint8_t c_pic_stop_req_size = 14;
    //! Return take pictures stop response size.
    static const uint8_t c_pic_stop_res_size = 10;
    //! Return temperature query body request size.
    static const uint8_t c_temp_query_req_body_size = 0x00;
    //! Return temperature query body request size.
    static const uint8_t c_temp_query_req_size = 10;
    //! Return temperature query body response size.
    static const uint8_t c_temp_query_res_size = 18;
    //! Return start notification body size.
    static const uint8_t c_notification_body_size = 0x14;
    //! Return start notification command size.
    static const uint8_t c_notification_size = 30;

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! TCP socket control.
      TCPSocket* m_sock_control;
      //! TCP socket notification.
      TCPSocket* m_sock_notif;
      //! Request buffer
      uint8_t m_request[c_request_size];
      //! Response buffer
      uint8_t m_response[c_response_size];
      //! Heartbeat timer
      Time::Counter<float> m_timer_heartbeat;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_sock_control(NULL),
        m_sock_notif(NULL),
        m_timer_heartbeat(0)
      {
        // Define configuration parameters.
        paramActive(Tasks::Parameter::SCOPE_MANEUVER,
                    Tasks::Parameter::VISIBILITY_USER);

        param("IPv4 Address", m_args.addr)
                .defaultValue("192.168.10.19")
                .description("IP address of the flir camera");

        param("TCP control", m_args.port_control)
                .defaultValue("6000")
                .minimumValue("0")
                .maximumValue("65535")
                .description("TCP port control");

        param("TCP notification", m_args.port_notification)
                .defaultValue("6002")
                .minimumValue("0")
                .maximumValue("65535")
                .description("TCP port notification");

        clearBuffers();
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        m_sock_notif = new TCPSocket();
        m_sock_notif->setNoDelay(true);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        try
        {
          m_sock_notif->connect(m_args.addr, m_args.port_notification);

          heartbeat();
          m_timer_heartbeat.setTop(10);
        }
        catch (std::runtime_error& e)
        {
         throw RestartNeeded(e.what(), 10.0, false);
        }
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(m_sock_notif);
        Memory::clear(m_sock_control);
      }

      //! Open short TCP connection
      void
      openConnection()
      {
        // Open connection
        m_sock_control = new TCPSocket();
        m_sock_control->connect(m_args.addr, m_args.port_control);
      }

      //! Close short TCP connection
      void
      closeConnection()
      {
          Memory::clear(m_sock_control);
      }

      void
      clearBuffers()
      {
        memset(m_request, 0, c_request_size);
        memset(m_response, 0, c_response_size);
      }

      int
      writeCommand(uint8_t size)
      {
        return m_sock_control->write((char*)m_request, size);
      }

      uint8_t
      readResponse()
      {
        int rv = 0;
        try {
            rv = m_sock_control->read((char *) m_response, c_response_size);
        } catch(std::exception& e) {
            err("Error: %s", e.what());
        }

        return rv;
      }

      //! Function to detect the camera online status, the proposed cycle of 5~10 s.
      void
      heartbeat()
      {
        setHeader(HEARTBEAT_REQ, c_heartbeat_body_size);

        openConnection();

        if(writeCommand(c_heartbeat_size) == -1)
        {
           debug("error to write...");
           return;
        }

        if(readResponse() != c_heartbeat_size)
          throw std::runtime_error(DTR("failed to get heartbeat response"));

        if(m_response[INST_NUMBER] != HEARTBEAT_RES || m_response[STATUS_CODE] != SUCCESS)
            debug("failed to get heartbeat response");
        else
            debug("Heartbeat");

        closeConnection();
        clearBuffers();
      }

      void
      temperatureMeasurement()
      {
        setHeader(TEMP_QUERY_REQ, c_temp_query_req_body_size);

        openConnection();

        if(writeCommand(c_temp_query_req_size) == -1)
        {
          debug("error to write...");
          return;
        }

        if(readResponse() != c_temp_query_res_size)
          throw std::runtime_error(DTR("failed to get temperature response"));

        uint16_t instruction;
        mempcpy(&instruction, &m_response[2], 2);

        if(instruction == TEMP_QUERY_RES && m_response[STATUS_CODE] == SUCCESS)
        {
          debug("Success to get temperature response");
          printDataTemperature();
        }

        closeConnection();
        clearBuffers();
      }

      void
      printDataTemperature()
      {
        uint8_t airTemperature = m_response[11] & 0xff;
        if(m_response[8] == 0)
          debug("Air temperature: %d ºC", airTemperature);
        else
          debug("Air temperature: %d ºF", airTemperature);

        if(m_response[9] == 0)
          debug("Measurement points enabled : closed");
        else
          debug("Measurement points enabled : opening");

        uint8_t emissivity = m_response[10] & 0xff;
        debug("Emissivity: %d", emissivity);

        uint8_t skyCondition = m_response[12] & 0xff;
        if(skyCondition == 0)
          debug("Sky Condition: clear skies");
        else if(skyCondition == 25)
          debug("Sky Condition: scattered skies");
        else
          debug("Sky Condition: cloudy skies");

        uint8_t humidity = m_response[13] & 0xff;
        debug("Humidity viewer: %d", humidity);

        uint16_t distance;
        mempcpy(&distance, &m_response[14], 2);
        debug("Subject distance dec: %d", distance);
      }


      //! Take pictures start: can specify a single shot or timer shot (seconds)
      void
      startTakePictures(uint8_t interval, uint8_t format, uint32_t id_picture)
      {
        setHeader(START_PICTURES_REQ, c_pic_start_req_body_size);

        m_request[BODY] = interval;
        m_request[BODY + 1] = format;
        m_request[BODY + 4] = id_picture & 0xff;
        m_request[BODY + 5] = id_picture >> 8;
        m_request[BODY + 6] = id_picture >> 16;
        m_request[BODY + 7] = id_picture >> 24 ;

        // Open connection
        openConnection();

        if(writeCommand(c_pic_start_req_size) == -1)
        {
          debug("error to write...");
          return;
        }

        if(readResponse() != c_pic_start_res_size)
          throw std::runtime_error(DTR("failed to get start take pictures response"));

        uint16_t instruction;
        mempcpy(&instruction, &m_response[2], 2);

        if(instruction != START_PICTURES_RES || m_response[STATUS_CODE] != SUCCESS)
          debug("Fail to get start pictures response");

        closeConnection();
        clearBuffers();
      }


      //! Take pictures stop: can specify a single shot or timer shot (seconds)
      void
      stopTakePictures(uint32_t id_picture)
      {
         setHeader(STOP_PICTURES_REQ, c_pic_stop_req_body_size);

         m_request[BODY] = id_picture & 0xff;
         m_request[BODY+1] = id_picture >> 8;
         m_request[BODY+2] = id_picture >> 16;
         m_request[BODY+3] = id_picture >> 24;

        openConnection();

        if(writeCommand(c_pic_stop_req_size) == -1)
        {
          debug("error to write...");
          return;
        }

        if(readResponse() != c_pic_stop_res_size)
          throw std::runtime_error(DTR("failed to get stop take pictures response"));

        uint16_t instruction;
        mempcpy(&instruction, &m_response[2], 2);

        if(instruction != STOP_PICTURES_RES || m_response[STATUS_CODE] != SUCCESS)
           debug("Fail to get stop pictures response");

        closeConnection();
        clearBuffers();
      }

      //! Change header according to the command
      void
      setHeader(Instruction inst, uint8_t size)
      {
        m_request[ID_CODE] = c_identification_code;
        m_request[STATUS_CODE] = SUCCESS;
        m_request[INST_NUMBER] = inst & 0xff;
        m_request[INST_NUMBER+1] = inst >> 8;
        m_request[INST_LENGTH] = size;
      }

      //! Activate recording or capturing start notifications
      void
      activatePicturesNotifications()
      {
        setHeader(START_NOTIFICATION, c_notification_body_size);

        // write command
        int wr = m_sock_notif->write((char*)m_request, c_notification_size);
        if(wr == -1)
          debug("error to write...");
      }

      //! Receive and process notifications received
      void
      readNotification()
      {

        try {
          m_sock_notif->read((char *) m_response, c_response_size);
        } catch(std::exception& e) {
          err("Error: %s", e.what());
        }

        uint16_t instruction;
        mempcpy(&instruction, &m_response[2], 2);

        if(instruction == CAPTURE_NOTIFICATION)
        {
          uint8_t errorCode;
          mempcpy(&errorCode, &m_response[9], 1);
          if(errorCode != SUCCESS)
          {
              debug("Error code: %x", errorCode);
              return;
          }

        uint8_t type;
        mempcpy(&type, &m_response[8], 1);

        if(type == 1)
          debug("Success! Type: Image file");
        else
          debug("Success! Type: Video file");

        uint8_t picName[64];
        mempcpy(&picName, &m_response[16], 64);
        debug("Picture name: %s", (char*)picName);
        }
      }


      //! Main loop.
      void
      onMain(void)
      {
        Time::Counter<float> timer_picture;
        uint32_t unique_id = 5000;

        bool first = true;
        bool stop = false;

        temperatureMeasurement();

        activatePicturesNotifications();
        timer_picture.setTop(0.1);

        while (!stopping())
        {
          waitForMessages(1.0);

          if(m_timer_heartbeat.overflow())
          {
            heartbeat();
            m_timer_heartbeat.setTop(10);
          }

          if(first) {
            debug("Send take picture command...");
            startTakePictures(0, 0, unique_id);
            timer_picture.setTop(100);
            first = false;
          }

          if(timer_picture.overflow() && !stop)
          {
            debug("Send stop picture command...");
            stopTakePictures(unique_id);
            stop = true;
          }

          if(Poll::poll(*m_sock_notif, 0.1))
            readNotification();

        }
      }
    };
  }
}

DUNE_TASK
