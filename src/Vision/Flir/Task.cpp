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
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Ana Santos
  namespace Flir
  {
    using DUNE_NAMESPACES;

     enum Index
     {
      // Identification code.
      ID_CODE = 0,
      // Status code.
      STATUS_CODE = 1,
      // Instruction number.
      INST_NUMBER = 2,
      // Instruction number.
      INST_LENGTH = 4,
      // CRC check (checksum).
      CRC1 = 6,
      // Body
      BODY = 8
    };

    enum Status_code
    {
      // Success.
      SUCCESS = 0x00,
      // Operation error.
      OP_ERROR = 0x02,
      // Instruction number is incorrect.
      INSTRUCTION_INCORRECT = 0x03,
      // Parameter is illegal.
      ILLEGAL_PARAM = 0x04,
      // CRC1 check code error.
      CRC1_CHECK_ERROR = 0x05,
      // CRC2 check code error.
      CRC2_CHECK_ERROR = 0x06,
      // File could not be found.
      FILE_NOT_FOUND = 0x07,
      // Device is busy and can not respond.
      BUSY = 0x08
    };

    enum Instruction
    {
      // Camera status request.
      HEARTBEAT_REQ = 0x0000,
      // Camera status response.
      HEARTBEAT_RES = 0x0001,
      // Time synchronization request.
      TIME_SYNC_REQ = 0x0002,
      // Time synchronization response.
      TIME_SYNC_RES = 0x0003,
      // Set camera GPS information.
      GPS_INF_REQ = 0x0004,
      // Recording status request.
      REC_STATUS_REQ = 0x1004,
      // Recording status response.
      REC_STATUS_RES = 0x1005,
      // Take pictures start.
      START_PICTURES_REQ = 0x2000,
      // Take pictures start.
      START_PICTURES_RES = 0x2001,
      // Take pictures stop.
      STOP_PICTURES_REQ = 0x2002,
      // Take pictures stop.
      STOP_PICTURES_RES = 0x2003,
      // Query radiation coefficient of temperature measurement.
      TEMP_QUERY_REQ = 0x5000,
      // Query radiation coefficient of temperature measurement.
      TEMP_QUERY_RES = 0x5001,
      // Set radiation coefficient of temperature measurement.
      TEMP_SET_REQ = 0x5002,
      // Set radiation coefficient of temperature measurement.
      TEMP_SET_RES = 0x5003,
      // Recording or capturing start notification.
      START_NOTIFICATION = 0xE008,
      // Recording or capturing stop notification.
      STOP_NOTIFICATION = 0xE006,
      // Recording or capturing stop notification.
      FILE_DOWNLOAD_REQ = 0xF004,
      // Recording or capturing stop notification.
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

    static const uint8_t c_identification_code = 0x64;

    // Return request max size.
    static const int c_request_size = 80;
    // Return response max size.
    static const int c_response_size = 22;

    // Return header request/response size.
    static const uint8_t c_header_size = 8;

    // Return heartbeat body request/response size.
    static const uint8_t c_heartbeat_size = 0x00;

    // Return time synchronization body request size.
    static const uint8_t c_time_sync_req_size = 0x08;
    // Return time synchronization body response size.
    static const uint8_t c_time_sync_res_size = 0x00;

    // Return gps information body request size.
    static const uint8_t c_gps_req_size = 0x0c;

    // Return recording status body request size.
    static const uint8_t c_rec_status_req_size = 0x00;
    // Return recording status body response size.
    static const uint8_t c_rec_status_res_size = 0x04;

    // Return take pictures start body request size.
    static const uint8_t c_pic_start_req_size = 0x08;
    // Return take pictures start body response size.
    static const uint8_t c_pic_start_res_size = 0x04;

    // Return take pictures stop body request size.
    static const uint8_t c_pic_stop_req_size = 0x04;
    // Return take pictures stop body response size.
    static const uint8_t c_pic_stop_res_size = 0x00;

    // Return temperature query body request size.
    static const uint8_t c_temp_query_req_size = 0x00;
    // Return temperature query body response size.
    static const uint8_t c_temp_query_res_size = 0x08;

    // Return temperature set body request size.
    static const uint8_t c_temp_set_req_size = 0x08;
    // Return temperature set body response size.
    static const uint8_t c_temp_set_res_size = 0x00;

    // Return start / stop notification body request size.
    static const uint8_t c_notification_size = 0x14;

    // Return file download body request size.
    static const uint8_t c_file_download_req_size = 0x50;
    // Return file download body response size (12+n).
    static const uint8_t c_file_download_res_size = 12;


    struct Task: public DUNE::Tasks::Task
    {
      // TCP socket control.
      TCPSocket* m_sock_control;

      // TCP socket notification.
      TCPSocket* m_sock_notif;

      // Request
      uint8_t m_request[c_request_size];

      // Response
      uint8_t m_response[c_response_size];

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_sock_control(NULL),
        m_sock_notif(NULL)
      {
        param("IPv4 Address", m_args.addr)
                .defaultValue("192.168.10.19")
                .description("IP address of the flir camera");

        param("TCP port control", m_args.port_control)
                .defaultValue("6000")
                .minimumValue("0")
                .maximumValue("65535")
                .description("TCP port control");

        param("TCP port notification", m_args.port_notification)
                .defaultValue("6002")
                .minimumValue("0")
                .maximumValue("65535")
                .description("TCP port notification");

        std::memset(m_request, 0, sizeof(c_request_size));
        std::memset(m_response, 0, sizeof(c_response_size));

      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        m_sock_control = new TCPSocket();
        m_sock_control->setNoDelay(true);

        m_sock_notif = new TCPSocket();
        m_sock_notif->setNoDelay(true);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        try
        {
          m_sock_control->connect(m_args.addr, m_args.port_control);
          heartbeat();

          m_sock_notif->connect(m_args.addr, m_args.port_notification);
          // TODO how to test ?

          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
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
        Memory::clear(m_sock_control);
        Memory::clear(m_sock_notif);
      }


      //! Function to detect the camera online status, the proposed cycle of 5~10 s.
      void
      heartbeat()
      {
        m_request[ID_CODE] = c_identification_code;
        m_request[STATUS_CODE] = SUCCESS;
        m_request[INST_NUMBER] = HEARTBEAT_REQ;
        m_request[INST_LENGTH] = c_heartbeat_size;

        // Write command
        m_sock_control->write((char*)m_request, c_heartbeat_req_size);

        // Read response
        int rv = m_sock->read((char*)m_response, c_response_size);
        if (rv != c_heartbeat_size)
          throw std::runtime_error(DTR("failed to get heartbeat response"));

        if(m_response[INST_NUMBER] == HEARTBEAT_RES
           && m_response[STATUS_CODE] == SUCCESS)
          debug("Success to get heartbeat response");

      }


      //! Take pictures start: can specify a single shot or timer shot (seconds)
      void
      startTakePictures(uint8_t interval, uint8_t format, uint32_t id_picture)
      {
        m_request[ID_CODE] = c_identification_code;
        m_request[STATUS_CODE] = SUCCESS;
        m_request[INST_NUMBER] = START_PICTURES_REQ;
        m_request[INST_LENGTH] = c_pic_start_req_size;

        m_request[BODY]   = interval >> 8;
        m_request[BODY+1] = interval & 0xff;

        m_request[BODY+2] = format >> 8;
        m_request[BODY+3] = format & 0xff;

        m_request[BODY+6] = id_picture >> 24;
        m_request[BODY+5] = id_picture >> 16;
        m_request[BODY+6] = id_picture >>  8;
        m_request[BODY+7] = id_picture & 0xff;

        // Write command
        m_sock_control->write((char *) m_request, c_pic_start_res_size);

        // Read response
        int rv = m_sock->read((char *) m_response, c_response_size);
        if(rv != c_pic_start_res_size)
          throw std::runtime_error(DTR("failed to get start take pictures response"));

        if(m_response[INST_NUMBER] == START_PICTURES_RES && m_response[STATUS_CODE] == SUCCESS)
          debug("Success to get start pictures response");

        // todo check id
        //uint8 aux[] = "Quiz";
        // memcpy(aux, &m_response[BODY], 4);
        //int i = atoi(aux);

        if(m_response[BODY] == START_PICTURES_RES)
          debug("Success to get start pictures response");
      }


      //! Take pictures stop: can specify a single shot or timer shot (seconds)
      void
      stopTakePictures(uint32_t id_picture) {
        m_request[ID_CODE] = c_identification_code;
        m_request[STATUS_CODE] = SUCCESS;
        m_request[INST_NUMBER] = STOP_PICTURES_REQ;
        m_request[INST_LENGTH] = c_pic_stop_req_size;

        m_request[BODY]   = id_picture >> 24;
        m_request[BODY+1] = id_picture >> 16;
        m_request[BODY+2] = id_picture >>  8;
        m_request[BODY+3] = id_picture & 0xff;

        // Write command
        m_sock_control->write((char *) m_request, c_pic_stop_req_size);

        // Read response
        int rv = m_sock->read((char *) m_response, c_response_size);
        if (rv != c_pic_stop_res_size)
          throw std::runtime_error(DTR("failed to get stop take pictures response"));

        if (m_response[INST_NUMBER] == STOP_PICTURES_RES)
          debug("Success to get stop pictures response");
      }


      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
