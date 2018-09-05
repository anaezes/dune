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

#ifndef TRANSPORTS_GSM_ERROR_HPP_INCLUDED_
#define TRANSPORTS_GSM_ERROR_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Transports
{
    namespace GSM {
        static const char* cms_error_1 = "Unassigned (unallocated) number";

        static const char* cms_error_8 = "Operator determined barring";

        static const char* cms_error_10 = "Call barred";

        static const char* cms_error_17 = "Network failure";

        static const char* cms_error_21 = "Short message transfer rejected";

        static const char* cms_error_22 = "Congestion (e.g. no channel, facility busy/congested etc.)";

        static const char* cms_error_27 = "Destination out of service";

        static const char* cms_error_28 = "Unidentified subscriber";

        static const char* cms_error_29 = "Facility rejected";

        static const char* cms_error_30 = "Unknown subscriber";

        static const char* cms_error_38 = "Network out of order";

        static const char* cms_error_41 = "Temporary failure";

        static const char* cms_error_42 = "Congestion (e.g. high traffic)";

        static const char* cms_error_47 = "Resources unavailable, unspecified";

        static const char* cms_error_50 = "Requested facility not subscribed";

        static const char* cms_error_69 = "Requested facility not implemented";

        static const char* cms_error_81 = "Invalid short message transfer reference value";

        static const char* cms_error_95 = "Invalid message, unspecified";

        static const char* cms_error_96 = "Invalid mandatory information";

        static const char* cms_error_97 = "Message type non existent or not implemented";

        static const char* cms_error_98 = "Message not compatible with short message protocol state";

        static const char* cms_error_99 = "Information element non existent or not implemented";

        static const char* cms_error_111 = "Protocol error, unspecified";

        static const char* cms_error_127 = "Interworking, unspecified";

        static const char* cms_error_128 = "Telematic interworking not supported x";

        static const char* cms_error_129 = "Short message Type 0 not supported x x";

        static const char* cms_error_130 = "Cannot replace short message x x ";

        static const char* cms_error_143 = "Unspecified TP PID error x x ";

        static const char* cms_error_144 = "Data coding scheme (alphabet) not supported x";

        static const char* cms_error_145 = "Message class not supported x";

        static const char* cms_error_159 = "Unspecified TP DCS error x x";

        static const char* cms_error_160 = "Command cannot be actioned x";

        static const char* cms_error_161 = "Command unsupported x";

        static const char* cms_error_175 = "Unspecified TP Command error x";

        static const char* cms_error_176 = "TPDU not supported x x";

        static const char* cms_error_192 = "SC busy x";

        static const char* cms_error_193 = "No SC subscription x";

        static const char* cms_error_194 = "SC system failure x ";

        static const char* cms_error_195 = "Invalid SME address x";

        static const char* cms_error_196 = "Destination SME barred x";

        static const char* cms_error_197 = "SM Rejected Duplicate SM x";

        static const char* cms_error_198 = "Unspecified TP PID error x x";

        static const char* cms_error_199 = "TP VP not supported X";

        static const char* cms_error_208 = "SIM SMS storage full x";

        static const char* cms_error_209 = "No SMS storage capability in SIM x";

        static const char* cms_error_210 = "Error in MS x";

        static const char* cms_error_211 = "Memory Capacity Exceeded X";

        static const char* cms_error_212 = "SIM Application Toolkit Busy x x";

        static const char* cms_error_255 = "Unspecified error cause";

        static const char* cms_error_300 = "ME failure";

        static const char* cms_error_301 = "SMS service of ME reserved";

        static const char* cms_error_302 = "operation not allowed";

        static const char* cms_error_303 = "operation not supported ";

        static const char* cms_error_304 = "invalid PDU mode parameter";

        static const char* cms_error_305 = "invalid text mode parameter";

        static const char* cms_error_310 = "SIM not inserted ";

        static const char* cms_error_311 = "SIM PIN required";

        static const char* cms_error_312 = "PH SIM PIN required";

        static const char* cms_error_313 = "SIM failure";

        static const char* cms_error_314 = "SIM busy";

        static const char* cms_error_315 = "SIM wrong";

        static const char* cms_error_316 = "SIM PUK required";

        static const char* cms_error_317 = "SIM PIN2 required";

        static const char* cms_error_318 = "SIM PUK2 required";

        static const char* cms_error_320 = "memory failure";

        static const char* cms_error_321 = "invalid memory index";

        static const char* cms_error_330 = "SMSC address unknown";

        static const char* cms_error_331 = "no network service";

        static const char* cms_error_332 = "network timeout";

        static const char* cms_error_340 = "no +CNMA acknowledgment expected";

        static const char* cms_error_500 = "unknown error";


        const char*
        cms_error_str(const int &cms_code_error) {

            switch(cms_code_error) {
                case 1:
                    return cms_error_1;

                case 8:
                    return cms_error_8;

                case 10:
                    return cms_error_10;

                case 17:
                    return cms_error_17;

                case 21:
                    return cms_error_21;

                case 22:
                    return cms_error_22;

                case 27:
                    return cms_error_27;

                case 28:
                    return cms_error_28;

                case 29:
                    return cms_error_29;

                case 30:
                    return cms_error_30;

                case 38:
                    return cms_error_38;

                case 41:
                    return cms_error_41;

                case 42:
                    return cms_error_42;

                case 47:
                    return cms_error_47;

                case 50:
                    return cms_error_50;

                case 69:
                    return cms_error_69;

                case 81:
                    return cms_error_81;

                case 95:
                    return cms_error_95;

                case 96:
                    return cms_error_96;

                case 97:
                    return cms_error_97;

                case 98:
                    return cms_error_98;

                case 99:
                    return cms_error_99;

                case 111:
                    return cms_error_111;

                case 127:
                    return cms_error_127;

                case 128:
                    return cms_error_128;

                case 129:
                    return cms_error_129;

                case 130:
                    return cms_error_130;

                case 143:
                    return cms_error_143;

                case 144:
                    return cms_error_144;

                case 145:
                    return cms_error_145;

                case 159:
                    return cms_error_159;

                case 160:
                    return cms_error_160;

                case 161:
                    return cms_error_161;

                case 175:
                    return cms_error_175;

                case 176:
                    return cms_error_176;

                case 192:
                    return cms_error_192;

                case 193:
                    return cms_error_193;

                case 194:
                    return cms_error_194;

                case 195:
                    return cms_error_195;

                case 196:
                    return cms_error_196;

                case 197:
                    return cms_error_197;

                case 198:
                    return cms_error_198;

                case 199:
                    return cms_error_199;

                case 208:
                    return cms_error_208;

                case 209:
                    return cms_error_209;

                case 210:
                    return cms_error_210;

                case 211:
                    return cms_error_211;

                case 212:
                    return cms_error_212;

                case 255:
                    return cms_error_255;

                case 300:
                    return cms_error_300;

                case 301:
                    return cms_error_301;

                case 302:
                    return cms_error_302;

                case 303:
                    return cms_error_303;

                case 304:
                    return cms_error_304;

                case 305:
                    return cms_error_305;

                case 310:
                    return cms_error_310;

                case 311:
                    return cms_error_311;

                case 312:
                    return cms_error_312;

                case 313:
                    return cms_error_313;

                case 314:
                    return cms_error_314;

                case 315:
                    return cms_error_315;

                case 316:
                    return cms_error_316;

                case 317:
                    return cms_error_317;

                case 318:
                    return cms_error_318;

                case 320:
                    return cms_error_320;

                case 321:
                    return cms_error_321;

                case 330:
                    return cms_error_330;

                case 331:
                    return cms_error_331;

                case 332:
                    return cms_error_332;

                case 340:
                    return cms_error_340;

                default:
                    return cms_error_500;
            }
        }
    }
}

#endif //DUNE_ERROR_H
