#include <scanner.h>
#include <serial.h>

#ifdef DEBUG
#define DEBUG 1
#else
#define DEBUG 0
#endif

//#include "dtc.h" // Testing
//#include "dtc2.h" // Testing
#include "pid.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

extern "C"
{
#include <unistd.h>
}

uint16_t hex2uint16(char *p);
uint8_t hex2uint8(char *p);

std::string ERROR[] = {
    "ACT ALERT",
    "!ACT ALERT",
    "BUFFER FULL",
    "BUS BUSY",
    "BUS ERROR",
    "CAN ERROR",
    "DATA ERROR",
    "<DATA ERROR",
    "ERR",
    "FB ERROR",
    "LP ALERT",
    "!LP ALERT",
    "LV RESET",
    "NO DATA",
    "<RX ERROR",
    "STOPPED",
    "UNABLE TO CONNECT",
    "SEARCHING"};


class scanner::impl {
    public:
        impl(std::string dev, int baud);
        impl(std::string dev);
        ~impl();

        int sendQuery(unsigned char dataMode, unsigned char pid, std::string& rawResponse);
        int sendQuery(unsigned char dataMode, unsigned char pid);
        int fullQuery(unsigned char mode);
        int dtcErrors();
        void displayDTC();
        float getVoltage();
        int availablePid(); // TODO

    private:
        serial *serialPort;
        void configure();
        void sendCommand(std::string cmd);
        int readCommand(std::string& msg);
        bool isError(std::string msg);
        void setProtocol(unsigned char id);
        bool checkHeader(unsigned char mode, unsigned char pid, std::string response, std::string& data);
        int getData(unsigned char mode, unsigned char pid, std::string response);
        void dtcCodes(int n,std::vector<std::string> &dtcVec);
        std::string dtcMessage(std::string code);
};


/**
    Gets available PID for vehicle. TODO
*/
int scanner::impl::availablePid() {
    std::string resp;
    std::vector<unsigned char> pids;

    for(unsigned char i =0;i<=0x90;i+=0x20)
    {
        if(sendQuery(0x01,i,resp))
        {
            std::cerr << "scanner:availablePid: Error getting response\n";
        }

        for(unsigned char j =i; j<=(0x20+i);j+=0x1)
        {
            ;
        }
    }
    return 0;
}

/**
Constructor and incitialization.
*/
scanner::impl::impl(std::string dev, int baud) {
    try {
        serialPort = new serial(dev, baud, '>');
    } catch(std::exception &e) {
        std::cout << "\n" << e.what() << "\n";
        throw std::runtime_error("[SCANNER] Error communicating with device \n");
    }

    configure();
    if(DEBUG)
        std::cout << "[scanner : scanner] Connected to " << dev << " @ " << baud << " kbps\n";
}

/**
    Destructor
*/
scanner::impl::~impl()
{
    if(serialPort != NULL)
       serialPort->disconnect();
}

void scanner::impl::configure() {
    if(!serialPort->isConnected())
        return ;
    serialPort->write("ATE0\r\n");
    std::string msg;
    readCommand(msg);
}


/**
    Sends PID query to ODB device
*/
int scanner::impl::sendQuery(unsigned char dataMode, unsigned char pid) {
    std::string a;
    return sendQuery(dataMode,pid,a);
}

int scanner::impl::sendQuery(unsigned char dataMode, unsigned char pid, std::string& rawResponse) {
    char cmd[10];
    sprintf(cmd, "%02X%02X\r\n", dataMode, pid);
    sendCommand(cmd);
    std::string resp;
    if(readCommand(resp) != -1) {
        rawResponse = resp;
        return getData(dataMode, pid, resp); // Returns response to command sent
    }
    return -1;
}

/**
    Sends command to ODB device
*/
void scanner::impl::sendCommand(std::string cmd) {
    std::string msg;
    do {
        if(serialPort != NULL) {
            serialPort->write("ATIGN\r\n"); // Checks if device is ready
            readCommand(msg);
        }
        usleep(500);
    } while(msg != "ON");

    if(serialPort != NULL)
        serialPort->write(cmd);

    if(DEBUG)
        std::cout << "[scanner : sendCommand] " << cmd << "\n";
}

/**
    Reads comand from ODB device and writes in msg
*/
int scanner::impl::readCommand(std::string& msg){
    std::string cmd;
    if(serialPort != NULL)
        serialPort->read(cmd);

    if(DEBUG)
        std::cout << "[scanner : readCommand] " << cmd << "\n";

    if(isError(cmd)) {
        return -1;
    } else {
        msg = cmd;
       return 0;
    }
}

/**
    Gets data from sensor
*/
int scanner::impl::getData(unsigned char mode, unsigned char pid, std::string response)
{
    std::string data;
    if(!checkHeader(mode,pid,response,data))
        return -1;

    int result = 0;
    // Calculates real value acording to data and pid
    switch (pid) {
        case PID_RPM:
        case PID_EVAP_SYS_VAPOR_PRESSURE:
            result = hex2uint16((char*)data.c_str()) >> 2;
            break;
        case PID_FUEL_PRESSURE:
            result = hex2uint8((char*)data.c_str()) * 3;
            break;
        case PID_COOLANT_TEMP:
        case PID_INTAKE_TEMP:
        case PID_AMBIENT_TEMP:
        case PID_ENGINE_OIL_TEMP:
            result = (int)hex2uint8((char*)data.c_str()) - 40;
            break;
        case PID_THROTTLE:
        case PID_COMMANDED_EGR:
        case PID_COMMANDED_EVAPORATIVE_PURGE:
        case PID_FUEL_LEVEL:
        case PID_RELATIVE_THROTTLE_POS:
        case PID_ABSOLUTE_THROTTLE_POS_B:
        case PID_ABSOLUTE_THROTTLE_POS_C:
        case PID_ACC_PEDAL_POS_D:
        case PID_ACC_PEDAL_POS_E:
        case PID_ACC_PEDAL_POS_F:
        case PID_COMMANDED_THROTTLE_ACTUATOR:
        case PID_ENGINE_LOAD:
        case PID_ABSOLUTE_ENGINE_LOAD:
        case PID_ETHANOL_FUEL:
        case PID_HYBRID_BATTERY_PERCENTAGE:
            result = (uint16_t)hex2uint8((char*)data.c_str()) * 100 / 255;
            break;
        case PID_MAF_FLOW:
            result = hex2uint16((char*)data.c_str()) / 100;
            break;
        case PID_TIMING_ADVANCE:
            result = (int)(hex2uint8((char*)data.c_str()) / 2) - 64;
            break;
        case PID_DISTANCE: // km
        case PID_DISTANCE_WITH_MIL: // km
        case PID_TIME_WITH_MIL: // minute
        case PID_TIME_SINCE_CODES_CLEARED: // minute
        case PID_RUNTIME: // second
        case PID_FUEL_RAIL_PRESSURE: // kPa
        case PID_ENGINE_REF_TORQUE: // Nm
            result = hex2uint16((char*)data.c_str());
            break;
        case PID_CONTROL_MODULE_VOLTAGE: // V
            result = hex2uint16((char*)data.c_str()) / 1000;
            break;
        case PID_ENGINE_FUEL_RATE: // L/h
            result = hex2uint16((char*)data.c_str()) / 20;
            break;
        case PID_ENGINE_TORQUE_DEMANDED: // %
        case PID_ENGINE_TORQUE_PERCENTAGE: // %
            result = (int)hex2uint8((char*)data.c_str()) - 125;
            break;
        case PID_SHORT_TERM_FUEL_TRIM_1:
        case PID_LONG_TERM_FUEL_TRIM_1:
        case PID_SHORT_TERM_FUEL_TRIM_2:
        case PID_LONG_TERM_FUEL_TRIM_2:
        case PID_EGR_ERROR:
            result = ((int)hex2uint8((char*)data.c_str()) - 128) * 100 / 128;
            break;
        case PID_FUEL_INJECTION_TIMING:
            result = ((int32_t)hex2uint16((char*)data.c_str()) - 26880) / 128;
            break;
        case PID_CATALYST_TEMP_B1S1:
        case PID_CATALYST_TEMP_B2S1:
        case PID_CATALYST_TEMP_B1S2:
        case PID_CATALYST_TEMP_B2S2:
            result = hex2uint16((char*)data.c_str()) / 10 - 40;
            break;
        default:
            result = hex2uint8((char*)data.c_str());
    }
    return result;
}

/**
    Sets capture protocol
*/
void scanner::impl::setProtocol(unsigned char id) {
    sendCommand("ATST" + std::to_string(id));
}

/**
    Circles through all available PID and retrieves all data for a certain MODE
*/
int scanner::impl::fullQuery(unsigned char mode) {
    if(mode != 1) {
        std::cerr << "scanner::impl::fullQuery : wrong mode\n";
        return -1;
    }

    unsigned char n_pid = 0;
    if(mode == 1) {
        n_pid = 0x87; // Available PIDs
    }

    std::vector<int> responses;
    for(unsigned char i = 0; i < n_pid; i+=0x01) {
        std::string rawResponse;
        int res = sendQuery(mode,i,rawResponse);
        if(res == -1) {
            responses.push_back(-1);
        } else {
            std::cout << "scanner::impl::fullQuery: mode: "<< (int)mode << " pid: 0x"<<std::hex<< (int)i << std::dec<<" res: " << res <<" Raw: " << rawResponse <<"\n";
            responses.push_back(res);
        }
    }
    return 0;
}

/**
    Checks if 'msg' is an error message
*/
bool scanner::impl::isError(std::string msg) {
    std::vector<std::string> errors(ERROR, ERROR + 18);
    for(unsigned int i=0; i < errors.size(); i++) {
        if(msg.find(errors[i]) != std::string::npos)
        return true;
    }
    return false;
}

/**
    Check for DTC errors in vehicle. Returns number of errors.
*/
int scanner::impl::dtcErrors() {
    sendCommand("0101\r\n");
    std::string resp;

    if(readCommand(resp) == -1) {
        return -1;
    }

    std::string data;
    if(!checkHeader(0x01,0x01,resp,data))
        return -1;

    unsigned char err = hex2uint8((char*)data.c_str());// Get 2 bytes

    if( ((err&0x80)>>7) == 0x0) // If MSB is 0, there's no error
        return 0;

    return (err&0x7F); // Returns rest of bits
}

/**
    Return actual DTC codes for errors
*/
void scanner::impl::dtcCodes(int n, std::vector<std::string> &dtcVec) {
    sendCommand("03\r\n");
    std::string resp;
    if(readCommand(resp) == -1)
        return ;

    std::string data;
    if(!checkHeader(0x03,-1,resp,data))
        return ;

    std::string code;
    // Get every DTC error code
    for(unsigned int i=0; i<data.size(); i+=6) {
        code = std::string(data,i,5);

        // Formats retrieved code in regular DTC code
        unsigned char MSByte = hex2uint8((char*)code.c_str()) >> 4;// Get most significant byte
        std::string dtc;
        switch(MSByte) {
            case 0:
                dtc = "P0";
                break;
            case 0x1:
                dtc = "P1";
                break;
            case 0x2:
                dtc = "P2";
                break;
            case 0x3:
                dtc = "P3";
                break;
            case 0x4:
                dtc = "C0";
                break;
            case 0x5:
                dtc = "C1";
                break;
            case 0x6:
                dtc = "C2";
                break;
            case 0x7:
                dtc = "C3";
                break;
            case 0x8:
                dtc = "B0";
                break;
            case 0x9:
                dtc = "B1";
                break;
            case 0xA:
                dtc = "B2";
                break;
            case 0xB:
                dtc = "B3";
                break;
            case 0xC:
                dtc = "U0";
                break;
            case 0xD:
                dtc = "U1";
                break;
            case 0xE:
                dtc = "U2";
                break;
            case 0xF:
                dtc = "U3";
                break;
            default:
                ;
        }

        dtc += std::string(code,1,4);
        dtc.erase(std::remove(dtc.begin(), dtc.end(), ' '), dtc.end());
        dtcVec.push_back(dtc);
    }
}

/**
    Checks if header from response is valid according to mode&pid, and writes data part in &data
*/
bool scanner::impl::checkHeader(unsigned char mode, unsigned char pid, std::string response, std::string& data) {
    // Builds header, according to mode and pid
    char cmd[8];
    if(mode == 0x03) // Mode 03's header has only 1 byte
        sprintf(cmd, "4%X",mode);
    else
        sprintf(cmd, "4%X %02X",mode,pid);

    unsigned int pos = response.find(cmd); // Search for header in response msg
    if(pos == std::string::npos)
        return false;
    if(response == "OK") // Not header
        return true;
    if(mode == 0x01 || mode == 0x02)
        data = response.substr(pos+6); // Erase header, leaves data only
    if(mode == 0x03)
        data = response.substr(pos+3);

    return true;
}


/**
    Gets error message for a certain dtc code. ULTRA SLOW.
*/
/*
std::string scanner::impl::dtcMessage(std::string code)
{
    for(unsigned int i=0; i < (sizeof(dtc_code)/sizeof(dtc_code[0])); i++)
    {
        if(dtc_code[i]==code)
            return dtc_message[i];
    }
    return "-";
}
*/

/**
    Display DTC codes and message
*/
void scanner::impl::displayDTC() {
    std::vector<std::string> dtcVec;
    int n = dtcErrors();
    if(n == 0)
        return;

    dtcCodes(n,dtcVec);

    for (int i = 0; i < (int)dtcVec.size(); i++) {
        // Ultra slow, see dtcMessage()
        ;//std::cout<< dtcVec.at(i) << "\t" << dtcMessage(dtcVec.at(i)) << "\n";
    }
}

/**
    Retrieves battery voltage
*/
float scanner::impl::getVoltage() {
    std::string buf;
    sendCommand("ATRV\r\n");
    int n = readCommand(buf);

    if (n == 0) {
        int pos = buf.find('.',0);
        int v1 = 0, v2 = 0, v3 = 0;

        if(buf[pos-2] >= '0' && buf[pos-2]<='9')
            v1 = buf[pos-2]-'0';
        if(buf[pos-1] >= '0' && buf[pos-1]<='9')
            v2 = buf[pos-1]-'0';
        if(buf[pos+1] >= '0' && buf[pos+1]<='9')
            v3 = buf[pos+1]-'0';

        return (v1*10.0 + v2 + v3/10.0);
    }
    return -1.0;
}

uint16_t hex2uint16( char *p) {
    char c = *p;
    uint16_t i = 0;
    for (char n = 0; c && n < 4; c = *(++p)) {
        if (c >= 'A' && c <= 'F')
            c -= 7;
        else if (c>='a' && c<='f')
            c -= 39;
        else if (c == ' ')
            continue;
        else if (c < '0' || c > '9')
            break;
        i = (i << 4) | (c & 0xF);
        n++;
    }
    return i;
}

unsigned char hex2uint8( char *p) {
    unsigned char c1 = *p;
    unsigned char c2 = *(p + 1);
    if (c1 >= 'A' && c1 <= 'F')
        c1 -= 7;
    else if (c1 >='a' && c1 <= 'f')
        c1 -= 39;
    else if (c1 < '0' || c1 > '9')
        return 0;

    if (c2 >= 'A' && c2 <= 'F')
        c2 -= 7;
    else if (c2 >= 'a' && c2 <= 'f')
        c2 -= 39;
    else if (c2 < '0' || c2 > '9')
        return 0;

    return c1 << 4 | (c2 & 0xf);
}


// Public API

scanner::scanner(std::string dev, int baud)
    : m_impl(new scanner::impl(dev,baud))
{
}

scanner::~scanner() {
}

int scanner::sendQuery(unsigned char dataMode, unsigned char pid, std::string& rawResponse) {
    return m_impl->sendQuery(dataMode, pid, rawResponse);
}

int scanner::sendQuery(unsigned char dataMode, unsigned char pid) {
    return m_impl->sendQuery(dataMode, pid);
}

int scanner::fullQuery(unsigned char mode) {
    return m_impl->fullQuery(mode);
}

int scanner::dtcErrors() {
    return m_impl->dtcErrors();
}

void scanner::displayDTC() {
    m_impl->displayDTC();
}

float scanner::getVoltage() {
    return m_impl->getVoltage();
}

int scanner::availablePid() {
    return m_impl->availablePid();
}
