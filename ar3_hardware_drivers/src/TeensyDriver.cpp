/**
 * Драйвер взаимодействия с котроллером манипулятора RedAR3 Teensy 4.1.
 * Данная версия драйвера основовыается на репозитории https://github.com/ongdexter/ar3_core.git.
 * Описание внесенных изменения см. в корневом README
 */

#include "ar3_hardware_drivers/TeensyDriver.h"

#include <thread>
#include <chrono>

#define FW_VERSION "0.0.1"

namespace ar3_hardware_drivers {

void TeensyDriver::init(std::string port, int baudrate, int num_joints, std::vector<double>& enc_steps_per_deg)
{
    // @TODO read version from config
    version_ = FW_VERSION;

    // establish connection with teensy board
    boost::system::error_code ec;
    serial_port_.open(port, ec);

    if (ec)
    {
        ROS_WARN("Failed to connect to serial port %s", port.c_str());
        return;
    }
    else
    {
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(static_cast<uint32_t>(baudrate)));
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        ROS_INFO("Successfully connected to serial port %s", port.c_str());
    }
    
    initialised_ = false;
    /*COMMAND UPDATE PARAMS "UP" */ 
    std::string msg = "UPA0B 0 C0D 0 E 0 F 0G1H0I0J1K0L0M1N1O1P1Q0R1S0T0U1V0W0X0Y170Z170a90b42c52d89e165f165g105h105i155j155k44.4444l111.1111m55.5555n42.7266o21.8602p22.2222q10.0r10.0s10.0t10.0u5.0v10.0w0x-90y180z0!0@0#-90$0%90^-90&90*0(169.77)0+0=222.63,0_36.25<64.2>305?0{0}0~0\n";

    while (!initialised_)
    {
        ROS_INFO("Waiting for response from Teensy on port %s", port.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        exchange(msg);
    }
    ROS_INFO("Successfully initialised driver on port %s", port.c_str());

    // initialise joint and encoder calibration
    num_joints_ = num_joints;
    enc_commands_deg_.resize(num_joints_);
    enc_deg_.resize(num_joints_);
    enc_steps_per_deg_.resize(num_joints_);
    enc_calibrations_.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i)
    {
        enc_steps_per_deg_[i] = enc_steps_per_deg[i];
    }

    // get current joint positions
    msg = "RP\n";
    exchange(msg);
    for (int i = 0; i < num_joints_; ++i)
    {
        enc_commands_deg_[i] = enc_deg_[i];
    }
}

TeensyDriver::TeensyDriver()
: serial_port_(io_service_)
{
}

void TeensyDriver::setStepperSpeed(std::vector<double>& max_speed, std::vector<double>& max_accel)
{
    //std::string outMsg = "SS";
    //for (int i = 0, charIdx = 0; i < num_joints_; ++i, charIdx += 2)
    //{
    //    outMsg += 'A' + charIdx;
    //    outMsg += std::to_string(max_speed[i]);
    //    outMsg += 'A' + charIdx + 1;
    //    outMsg += std::to_string(max_accel[i]);
    //}
    //outMsg += "\n";
    //exchange(outMsg);
}

// Update between hardware interface and hardware driver
void TeensyDriver::update(std::vector<double>& pos_commands, std::vector<double>& joint_positions)
{
    // get updated position commands
    jointPosToEncSteps(pos_commands, enc_commands_deg_);
    enc_commands_deg_[0] = -enc_commands_deg_[0]; // invert J1 (why? Dont know...)

    // construct update message
    std::string outMsg = "RJ"; // MOVE IN JOINTS ROTATION
    for (int i = 0; i < num_joints_; ++i)
    {
        outMsg += 'A' + i;
        outMsg += std::to_string(enc_commands_deg_[i]);
    }
    outMsg += "J70.0J80.0J90.0"; // + extrenal joint
    outMsg += "Sp50Ac20Dc20Rm100WFLm111111"; //  + speed + accel + Rm + W + Loop mode
    outMsg += "\n";

    // run the communication with board
    exchange(outMsg);

    // return updated joint_positions
    encStepsToJointPos(enc_deg_ , joint_positions);
    joint_positions[0] = -joint_positions[0]; // invert J1 (why? Dont know...)
}

void TeensyDriver::calibrateJoints()
{
    std::string outMsg = "LLA1B1C1D1E1F1G0H0I0J8K-10L5M-8N-13O5P0Q0R0\n";
    sendCommand(outMsg);
}

void TeensyDriver::getJointPositions(std::vector<double>& joint_positions)
{
    // get current joint positions
    std::string msg = "RP\n"; // REQUEST POSITION
    exchange(msg);
    encStepsToJointPos(enc_deg_, joint_positions);
}

// Send specific commands
void TeensyDriver::sendCommand(std::string outMsg)
{
    exchange(outMsg);
}

// Send msg to board and collect data
void TeensyDriver::exchange(std::string outMsg)
{
    std::string inMsg;
    std::string errTransmit = "";
    std::string errReceive = "";
    
    if (!transmit(outMsg, errTransmit))
    {
        // print err
    }

    if (!receive(inMsg, errReceive))
    {
        // print err
    }
    // parse msg
    std::string header = inMsg.substr(0, 2);
    if (header == "Do") // "Done"
    {
        // init acknowledgement
        checkInit(inMsg);   
    }
    else if (header == "JC")
    {
        // encoder calibration values
        updateEncoderCalibrations(inMsg);
    }
    else if (header.substr(0, 1) == "A") // REQUEST POSITION (response)
    {
        // encoder steps
        ROS_INFO("%s", inMsg.c_str());
        updateEncoderSteps(inMsg);
    }
    else
    {
        // unknown header
        ROS_WARN("Unknown header %s", header.c_str());
    } 
}

bool TeensyDriver::transmit(std::string msg, std::string& err)
{
    boost::system::error_code ec;
    const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

    boost::asio::write(serial_port_, sendBuffer, ec);

    if(!ec)
    {
        return true;
    }
    else
    {
        err = "Error in transmit";
        return false;
    }    
}

bool TeensyDriver::receive(std::string& inMsg, std::string& err)
{
    char c;
    std::string msg = "";
    bool eol = false;
    while (!eol)
    {
        boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                eol = true;
            default:
                msg += c;
        }
    }
    inMsg = msg;
    return true;
}

void TeensyDriver::checkInit(std::string msg)
{
    //std::size_t ack_idx = msg.find("A", 2) + 1;
    //std::size_t version_idx = msg.find("B", 2) + 1;
    //int ack = std::stoi(msg.substr(ack_idx, version_idx));
    if (msg == "Done" || msg == "Done\n")
    {
        ROS_INFO("Response: %s", msg.c_str());
        initialised_ = true;
    }
    else
    {
        //std::string version = msg.substr(version_idx);
        ROS_ERROR("Error response: %s", msg);
    }  
}

void TeensyDriver::updateEncoderCalibrations(std::string msg)
{
    size_t idx1 = msg.find("A", 0) + 1;
    size_t idx2 = msg.find("B", 0) + 1;
    size_t idx3 = msg.find("C", 0) + 1;
    size_t idx4 = msg.find("D", 0) + 1;
    size_t idx5 = msg.find("E", 0) + 1;
    size_t idx6 = msg.find("F", 0) + 1;
    enc_calibrations_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
    enc_calibrations_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
    enc_calibrations_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
    enc_calibrations_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
    enc_calibrations_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
    enc_calibrations_[5] = std::stoi(msg.substr(idx6));

    // @TODO update config file
    ROS_INFO("Successfully updated encoder calibrations");
}

void TeensyDriver::updateEncoderSteps(std::string msg)
{
    size_t idx1 = msg.find("A", 0) + 1;
    size_t idx2 = msg.find("B", 0) + 1;
    size_t idx3 = msg.find("C", 0) + 1;
    size_t idx4 = msg.find("D", 0) + 1;
    size_t idx5 = msg.find("E", 0) + 1;
    size_t idx6 = msg.find("F", 0) + 1;
    /* Joint position recieve in deg */
    //ROS_INFO("%s\n", msg.substr(idx1, idx2 - idx1).c_str());
    enc_deg_[0] = std::stod(msg.substr(idx1, idx2 - idx1));
    enc_deg_[1] = std::stod(msg.substr(idx2, idx3 - idx2));
    enc_deg_[2] = std::stod(msg.substr(idx3, idx4 - idx3));
    enc_deg_[3] = std::stod(msg.substr(idx4, idx5 - idx4));
    enc_deg_[4] = std::stod(msg.substr(idx5, idx6 - idx5));
    enc_deg_[5] = std::stod(msg.substr(idx6));
}

/**
 * It is not required to use
 */
void TeensyDriver::encStepsToJointPos(std::vector<double>& enc_deg, std::vector<double>& joint_positions)
{
    for (int i = 0; i < enc_deg.size(); ++i)
    {
        // convert enc steps to joint deg
        joint_positions[i] = enc_deg[i]; //static_cast<double>(enc_steps[i]) / enc_steps_per_deg_[i];
    }
}

void TeensyDriver::jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<double>& enc_deg)
{
    for (int i = 0; i < joint_positions.size(); ++i)
    {
        // convert joint deg to enc steps
        enc_deg[i] = joint_positions[i]; //static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i]);
    }
}

} // namespace ar3_hardware_drivers
