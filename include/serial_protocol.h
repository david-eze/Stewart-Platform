#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>

#define PACKET_START_MARKER    0xAA
#define PACKET_END_MARKER      0x55
#define PACKET_MAX_SIZE        256

#define PROTOCOL_VERSION       1

enum PacketType {
    PACKET_COMMAND = 0x01,
    PACKET_TELEMETRY = 0x02,
    PACKET_ACK = 0x03,
    PACKET_NACK = 0x04,
    PACKET_ERROR = 0x05,
    PACKET_CONFIG = 0x06,
    PACKET_CALIBRATION = 0x07,
    PACKET_HEARTBEAT = 0x08
};

enum CommandType {
    CMD_MOVE_POSE = 0x00,
    CMD_MOVE_RELATIVE = 0x01,
    CMD_HOME = 0x02,
    CMD_EMERGENCY_STOP = 0x03,
    CMD_SET_VELOCITY = 0x04,
    CMD_SET_ACCELERATION = 0x05,
    CMD_GET_TELEMETRY = 0x06,
    CMD_CALIBRATE = 0x07,
    CMD_RESET_ERROR = 0x08,
    CMD_SET_PID = 0x09,
    CMD_GET_CONFIG = 0x0A
};

enum ErrorCode {
    ERROR_NONE = 0x00,
    ERROR_INVALID_PACKET = 0x01,
    ERROR_CRC_MISMATCH = 0x02,
    ERROR_INVALID_COMMAND = 0x03,
    ERROR_OUT_OF_RANGE = 0x04,
    ERROR_SINGULARITY = 0x05,
    ERROR_LIMIT_SWITCH = 0x06,
    ERROR_OVERCURRENT = 0x07,
    ERROR_TIMEOUT = 0x08,
    ERROR_SYSTEM_ERROR = 0x09
};

struct PacketHeader {
    uint8_t start_marker;
    uint8_t packet_type;
    uint8_t protocol_version;
    uint16_t packet_length;
    uint16_t sequence_number;
    uint8_t payload_crc_h;
    uint8_t payload_crc_l;
};

struct CommandPayload {
    uint8_t command_type;
    double target_x;
    double target_y;
    double target_z;
    double target_phi;
    double target_theta;
    double target_psi;
    double velocity;
    double acceleration;
    double duration;
    uint32_t timestamp;
    uint8_t flags;
    uint8_t reserved[7];
};

struct TelemetryPayload {
    double current_x;
    double current_y;
    double current_z;
    double current_phi;
    double current_theta;
    double current_psi;
    
    double actuator_lengths[6];
    double actuator_velocities[6];
    
    double control_loop_period_us;
    double ik_computation_time_us;
    
    uint32_t loop_counter;
    uint8_t error_flags;
    uint8_t system_status;
    uint8_t actuator_errors;
    uint8_t flags;
    
    uint32_t timestamp;
    uint8_t reserved[8];
};

struct ErrorPayload {
    uint8_t error_code;
    uint8_t error_severity;
    uint16_t error_count;
    uint32_t error_timestamp;
    char error_message[32];
    uint8_t reserved[16];
};

struct ConfigPayload {
    double max_velocity;
    double max_acceleration;
    double position_tolerance;
    double orientation_tolerance;
    
    double pid_kp;
    double pid_ki;
    double pid_kd;
    
    uint16_t control_frequency;
    uint16_t telemetry_frequency;
    
    uint8_t flags;
    uint8_t reserved[14];
};

class SerialProtocol {
private:
    uint8_t rx_buffer[PACKET_MAX_SIZE];
    uint16_t rx_index;
    bool receive_complete;
    
    uint8_t tx_buffer[PACKET_MAX_SIZE];
    
    uint16_t tx_sequence;
    uint16_t rx_sequence;
    
    uint32_t packets_received;
    uint32_t packets_sent;
    uint32_t crc_errors;
    uint32_t framing_errors;
    
    uint16_t computeCRC(const uint8_t* data, uint16_t length);
    void processPacket(const PacketHeader& header, const uint8_t* payload, uint16_t payload_length);
    void sendAcknowledgment(uint16_t sequence_number, bool ack_type);

public:
    SerialProtocol();
    
    void init();
    void processByte(uint8_t byte);
    bool isPacketComplete() const;
    bool getCommand(CommandPayload& command);
    bool sendTelemetry(const TelemetryPayload& telemetry);
    bool sendError(const ErrorPayload& error);
    bool sendConfig(const ConfigPayload& config);
    bool sendHeartbeat();
    void resetReceive();
    void getStatistics(uint32_t& received, uint32_t& sent, 
                       uint32_t& crc_err, uint32_t& framing_err);
};

void doubleToNetwork(double value, uint8_t* buffer);
double networkToDouble(const uint8_t* buffer);
uint16_t htons(uint16_t value);
uint16_t ntohs(uint16_t value);
uint32_t htonl(uint32_t value);
uint32_t ntohl(uint32_t value);

#endif
