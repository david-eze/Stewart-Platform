#include "serial_protocol.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "hardware_config.h"

uint16_t SerialProtocol::computeCRC(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

void doubleToNetwork(double value, uint8_t* buffer) {
    uint64_t* value_ptr = (uint64_t*)&value;
    uint64_t value_int = *value_ptr;
    
    buffer[0] = (value_int >> 56) & 0xFF;
    buffer[1] = (value_int >> 48) & 0xFF;
    buffer[2] = (value_int >> 40) & 0xFF;
    buffer[3] = (value_int >> 32) & 0xFF;
    buffer[4] = (value_int >> 24) & 0xFF;
    buffer[5] = (value_int >> 16) & 0xFF;
    buffer[6] = (value_int >> 8) & 0xFF;
    buffer[7] = value_int & 0xFF;
}

double networkToDouble(const uint8_t* buffer) {
    uint64_t value_int = ((uint64_t)buffer[0] << 56) |
                         ((uint64_t)buffer[1] << 48) |
                         ((uint64_t)buffer[2] << 40) |
                         ((uint64_t)buffer[3] << 32) |
                         ((uint64_t)buffer[4] << 24) |
                         ((uint64_t)buffer[5] << 16) |
                         ((uint64_t)buffer[6] << 8) |
                         (uint64_t)buffer[7];
    
    double* value_ptr = (double*)&value_int;
    return *value_ptr;
}

uint16_t htons(uint16_t value) {
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

uint16_t ntohs(uint16_t value) {
    return htons(value);
}

uint32_t htonl(uint32_t value) {
    return ((value & 0xFF) << 24) |
           ((value & 0xFF00) << 8) |
           ((value >> 8) & 0xFF00) |
           ((value >> 24) & 0xFF);
}

uint32_t ntohl(uint32_t value) {
    return htonl(value);
}

SerialProtocol::SerialProtocol() {
    rx_index = 0;
    receive_complete = false;
    tx_sequence = 0;
    rx_sequence = 0;
    
    packets_received = 0;
    packets_sent = 0;
    crc_errors = 0;
    framing_errors = 0;
    
    memset(rx_buffer, 0, MAX_PACKET_SIZE);
    memset(tx_buffer, 0, MAX_PACKET_SIZE);
}

void SerialProtocol::init() {
    rx_index = 0;
    receive_complete = false;
    tx_sequence = 0;
    rx_sequence = 0;
    
    packets_received = 0;
    packets_sent = 0;
    crc_errors = 0;
    framing_errors = 0;
}

void SerialProtocol::processByte(uint8_t byte) {
    if (receive_complete) {
        return;
    }
    
    if (rx_index == 0 && byte != PACKET_START_MARKER) {
        framing_errors++;
        return;
    }
    
    if (rx_index < PACKET_MAX_SIZE) {
        rx_buffer[rx_index++] = byte;
    } else {
        rx_index = 0;
        framing_errors++;
        return;
    }
    
    if (rx_index >= sizeof(PacketHeader) + 1) {
        PacketHeader* header = (PacketHeader*)rx_buffer;
        uint16_t total_length = sizeof(PacketHeader) + header->packet_length + 1;
        
        if (rx_index >= total_length) {
            if (rx_buffer[total_length - 1] == PACKET_END_MARKER) {
                uint16_t computed_crc = computeCRC(rx_buffer + sizeof(PacketHeader), header->packet_length);
                uint16_t received_crc = (header->payload_crc_h << 8) | header->payload_crc_l;
                
                if (computed_crc == received_crc) {
                    receive_complete = true;
                    packets_received++;
                    
                    processPacket(*header, rx_buffer + sizeof(PacketHeader), header->packet_length);
                } else {
                    crc_errors++;
                    rx_index = 0;
                }
            } else {
                framing_errors++;
                rx_index = 0;
            }
        }
    }
}

void SerialProtocol::processPacket(const PacketHeader& header, const uint8_t* payload, uint16_t payload_length) {
    rx_sequence = header.sequence_number;
    
    sendAcknowledgment(header.sequence_number, true);
}

void SerialProtocol::sendAcknowledgment(uint16_t sequence_number, bool ack_type) {
    tx_buffer[0] = PACKET_START_MARKER;
    tx_buffer[1] = ack_type ? PACKET_ACK : PACKET_NACK;
    tx_buffer[2] = PROTOCOL_VERSION;
    tx_buffer[3] = 0;
    tx_buffer[4] = 0;
    tx_buffer[5] = (sequence_number >> 8) & 0xFF;
    tx_buffer[6] = sequence_number & 0xFF;
    tx_buffer[7] = 0;
    tx_buffer[8] = 0;
    tx_buffer[9] = PACKET_END_MARKER;
    
    Serial.write(tx_buffer, 10);
    packets_sent++;
}

bool SerialProtocol::isPacketComplete() const {
    return receive_complete;
}

bool SerialProtocol::getCommand(CommandPayload& command) {
    if (!receive_complete) {
        return false;
    }
    
    PacketHeader* header = (PacketHeader*)rx_buffer;
    
    if (header->packet_type != PACKET_COMMAND) {
        resetReceive();
        return false;
    }
    
    if (header->packet_length >= sizeof(CommandPayload)) {
        const uint8_t* payload = rx_buffer + sizeof(PacketHeader);
        
        command.command_type = payload[0];
        command.target_x = networkToDouble(payload + 1);
        command.target_y = networkToDouble(payload + 9);
        command.target_z = networkToDouble(payload + 17);
        command.target_phi = networkToDouble(payload + 25);
        command.target_theta = networkToDouble(payload + 33);
        command.target_psi = networkToDouble(payload + 41);
        command.velocity = networkToDouble(payload + 49);
        command.acceleration = networkToDouble(payload + 57);
        command.duration = networkToDouble(payload + 65);
        command.timestamp = ntohl(*(uint32_t*)(payload + 73));
        command.flags = payload[77];
        
        resetReceive();
        return true;
    }
    
    resetReceive();
    return false;
}

bool SerialProtocol::sendTelemetry(const TelemetryPayload& telemetry) {
    uint16_t index = 0;
    
    tx_buffer[index++] = PACKET_START_MARKER;
    tx_buffer[index++] = PACKET_TELEMETRY;
    tx_buffer[index++] = PROTOCOL_VERSION;
    
    uint16_t payload_length = sizeof(TelemetryPayload);
    tx_buffer[index++] = (payload_length >> 8) & 0xFF;
    tx_buffer[index++] = payload_length & 0xFF;
    
    tx_sequence++;
    tx_buffer[index++] = (tx_sequence >> 8) & 0xFF;
    tx_buffer[index++] = tx_sequence & 0xFF;
    
    uint8_t* payload_start = &tx_buffer[index];
    
    doubleToNetwork(telemetry.current_x, &tx_buffer[index]); index += 8;
    doubleToNetwork(telemetry.current_y, &tx_buffer[index]); index += 8;
    doubleToNetwork(telemetry.current_z, &tx_buffer[index]); index += 8;
    doubleToNetwork(telemetry.current_phi, &tx_buffer[index]); index += 8;
    doubleToNetwork(telemetry.current_theta, &tx_buffer[index]); index += 8;
    doubleToNetwork(telemetry.current_psi, &tx_buffer[index]); index += 8;
    
    for (int i = 0; i < 6; i++) {
        doubleToNetwork(telemetry.actuator_lengths[i], &tx_buffer[index]); index += 8;
    }
    
    for (int i = 0; i < 6; i++) {
        doubleToNetwork(telemetry.actuator_velocities[i], &tx_buffer[index]); index += 8;
    }
    
    doubleToNetwork(telemetry.control_loop_period_us, &tx_buffer[index]); index += 8;
    doubleToNetwork(telemetry.ik_computation_time_us, &tx_buffer[index]); index += 8;
    
    uint32_t loop_counter = htonl(telemetry.loop_counter);
    memcpy(&tx_buffer[index], &loop_counter, 4); index += 4;
    
    tx_buffer[index++] = telemetry.error_flags;
    tx_buffer[index++] = telemetry.system_status;
    tx_buffer[index++] = telemetry.actuator_errors;
    tx_buffer[index++] = telemetry.flags;
    
    uint32_t timestamp = htonl(telemetry.timestamp);
    memcpy(&tx_buffer[index], &timestamp, 4); index += 4;
    
    for (int i = 0; i < 8; i++) {
        tx_buffer[index++] = 0;
    }
    
    uint16_t crc = computeCRC(payload_start, payload_length);
    tx_buffer[sizeof(PacketHeader) - 2] = (crc >> 8) & 0xFF;
    tx_buffer[sizeof(PacketHeader) - 1] = crc & 0xFF;
    
    tx_buffer[index++] = PACKET_END_MARKER;
    
    Serial.write(tx_buffer, index);
    packets_sent++;
    
    return true;
}

bool SerialProtocol::sendError(const ErrorPayload& error) {
    uint16_t index = 0;
    
    tx_buffer[index++] = PACKET_START_MARKER;
    tx_buffer[index++] = PACKET_ERROR;
    tx_buffer[index++] = PROTOCOL_VERSION;
    
    uint16_t payload_length = sizeof(ErrorPayload);
    tx_buffer[index++] = (payload_length >> 8) & 0xFF;
    tx_buffer[index++] = payload_length & 0xFF;
    
    tx_sequence++;
    tx_buffer[index++] = (tx_sequence >> 8) & 0xFF;
    tx_buffer[index++] = tx_sequence & 0xFF;
    
    uint8_t* payload_start = &tx_buffer[index];
    
    tx_buffer[index++] = error.error_code;
    tx_buffer[index++] = error.error_severity;
    
    uint16_t error_count = htons(error.error_count);
    memcpy(&tx_buffer[index], &error_count, 2); index += 2;
    
    uint32_t error_timestamp = htonl(error.error_timestamp);
    memcpy(&tx_buffer[index], &error_timestamp, 4); index += 4;
    
    memcpy(&tx_buffer[index], error.error_message, 32); index += 32;
    
    for (int i = 0; i < 16; i++) {
        tx_buffer[index++] = 0;
    }
    
    uint16_t crc = computeCRC(payload_start, payload_length);
    tx_buffer[sizeof(PacketHeader) - 2] = (crc >> 8) & 0xFF;
    tx_buffer[sizeof(PacketHeader) - 1] = crc & 0xFF;
    
    tx_buffer[index++] = PACKET_END_MARKER;
    
    Serial.write(tx_buffer, index);
    packets_sent++;
    
    return true;
}

bool SerialProtocol::sendConfig(const ConfigPayload& config) {
    uint16_t index = 0;
    
    tx_buffer[index++] = PACKET_START_MARKER;
    tx_buffer[index++] = PACKET_CONFIG;
    tx_buffer[index++] = PROTOCOL_VERSION;
    
    uint16_t payload_length = sizeof(ConfigPayload);
    tx_buffer[index++] = (payload_length >> 8) & 0xFF;
    tx_buffer[index++] = payload_length & 0xFF;
    
    tx_sequence++;
    tx_buffer[index++] = (tx_sequence >> 8) & 0xFF;
    tx_buffer[index++] = tx_sequence & 0xFF;
    
    uint8_t* payload_start = &tx_buffer[index];
    
    doubleToNetwork(config.max_velocity, &tx_buffer[index]); index += 8;
    doubleToNetwork(config.max_acceleration, &tx_buffer[index]); index += 8;
    doubleToNetwork(config.position_tolerance, &tx_buffer[index]); index += 8;
    doubleToNetwork(config.orientation_tolerance, &tx_buffer[index]); index += 8;
    
    doubleToNetwork(config.pid_kp, &tx_buffer[index]); index += 8;
    doubleToNetwork(config.pid_ki, &tx_buffer[index]); index += 8;
    doubleToNetwork(config.pid_kd, &tx_buffer[index]); index += 8;
    
    uint16_t control_freq = htons(config.control_frequency);
    memcpy(&tx_buffer[index], &control_freq, 2); index += 2;
    
    uint16_t telemetry_freq = htons(config.telemetry_frequency);
    memcpy(&tx_buffer[index], &telemetry_freq, 2); index += 2;
    
    tx_buffer[index++] = config.flags;
    
    for (int i = 0; i < 14; i++) {
        tx_buffer[index++] = 0;
    }
    
    uint16_t crc = computeCRC(payload_start, payload_length);
    tx_buffer[sizeof(PacketHeader) - 2] = (crc >> 8) & 0xFF;
    tx_buffer[sizeof(PacketHeader) - 1] = crc & 0xFF;
    
    tx_buffer[index++] = PACKET_END_MARKER;
    
    Serial.write(tx_buffer, index);
    packets_sent++;
    
    return true;
}

bool SerialProtocol::sendHeartbeat() {
    uint16_t index = 0;
    
    tx_buffer[index++] = PACKET_START_MARKER;
    tx_buffer[index++] = PACKET_HEARTBEAT;
    tx_buffer[index++] = PROTOCOL_VERSION;
    tx_buffer[index++] = 0;
    tx_buffer[index++] = 0;
    
    tx_sequence++;
    tx_buffer[index++] = (tx_sequence >> 8) & 0xFF;
    tx_buffer[index++] = tx_sequence & 0xFF;
    tx_buffer[index++] = 0;
    tx_buffer[index++] = 0;
    tx_buffer[index++] = PACKET_END_MARKER;
    
    Serial.write(tx_buffer, index);
    packets_sent++;
    
    return true;
}

void SerialProtocol::resetReceive() {
    rx_index = 0;
    receive_complete = false;
}

void SerialProtocol::getStatistics(uint32_t& received, uint32_t& sent, 
                                  uint32_t& crc_err, uint32_t& framing_err) {
    received = packets_received;
    sent = packets_sent;
    crc_err = crc_errors;
    framing_err = framing_errors;
}
