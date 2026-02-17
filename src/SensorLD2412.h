#pragma once

#ifdef PMMODULE
#ifdef HF_SERIAL

#include "Sensor.h"
#include <Arduino.h>
#include <string>

// LD2412 Protocol Constants
// Frame headers and footers
static constexpr uint8_t LD2412_HEADER_COMMAND[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static constexpr uint8_t LD2412_FOOTER_COMMAND[4] = {0x04, 0x03, 0x02, 0x01};
static constexpr uint8_t LD2412_HEADER_DATA[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static constexpr uint8_t LD2412_FOOTER_DATA[4] = {0xF8, 0xF7, 0xF6, 0xF5};

// Periodic data frame markers
static constexpr uint8_t LD2412_DATA_HEADER = 0xAA;
static constexpr uint8_t LD2412_DATA_FOOTER = 0x55;
static constexpr uint8_t LD2412_DATA_CHECK = 0x00;

// Header/Footer size
static constexpr uint8_t LD2412_HEADER_FOOTER_SIZE = 4;

// Command definitions (from ESPHome)
#define LD2412_CMD_ENABLE_CONF 0xFF
#define LD2412_CMD_DISABLE_CONF 0xFE
#define LD2412_CMD_ENABLE_ENG 0x62
#define LD2412_CMD_DISABLE_ENG 0x63
#define LD2412_CMD_QUERY_BASIC_CONF 0x12
#define LD2412_CMD_BASIC_CONF 0x02
#define LD2412_CMD_QUERY_VERSION 0xA0
#define LD2412_CMD_QUERY_DISTANCE_RESOLUTION 0x11
#define LD2412_CMD_SET_DISTANCE_RESOLUTION 0x01
#define LD2412_CMD_QUERY_LIGHT_CONTROL 0x1C
#define LD2412_CMD_SET_LIGHT_CONTROL 0x0C
#define LD2412_CMD_SET_BAUD_RATE 0xA1
#define LD2412_CMD_QUERY_MAC_ADDRESS 0xA5
#define LD2412_CMD_FACTORY_RESET 0xA2
#define LD2412_CMD_RESTART 0xA3
#define LD2412_CMD_BLUETOOTH 0xA4
#define LD2412_CMD_MOTION_GATE_SENS 0x03
#define LD2412_CMD_QUERY_MOTION_GATE_SENS 0x13
#define LD2412_CMD_STATIC_GATE_SENS 0x04
#define LD2412_CMD_QUERY_STATIC_GATE_SENS 0x14

// Periodic data byte offsets
enum LD2412_PeriodicData : uint8_t {
    LD2412_DATA_TYPES = 6,
    LD2412_TARGET_STATES = 8,
    LD2412_MOVING_TARGET_LOW = 9,
    LD2412_MOVING_TARGET_HIGH = 10,
    LD2412_MOVING_ENERGY = 11,
    LD2412_STILL_TARGET_LOW = 12,
    LD2412_STILL_TARGET_HIGH = 13,
    LD2412_STILL_ENERGY = 14,
    LD2412_DETECTION_DISTANCE_LOW = 15,
    LD2412_DETECTION_DISTANCE_HIGH = 16,
    LD2412_MOVING_SENSOR_START = 17,
    LD2412_STILL_SENSOR_START = 31,
    LD2412_OUT_PIN_SENSOR = 38,
    LD2412_LIGHT_SENSOR = 45
};

// ACK data byte offsets
enum LD2412_AckData : uint8_t {
    LD2412_ACK_COMMAND = 6,
    LD2412_ACK_COMMAND_STATUS = 7
};

// Target state bitmasks
#define LD2412_MOVE_BITMASK 0x01
#define LD2412_STILL_BITMASK 0x02

// Constants
#define LD2412_TOTAL_GATES 14
#define LD2412_MAX_BUFFER_SIZE 128
#define LD2412_DEFAULT_TIMEOUT 5  // seconds

// Packet state machine states
enum LD2412_PacketState {
    LD2412_GET_SYNC_STATE,
    LD2412_GET_PACKET_DATA,
    LD2412_PROCESS_PACKET_STATE
};

// Packet types
enum LD2412_PacketType {
    LD2412_DATA_FRAME,
    LD2412_COMMAND_RESPONSE
};

// Startup states
enum LD2412_StartupState {
    LD2412_START_INIT,
    LD2412_START_WAIT_DATA,
    LD2412_START_ENABLE_CONFIG,
    LD2412_START_QUERY_VERSION,
    LD2412_START_VERSION_RECEIVED,
    LD2412_START_QUERY_CONFIG,
    LD2412_START_CONFIG_RECEIVED,
    LD2412_START_FINISHED
};

class SensorLD2412 : public Sensor
{
  private:
    // Packet parsing
    uint8_t mBuffer[LD2412_MAX_BUFFER_SIZE];
    uint8_t mBufferIndex = 0;
    LD2412_PacketState mPacketState = LD2412_GET_SYNC_STATE;
    LD2412_PacketType mPacketType;
    
    // Startup state machine
    LD2412_StartupState mStartupState = LD2412_START_INIT;
    
    // Sensor data
    uint8_t mTargetState = 0;  // 0=none, 1=moving, 2=still, 3=both
    int16_t mMovingTargetDistance = 0;  // cm
    uint8_t mMovingTargetEnergy = 0;
    int16_t mStillTargetDistance = 0;   // cm
    uint8_t mStillTargetEnergy = 0;
    int16_t mDetectionDistance = 0;     // cm
    
    // Configuration
    uint8_t mMaxMovingGate = 8;
    uint8_t mMaxStillGate = 8;
    uint8_t mTimeout = LD2412_DEFAULT_TIMEOUT;
    
    // Module info
    std::string mModuleVersion;
    uint8_t mMacAddress[6] = {0, 0, 0, 0, 0, 0};
    
    // Helper methods
    void uartGetPacket();
    void processPacket();
    void processDataFrame();
    void processCommandResponse();
    void sendCommand(uint8_t command, const uint8_t* params = nullptr, uint8_t paramLen = 0);
    void setConfigMode(bool enable);
    void startupLoop();
    int16_t bytesToInt16(uint8_t lowByte, uint8_t highByte);
    bool validateHeaderFooter(const uint8_t* headerFooter, const uint8_t* buffer);
    
  public:
    SensorLD2412(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorLD2412(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    
    std::string logPrefix() override;
    uint8_t getSensorClass() override;
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;
    
    // Additional public methods
    void setBluetooth(bool enable);
    void rebootSensorSoft();
    void queryVersion();
    
    // Sensor value access
    bool getPresence();
    int16_t getDistance();
    uint8_t getTargetState() { return mTargetState; }
};

#endif // HF_SERIAL
#endif // PMMODULE
