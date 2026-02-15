#pragma once
#ifdef PMMODULE
#ifdef HF_SERIAL
#include "Sensor.h"
#include <string>

#define HKLD2412_HEADER_FOOTER_SIZE 4
#define HKLD2412_TOTAL_GATES 14

// LD2412 Command bytes (in command mode, framed by FD FC FB FA / 04 03 02 01)
#define HKLD2412_CMD_ENABLE_CONF 0xFF
#define HKLD2412_CMD_DISABLE_CONF 0xFE
#define HKLD2412_CMD_ENABLE_ENG 0x62
#define HKLD2412_CMD_DISABLE_ENG 0x63
#define HKLD2412_CMD_BASIC_CONF 0x02
#define HKLD2412_CMD_QUERY_BASIC_CONF 0x12
#define HKLD2412_CMD_MOTION_GATE_SENS 0x03
#define HKLD2412_CMD_QUERY_MOTION_GATE_SENS 0x13
#define HKLD2412_CMD_STATIC_GATE_SENS 0x04
#define HKLD2412_CMD_QUERY_STATIC_GATE_SENS 0x14
#define HKLD2412_CMD_QUERY_VERSION 0xA0
#define HKLD2412_CMD_SET_BAUD_RATE 0xA1
#define HKLD2412_CMD_FACTORY_RESET 0xA2
#define HKLD2412_CMD_RESTART 0xA3
#define HKLD2412_CMD_BLUETOOTH 0xA4

// Start configuration enable value
#define HKLD2412_CONF_ENABLE_VALUE 0x0001

// Startup state machine
#define HKLD2412_START_INIT 0
#define HKLD2412_START_SENSOR_ACTIVE 1
#define HKLD2412_START_VERSION_RECEIVED 2
#define HKLD2412_START_CONFIG_READ 3
#define HKLD2412_START_FINISHED 255

class SensorHKLD2412 : public Sensor {
private:
  //! uartGetPacket state machine states.
  enum PacketStates {
    //! Waiting for the synchronisation header bytes
    GET_SYNC_STATE = 0,
    //! Reads message data in our buffer
    GET_PACKET_DATA,
    //! Process the correctly received packet
    PROCESS_PACKET_STATE
  };

  enum PacketType {
    //! Data reporting frame (periodic sensor data)
    DATA_REPORT = 0,
    //! Command ACK frame
    COMMAND_RESPONSE,
  };

  static constexpr uint8_t MAX_BUFFER_LENGTH = 64;

  uint8_t mBuffer[MAX_BUFFER_LENGTH]; // message buffer
  int mBufferIndex = 0;
  PacketStates mPacketState = GET_SYNC_STATE;
  PacketType mPacketType;

  std::string mModuleVersion = "";
  uint8_t mHfSensorStartupState = 0;

  // LD2412 uses different headers for data vs command frames
  // Command frame:  FD FC FB FA ... 04 03 02 01
  // Data frame:     F4 F3 F2 F1 ... F8 F7 F6 F5
  static constexpr uint8_t HEADER_COMMAND[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  static constexpr uint8_t FOOTER_COMMAND[4] = {0x04, 0x03, 0x02, 0x01};
  static constexpr uint8_t HEADER_DATA[4] = {0xF4, 0xF3, 0xF2, 0xF1};
  static constexpr uint8_t FOOTER_DATA[4] = {0xF8, 0xF7, 0xF6, 0xF5};

  // Parsed data from periodic frames
  uint8_t mTargetState = 0;           // 0=no target, 1=moving, 2=still, 3=both
  uint16_t mMovingTargetDistance = 0; // in cm
  uint8_t mMovingTargetEnergy = 0;
  uint16_t mStillTargetDistance = 0; // in cm
  uint8_t mStillTargetEnergy = 0;
  uint16_t mDetectionDistance = 0; // closest target distance in cm

  // Configuration readback
  uint8_t mMaxMovingGate = 8;
  uint8_t mMaxStillGate = 8;
  uint16_t mTimeout = 5; // presence timeout in seconds

  void uartGetPacket();
  bool processPacket();
  void startupLoop();
  void sendCommand(uint8_t command, const uint8_t *param = nullptr,
                   uint8_t paramLength = 0);
  void setConfigMode(bool enable);

protected:
  uint8_t mPresence = 0;
  float mDistance = NO_NUM;          // in meters
  uint8_t getSensorClass() override; // returns unique ID for this sensor type
  void sensorLoopInternal() override;
  bool checkSensorConnection() override;
  float measureValue(MeasureType iMeasureType) override;

public:
  SensorHKLD2412(uint16_t iMeasureTypes, TwoWire *iWire);
  SensorHKLD2412(uint16_t iMeasureTypes, TwoWire *iWire, uint8_t iAddress);
  virtual ~SensorHKLD2412() {}

  void rebootSensorHard();
  void rebootSensorSoft();
  void forceCalibration();
  void setBluetooth(bool enable);

  bool begin() override;
  uint8_t getI2cSpeed() override;
  void showHelp();
  bool processCommand(const std::string iCmd, bool iDebugKo);
  std::string logPrefix() override;
  void switchPower(bool on);
};
#endif
#endif
