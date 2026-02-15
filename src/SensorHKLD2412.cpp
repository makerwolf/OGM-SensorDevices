#ifdef PMMODULE
#ifdef HF_SERIAL
#include "SensorHKLD2412.h"
#include <Arduino.h>
#include <OpenKNX.h>
#include <Wire.h>

// Static constexpr member definitions
constexpr uint8_t SensorHKLD2412::HEADER_COMMAND[4];
constexpr uint8_t SensorHKLD2412::FOOTER_COMMAND[4];
constexpr uint8_t SensorHKLD2412::HEADER_DATA[4];
constexpr uint8_t SensorHKLD2412::FOOTER_DATA[4];

SensorHKLD2412::SensorHKLD2412(uint16_t iMeasureTypes, TwoWire *iWire)
    : SensorHKLD2412(iMeasureTypes, &Wire, 0) {};

SensorHKLD2412::SensorHKLD2412(uint16_t iMeasureTypes, TwoWire *iWire,
                               uint8_t iAddress)
    : Sensor(iMeasureTypes, &Wire, iAddress) {
  pMeasureTypes |= Pres | Distance;
};

std::string SensorHKLD2412::logPrefix() { return "Sensor<HKLD2412>"; }

uint8_t SensorHKLD2412::getSensorClass() { return SENS_HKLD2412; }

uint8_t SensorHKLD2412::getI2cSpeed() {
  return 4; // 400kHz - not actually used for UART sensor
}

bool SensorHKLD2412::begin() {
  // UART sensor, no I2C init needed
  // HF_SERIAL is initialized in Presence.cpp setup()
  logDebugP("LD2412 sensor begin");
  pSensorState = Calibrate;
  mHfSensorStartupState = HKLD2412_START_INIT;
  return true;
}

bool SensorHKLD2412::checkSensorConnection() {
  // UART sensor - we assume connection is good if we received a version
  return !mModuleVersion.empty();
}

void SensorHKLD2412::sensorLoopInternal() {
  switch (pSensorState) {
  case Wakeup:
    Sensor::sensorLoopInternal();
    break;
  case Calibrate:
    uartGetPacket();
    startupLoop();

    if (mHfSensorStartupState == HKLD2412_START_FINISHED)
      pSensorState = Finalize;
    break;
  case Finalize:
    uartGetPacket();

    // Close config mode and resume normal operation
    setConfigMode(false);
    pSensorState = Running;
    break;
  case Running:
    uartGetPacket();
    break;
  default:
    pSensorStateDelay = millis();
    break;
  }
}

// State machine handles startup behavior of HF sensor
void SensorHKLD2412::startupLoop() {
  switch (mHfSensorStartupState) {
  case HKLD2412_START_INIT: {
    // Wait for any data from sensor to confirm it's alive
    // We look for any data frame (presence data starts immediately on LD2412)
    if (mTargetState != 0xFF) {
      pSensorStateDelay = millis();
      mHfSensorStartupState = HKLD2412_START_SENSOR_ACTIVE;

      // Enter config mode to query version and settings
      setConfigMode(true);
    } else if (delayCheck(pSensorStateDelay, 5000)) {
      // After 5 seconds without data, try entering config mode
      logDebugP("No data from LD2412 after 5s, trying config mode");
      pSensorStateDelay = millis();
      setConfigMode(true);
      mHfSensorStartupState = HKLD2412_START_SENSOR_ACTIVE;
    }
    break;
  }
  case HKLD2412_START_SENSOR_ACTIVE:
    // Communication established, wait for version info
    if (!mModuleVersion.empty()) {
      pSensorStateDelay = millis();
      mHfSensorStartupState = HKLD2412_START_VERSION_RECEIVED;
    } else if (delayCheck(pSensorStateDelay, 2000)) {
      pSensorStateDelay = millis();
      // Query version
      sendCommand(HKLD2412_CMD_QUERY_VERSION);
    }
    break;
  case HKLD2412_START_VERSION_RECEIVED:
    // Got version, query basic config
    if (delayCheck(pSensorStateDelay, 500)) {
      pSensorStateDelay = millis();
      // Query basic configuration (max gates, timeout)
      sendCommand(HKLD2412_CMD_QUERY_BASIC_CONF);
      mHfSensorStartupState = HKLD2412_START_CONFIG_READ;
    }
    break;
  case HKLD2412_START_CONFIG_READ:
    // Config read or timeout
    if (delayCheck(pSensorStateDelay, 2000)) {
      logDebugP("LD2412 startup complete, version: %s", mModuleVersion.c_str());
      logDebugP("Max moving gate: %d, max still gate: %d, timeout: %d s",
                mMaxMovingGate, mMaxStillGate, mTimeout);
      mHfSensorStartupState = HKLD2412_START_FINISHED;
    }
    break;
  }
}

float SensorHKLD2412::measureValue(MeasureType iMeasureType) {
  switch (iMeasureType) {
  case Pres:
    return (float)mPresence;
  case Distance:
    return mDistance;
  default:
    return NO_NUM;
  }
}

// ---------------------------------------------------------------------------
// UART packet reading state machine
// ---------------------------------------------------------------------------
// LD2412 has two frame types:
//   Data report:  F4 F3 F2 F1 [len_lo len_hi] [data...] F8 F7 F6 F5
//   Command ACK:  FD FC FB FA [len_lo len_hi] [data...] 04 03 02 01
// ---------------------------------------------------------------------------
void SensorHKLD2412::uartGetPacket() {
  uint8_t rxByte;
  long lastDataReceived = millis();

  switch (mPacketState) {
  case GET_SYNC_STATE:
    while (mPacketState == GET_SYNC_STATE && HF_SERIAL.available() > 0 &&
           HF_SERIAL.readBytes(&rxByte, 1) == 1) {
      lastDataReceived = millis();

      mBuffer[mBufferIndex] = rxByte;
      mBufferIndex++;

      if (mBufferIndex == HKLD2412_HEADER_FOOTER_SIZE) {
        if (memcmp(mBuffer, HEADER_COMMAND, HKLD2412_HEADER_FOOTER_SIZE) == 0) {
          mPacketType = COMMAND_RESPONSE;
          mPacketState = GET_PACKET_DATA;
          break;
        }
        if (memcmp(mBuffer, HEADER_DATA, HKLD2412_HEADER_FOOTER_SIZE) == 0) {
          mPacketType = DATA_REPORT;
          mPacketState = GET_PACKET_DATA;
          break;
        }
      }

      // Shift buffer if header not matched
      if (mBufferIndex == HKLD2412_HEADER_FOOTER_SIZE) {
        memmove(mBuffer, mBuffer + 1, HKLD2412_HEADER_FOOTER_SIZE - 1);
        mBufferIndex -= 1;
      }
    }

    if (delayCheck(lastDataReceived, 100)) {
      mBufferIndex = 0;
      mPacketState = GET_SYNC_STATE;
    }
    break;

  case GET_PACKET_DATA:
    while (mPacketState == GET_PACKET_DATA && HF_SERIAL.available() > 0 &&
           HF_SERIAL.readBytes(&rxByte, 1) == 1) {
      lastDataReceived = millis();

      if (mBufferIndex < MAX_BUFFER_LENGTH) {
        mBuffer[mBufferIndex] = rxByte;
        mBufferIndex++;
      } else {
        // Buffer overflow - reset
        logDebugP("Buffer overflow, resetting");
        mBufferIndex = 0;
        mPacketState = GET_SYNC_STATE;
        break;
      }

      // Check for matching footer
      if (mBufferIndex >= HKLD2412_HEADER_FOOTER_SIZE +
                              4) // header + at least 2 byte length + footer
      {
        const uint8_t *expectedFooter =
            (mPacketType == COMMAND_RESPONSE) ? FOOTER_COMMAND : FOOTER_DATA;

        if (memcmp(mBuffer + mBufferIndex - HKLD2412_HEADER_FOOTER_SIZE,
                   expectedFooter, HKLD2412_HEADER_FOOTER_SIZE) == 0) {
          mPacketState = PROCESS_PACKET_STATE;
          break;
        }
      }
    }

    if (delayCheck(lastDataReceived, 100)) {
      mBufferIndex = 0;
      mPacketState = GET_SYNC_STATE;
    }
    break;

  case PROCESS_PACKET_STATE:
    processPacket();
    mBufferIndex = 0;
    mPacketState = GET_SYNC_STATE;
    break;
  }
}

// ---------------------------------------------------------------------------
// Process a complete packet (data or command response)
// ---------------------------------------------------------------------------
bool SensorHKLD2412::processPacket() {
  bool result = false;

  switch (mPacketType) {
  case DATA_REPORT: {
    // Data frame: F4 F3 F2 F1 [len_lo len_hi] [type] [head] [target_state]
    // [data...] [check] [check] F8 F7 F6 F5 Minimum frame: 4 header + 2 length
    // + data + 4 footer
    if (mBufferIndex < 12) {
      logDebugP("Data frame too short: %d bytes", mBufferIndex);
      break;
    }

    uint16_t payloadLength = mBuffer[4] | (mBuffer[5] << 8);

    // Data type byte at offset 6
    uint8_t dataType = mBuffer[6];

    // Target state at offset 8 (0=no target, 1=moving, 2=still, 3=both
    // moving+still)
    mTargetState = mBuffer[8];

    // Parse based on data type
    if (dataType == 0x02 || dataType == 0x01) {
      // Standard or engineering mode reporting
      // Moving target: distance at bytes 9-10 (little endian cm), energy at
      // byte 11
      mMovingTargetDistance = mBuffer[9] | (mBuffer[10] << 8);
      mMovingTargetEnergy = mBuffer[11];

      // Still target: distance at bytes 12-13 (little endian cm), energy at
      // byte 14
      mStillTargetDistance = mBuffer[12] | (mBuffer[13] << 8);
      mStillTargetEnergy = mBuffer[14];

      // Detection distance at bytes 15-16 (some firmware versions)
      if (payloadLength > 11) {
        mDetectionDistance = mBuffer[15] | (mBuffer[16] << 8);
      }

      // Derive presence and distance
      bool hasTarget = (mTargetState & 0x03) != 0;
      uint8_t newPresence = hasTarget ? 1 : 0;

      if (newPresence != mPresence) {
        mPresence = newPresence;
        logDebugP("Presence: %s (state=0x%02X)",
                  mPresence ? "detected" : "none", mTargetState);
      }

      // Choose closest target distance
      float newDistance = -1.0;
      if (mTargetState & 0x01) // moving target
      {
        newDistance = mMovingTargetDistance / 100.0f;
      }
      if (mTargetState & 0x02) // still target
      {
        float stillDist = mStillTargetDistance / 100.0f;
        if (newDistance < 0 || stillDist < newDistance)
          newDistance = stillDist;
      }

      if (newDistance != mDistance) {
        mDistance = newDistance;
        if (mDistance > 0)
          logTraceP("Distance: %.2f m (move=%d cm E=%d, still=%d cm E=%d)",
                    mDistance, mMovingTargetDistance, mMovingTargetEnergy,
                    mStillTargetDistance, mStillTargetEnergy);
      }
    }

    result = true;
    break;
  }

  case COMMAND_RESPONSE: {
    // Command ACK: FD FC FB FA [len_lo len_hi] [cmd+0x01] [status (0=ok,
    // 1=fail)] [...data] 04 03 02 01
    if (mBufferIndex < 10) {
      logDebugP("Command response too short: %d bytes", mBufferIndex);
      break;
    }

    uint16_t payloadLength = mBuffer[4] | (mBuffer[5] << 8);
    uint8_t ackCommand = mBuffer[6];
    uint8_t ackStatus = mBuffer[7]; // 0 = success, 1 = failure

    bool success = (ackStatus == 0);
    const char *statusStr = success ? "OK" : "FAIL";

    switch (ackCommand) {
    case HKLD2412_CMD_ENABLE_CONF +
        1: // 0x00 (ACK for enable conf = 0xFF+1=0x00... actually the response
           // cmd is 0xFF)
      logDebugP("ACK: Enable config mode (%s)", statusStr);
      result = true;
      break;

    case HKLD2412_CMD_DISABLE_CONF + 1:
      logDebugP("ACK: Disable config mode (%s)", statusStr);
      result = true;
      break;

    case HKLD2412_CMD_QUERY_VERSION + 1: // 0xA1
    {
      logDebugP("ACK: Query version (%s)", statusStr);
      if (success && payloadLength >= 11) {
        // Version data starts at offset 12 (after
        // header+length+ackCmd+status+...) Format: type(1) + major(1) +
        // minor(4) = 6 bytes starting at offset 10 Actually the ESPHome impl
        // reads 6 bytes of version info starting from offset 8+2
        uint8_t vMajor = mBuffer[12];
        uint8_t vBuild1 = mBuffer[13];
        uint8_t vBuild2 = mBuffer[14];
        uint8_t vBuild3 = mBuffer[15];
        uint8_t vBuild4 = mBuffer[16];
        uint8_t vBuild5 = mBuffer[17];

        char verBuf[32];
        snprintf(verBuf, sizeof(verBuf), "V%d.%02d.%02d%02d%02d%02d", vMajor,
                 vBuild1, vBuild2, vBuild3, vBuild4, vBuild5);
        mModuleVersion = std::string(verBuf);
        logDebugP("Module version: %s", mModuleVersion.c_str());
      }
      result = true;
      break;
    }

    case HKLD2412_CMD_QUERY_BASIC_CONF + 1: // 0x13
    {
      logDebugP("ACK: Query basic config (%s)", statusStr);
      if (success && payloadLength >= 10) {
        // Parse basic config response
        // After ackCmd(1)+status(1)+...
        // ESPHome parsing: max_move at byte 12, max_still at 13, timeout at
        // 14-15
        mMaxMovingGate = mBuffer[12];
        mMaxStillGate = mBuffer[13];
        mTimeout = mBuffer[14] | (mBuffer[15] << 8);
        logDebugP("Config: maxMove=%d, maxStill=%d, timeout=%d", mMaxMovingGate,
                  mMaxStillGate, mTimeout);
      }
      result = true;
      break;
    }

    case HKLD2412_CMD_FACTORY_RESET + 1:
      logDebugP("ACK: Factory reset (%s)", statusStr);
      result = true;
      break;

    case HKLD2412_CMD_RESTART + 1:
      logDebugP("ACK: Restart (%s)", statusStr);
      result = true;
      break;

    case HKLD2412_CMD_BLUETOOTH + 1:
      logDebugP("ACK: Bluetooth (%s)", statusStr);
      result = true;
      break;

    default:
      logDebugP("ACK: Unknown cmd 0x%02X (%s), payload len=%d", ackCommand,
                statusStr, payloadLength);
      result = true;
      break;
    }
    break;
  }

  default:
    logDebugP("Unknown packet type: %d", mPacketType);
    break;
  }

  return result;
}

// ---------------------------------------------------------------------------
// Send a command to the LD2412 module
// Frame format: FD FC FB FA [len_lo len_hi] [cmd] [param...] 04 03 02 01
// ---------------------------------------------------------------------------
void SensorHKLD2412::sendCommand(uint8_t command, const uint8_t *param,
                                 uint8_t paramLength) {
  uint16_t totalLength = 1 + paramLength; // command byte + parameters

  // Write header
  HF_SERIAL.write(HEADER_COMMAND, HKLD2412_HEADER_FOOTER_SIZE);

  // Write length (little endian)
  HF_SERIAL.write((uint8_t)(totalLength & 0xFF));
  HF_SERIAL.write((uint8_t)((totalLength >> 8) & 0xFF));

  // Write command
  HF_SERIAL.write(command);

  // Write parameters
  if (param != nullptr && paramLength > 0) {
    HF_SERIAL.write(param, paramLength);
  }

  // Write footer
  HF_SERIAL.write(FOOTER_COMMAND, HKLD2412_HEADER_FOOTER_SIZE);

  HF_SERIAL.flush();
  logDebugP("Sent command: 0x%02X, param length: %d", command, paramLength);
}

void SensorHKLD2412::setConfigMode(bool enable) {
  if (enable) {
    // Enable config mode: cmd=0xFF, param=0x01, 0x00
    uint8_t param[2] = {0x01, 0x00};
    sendCommand(HKLD2412_CMD_ENABLE_CONF, param, 2);
  } else {
    sendCommand(HKLD2412_CMD_DISABLE_CONF);
  }
}

void SensorHKLD2412::rebootSensorSoft() {
  logDebugP("Soft reboot LD2412");
  setConfigMode(true);
  delay(50);
  sendCommand(HKLD2412_CMD_RESTART);
  delay(100);
}

void SensorHKLD2412::rebootSensorHard() {
  switchPower(false);
  delay(1000);
  switchPower(true);
  logDebugP("Hard reboot LD2412");
}

void SensorHKLD2412::forceCalibration() {
  logInfoP("Force calibration not implemented for HKLD2412");
}

void SensorHKLD2412::setBluetooth(bool enable) {
  logInfoP("Set LD2412 Bluetooth: %s", enable ? "on" : "off");

  // Bluetooth command requires config mode.
  setConfigMode(true);
  delay(50);

  uint8_t param[2] = {static_cast<uint8_t>(enable ? 1 : 0), 0x00};
  sendCommand(HKLD2412_CMD_BLUETOOTH, param, sizeof(param));
  delay(50);

  setConfigMode(false);
}

void SensorHKLD2412::switchPower(bool on) {
  logDebugP("Switch power: %u", on);
  digitalWrite(HF_POWER_PIN, on ? HIGH : LOW);

  if (on) {
    delay(500);
    // Re-initialize startup state machine
    mModuleVersion = "";
    mTargetState = 0xFF; // sentinel to detect first data
    pSensorState = Calibrate;
    mHfSensorStartupState = HKLD2412_START_INIT;
  }
}

void SensorHKLD2412::showHelp() {
  openknx.console.printHelpLine("ld2412 ver", "Show LD2412 version");
  openknx.console.printHelpLine("ld2412 status", "Show LD2412 status");
  openknx.console.printHelpLine("ld2412 reboot", "Soft reboot LD2412 sensor");
}

bool SensorHKLD2412::processCommand(const std::string iCmd, bool iDebugKo) {
  if (iCmd.length() < 6)
    return false;

  if (iCmd.substr(0, 6) != "ld2412")
    return false;

  if (iCmd.length() >= 10 && iCmd.substr(7, 3) == "ver") {
    logInfoP("LD2412 version: %s", mModuleVersion.c_str());
    if (iDebugKo)
      openknx.console.writeDiagnoseKo("LD2412 %s", mModuleVersion.c_str());
    return true;
  } else if (iCmd.length() >= 13 && iCmd.substr(7, 6) == "status") {
    logInfoP("LD2412 state=%d, pres=%d, dist=%.2f m, moveE=%d, stillE=%d",
             mTargetState, mPresence, mDistance, mMovingTargetEnergy,
             mStillTargetEnergy);
    if (iDebugKo)
      openknx.console.writeDiagnoseKo("P=%d D=%.1f", mPresence, mDistance);
    return true;
  } else if (iCmd.length() >= 13 && iCmd.substr(7, 6) == "reboot") {
    rebootSensorSoft();
    return true;
  }

  return false;
}

#endif
#endif
