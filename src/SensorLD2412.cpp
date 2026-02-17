#ifdef PMMODULE
#ifdef HF_SERIAL

#include "SensorLD2412.h"
#include <Arduino.h>
#include <OpenKNX.h>

SensorLD2412::SensorLD2412(uint16_t iMeasureTypes, TwoWire* iWire)
    : SensorLD2412(iMeasureTypes, iWire, 0) {}

SensorLD2412::SensorLD2412(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress)
{
    pMeasureTypes |= Pres | Distance;
}

std::string SensorLD2412::logPrefix()
{
    return "Sensor<LD2412>";
}

uint8_t SensorLD2412::getSensorClass()
{
    return SENS_HKLD2412;
}

void SensorLD2412::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
            
        case Calibrate:
            uartGetPacket();
            startupLoop();
            
            if (mStartupState == LD2412_START_FINISHED)
                pSensorState = Finalize;
            break;
            
        case Finalize:
            uartGetPacket();
            
            // Exit config mode and resume normal operation
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

void SensorLD2412::startupLoop()
{
    switch (mStartupState)
    {
        case LD2412_START_INIT:
            // Wait for any data frame to confirm sensor is alive
            if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                mStartupState = LD2412_START_WAIT_DATA;
                logDebugP("LD2412 startup: waiting for data");
            }
            break;
            
        case LD2412_START_WAIT_DATA:
            // Once we receive data, sensor is alive
            if (mTargetState != 0xFF || delayCheck(pSensorStateDelay, 3000))
            {
                pSensorStateDelay = millis();
                mStartupState = LD2412_START_ENABLE_CONFIG;
                logDebugP("LD2412 startup: enabling config mode");
                setConfigMode(true);
            }
            break;
            
        case LD2412_START_ENABLE_CONFIG:
            if (delayCheck(pSensorStateDelay, 200))
            {
                pSensorStateDelay = millis();
                mStartupState = LD2412_START_QUERY_VERSION;
                queryVersion();
            }
            break;
            
        case LD2412_START_QUERY_VERSION:
            if (!mModuleVersion.empty())
            {
                pSensorStateDelay = millis();
                mStartupState = LD2412_START_VERSION_RECEIVED;
                logInfoP("LD2412 version: %s", mModuleVersion.c_str());
            }
            else if (delayCheck(pSensorStateDelay, 1000))
            {
                // Retry
                pSensorStateDelay = millis();
                queryVersion();
            }
            break;
            
        case LD2412_START_VERSION_RECEIVED:
            if (delayCheck(pSensorStateDelay, 100))
            {
                pSensorStateDelay = millis();
                mStartupState = LD2412_START_QUERY_CONFIG;
                sendCommand(LD2412_CMD_QUERY_BASIC_CONF);
            }
            break;
            
        case LD2412_START_QUERY_CONFIG:
            if (delayCheck(pSensorStateDelay, 500))
            {
                pSensorStateDelay = millis();
                mStartupState = LD2412_START_FINISHED;
                logInfoP("LD2412 startup complete");
            }
            break;
            
        case LD2412_START_FINISHED:
            // Done, will transition to Finalize
            break;
    }
}

void SensorLD2412::uartGetPacket()
{
    uint8_t rxByte;
    unsigned long lastDataReceived = millis();
    
    switch (mPacketState)
    {
        case LD2412_GET_SYNC_STATE:
            // Wait for a valid header
            while (mPacketState == LD2412_GET_SYNC_STATE && 
                   HF_SERIAL.available() > 0 && 
                   HF_SERIAL.readBytes(&rxByte, 1) == 1)
            {
                lastDataReceived = millis();
                mBuffer[mBufferIndex++] = rxByte;
                
                if (mBufferIndex == LD2412_HEADER_FOOTER_SIZE)
                {
                    if (validateHeaderFooter(LD2412_HEADER_DATA, mBuffer))
                    {
                        mPacketType = LD2412_DATA_FRAME;
                        mPacketState = LD2412_GET_PACKET_DATA;
                        break;
                    }
                    else if (validateHeaderFooter(LD2412_HEADER_COMMAND, mBuffer))
                    {
                        mPacketType = LD2412_COMMAND_RESPONSE;
                        mPacketState = LD2412_GET_PACKET_DATA;
                        break;
                    }
                    else
                    {
                        // Shift buffer left and continue searching
                        memmove(mBuffer, mBuffer + 1, LD2412_HEADER_FOOTER_SIZE - 1);
                        mBufferIndex--;
                    }
                }
            }
            
            if (delayCheck(lastDataReceived, 100))
            {
                // Timeout - reset
                mBufferIndex = 0;
                mPacketState = LD2412_GET_SYNC_STATE;
            }
            break;
            
        case LD2412_GET_PACKET_DATA:
            // Read packet data until footer found
            while (mPacketState == LD2412_GET_PACKET_DATA && 
                   HF_SERIAL.available() > 0 && 
                   HF_SERIAL.readBytes(&rxByte, 1) == 1 &&
                   mBufferIndex < LD2412_MAX_BUFFER_SIZE)
            {
                lastDataReceived = millis();
                mBuffer[mBufferIndex++] = rxByte;
                
                // Check for footer
                if (mBufferIndex >= LD2412_HEADER_FOOTER_SIZE * 2)
                {
                    const uint8_t* footer = (mPacketType == LD2412_DATA_FRAME) ? 
                                           LD2412_FOOTER_DATA : LD2412_FOOTER_COMMAND;
                    
                    if (validateHeaderFooter(footer, mBuffer + mBufferIndex - LD2412_HEADER_FOOTER_SIZE))
                    {
                        mPacketState = LD2412_PROCESS_PACKET_STATE;
                        break;
                    }
                }
            }
            
            if (delayCheck(lastDataReceived, 100) || mBufferIndex >= LD2412_MAX_BUFFER_SIZE)
            {
                // Timeout or buffer full - reset
                mBufferIndex = 0;
                mPacketState = LD2412_GET_SYNC_STATE;
            }
            break;
            
        case LD2412_PROCESS_PACKET_STATE:
            processPacket();
            mBufferIndex = 0;
            mPacketState = LD2412_GET_SYNC_STATE;
            break;
    }
}

void SensorLD2412::processPacket()
{
    if (mPacketType == LD2412_DATA_FRAME)
    {
        processDataFrame();
    }
    else if (mPacketType == LD2412_COMMAND_RESPONSE)
    {
        processCommandResponse();
    }
}

void SensorLD2412::processDataFrame()
{
    // Minimum data frame size: 4 header + 2 length + 1 data header + target data + 1 footer + 1 check + 4 footer
    if (mBufferIndex < 20)
    {
        logDebugP("LD2412 data frame too short: %d bytes", mBufferIndex);
        return;
    }
    
    // Verify data header at offset 7
    if (mBuffer[7] != LD2412_DATA_HEADER)
    {
        logDebugP("LD2412 invalid data header: 0x%02X", mBuffer[7]);
        return;
    }
    
    // Extract target state (offset 8)
    uint8_t newTargetState = mBuffer[LD2412_TARGET_STATES];
    
    // Extract moving target data
    int16_t newMovingDistance = bytesToInt16(mBuffer[LD2412_MOVING_TARGET_LOW], 
                                             mBuffer[LD2412_MOVING_TARGET_HIGH]);
    uint8_t newMovingEnergy = mBuffer[LD2412_MOVING_ENERGY];
    
    // Extract still target data
    int16_t newStillDistance = bytesToInt16(mBuffer[LD2412_STILL_TARGET_LOW], 
                                            mBuffer[LD2412_STILL_TARGET_HIGH]);
    uint8_t newStillEnergy = mBuffer[LD2412_STILL_ENERGY];
    
    // Extract detection distance
    int16_t newDetectionDistance = bytesToInt16(mBuffer[LD2412_DETECTION_DISTANCE_LOW],
                                                mBuffer[LD2412_DETECTION_DISTANCE_HIGH]);
    
    // Update state if changed
    bool stateChanged = false;
    
    if (mTargetState != newTargetState)
    {
        mTargetState = newTargetState;
        stateChanged = true;
        
        const char* stateStr;
        switch (mTargetState)
        {
            case 0: stateStr = "None"; break;
            case 1: stateStr = "Moving"; break;
            case 2: stateStr = "Still"; break;
            case 3: stateStr = "Moving+Still"; break;
            default: stateStr = "Unknown"; break;
        }
        logDebugP("LD2412 target state: %s", stateStr);
    }
    
    mMovingTargetDistance = newMovingDistance;
    mMovingTargetEnergy = newMovingEnergy;
    mStillTargetDistance = newStillDistance;
    mStillTargetEnergy = newStillEnergy;
    mDetectionDistance = newDetectionDistance;
    
    if (stateChanged)
    {
        logDebugP("LD2412 data: Moving=%dcm(%d) Still=%dcm(%d) Detect=%dcm", 
                  mMovingTargetDistance, mMovingTargetEnergy,
                  mStillTargetDistance, mStillTargetEnergy,
                  mDetectionDistance);
    }
}

void SensorLD2412::processCommandResponse()
{
    // Minimum ACK size: 4 header + 2 length + 1 cmd + 1 status + 2 reserved + 4 footer = 14 bytes
    if (mBufferIndex < 14)
    {
        logDebugP("LD2412 ACK too short: %d bytes", mBufferIndex);
        return;
    }
    
    uint8_t command = mBuffer[LD2412_ACK_COMMAND];
    uint8_t status = mBuffer[LD2412_ACK_COMMAND_STATUS];
    bool success = (status == 0x01);
    
    const char* statusStr = success ? "OK" : "FAIL";
    
    switch (command)
    {
        case LD2412_CMD_ENABLE_CONF:
            logDebugP("LD2412 ACK: Enable config (%s)", statusStr);
            break;
            
        case LD2412_CMD_DISABLE_CONF:
            logDebugP("LD2412 ACK: Disable config (%s)", statusStr);
            break;
            
        case LD2412_CMD_QUERY_VERSION:
            if (success && mBufferIndex >= 18)
            {
                // Version is at offset 12-17 (6 bytes)
                char versionStr[20];
                snprintf(versionStr, sizeof(versionStr), "%u.%02X.%02X%02X%02X%02X",
                        mBuffer[13], mBuffer[12], mBuffer[17], mBuffer[16], 
                        mBuffer[15], mBuffer[14]);
                mModuleVersion = versionStr;
                logDebugP("LD2412 ACK: Version %s", versionStr);
            }
            else
            {
                logDebugP("LD2412 ACK: Query version (%s)", statusStr);
            }
            break;
            
        case LD2412_CMD_QUERY_BASIC_CONF:
            logDebugP("LD2412 ACK: Query basic config (%s)", statusStr);
            if (success && mBufferIndex >= 22)
            {
                mMaxMovingGate = mBuffer[12];
                mMaxStillGate = mBuffer[14];
                mTimeout = bytesToInt16(mBuffer[16], mBuffer[17]);
                logDebugP("LD2412 config: MaxMove=%d MaxStill=%d Timeout=%d", 
                          mMaxMovingGate, mMaxStillGate, mTimeout);
            }
            break;
            
        case LD2412_CMD_BLUETOOTH:
            logInfoP("LD2412 ACK: Bluetooth (%s)", statusStr);
            break;
            
        case LD2412_CMD_RESTART:
            logInfoP("LD2412 ACK: Restart (%s)", statusStr);
            break;
            
        default:
            logDebugP("LD2412 ACK: Command 0x%02X (%s)", command, statusStr);
            break;
    }
}

void SensorLD2412::sendCommand(uint8_t command, const uint8_t* params, uint8_t paramLen)
{
    // Write frame header
    HF_SERIAL.write(LD2412_HEADER_COMMAND, LD2412_HEADER_FOOTER_SIZE);
    
    // Calculate and write length (command + params)
    uint8_t length = 2;  // Command is 2 bytes (low, high)
    if (params != nullptr)
        length += paramLen;
    
    uint8_t lenCmd[4] = {length, 0x00, command, 0x00};
    HF_SERIAL.write(lenCmd, 4);
    
    // Write parameters if any
    if (params != nullptr && paramLen > 0)
        HF_SERIAL.write(params, paramLen);
    
    // Write frame footer
    HF_SERIAL.write(LD2412_FOOTER_COMMAND, LD2412_HEADER_FOOTER_SIZE);
    
    logTraceP("LD2412 TX: cmd=0x%02X len=%d", command, paramLen);
    
    delay(30);  // Command processing delay
}

void SensorLD2412::setConfigMode(bool enable)
{
    uint8_t command = enable ? LD2412_CMD_ENABLE_CONF : LD2412_CMD_DISABLE_CONF;
    sendCommand(command);
    delay(50);  // Config mode switching delay
}

void SensorLD2412::setBluetooth(bool enable)
{
    logInfoP("LD2412 set Bluetooth: %s", enable ? "ON" : "OFF");
    
    setConfigMode(true);
    
    uint8_t param[2] = {static_cast<uint8_t>(enable ? 0x01 : 0x00), 0x00};
    sendCommand(LD2412_CMD_BLUETOOTH, param, sizeof(param));
    
    setConfigMode(false);
}

void SensorLD2412::rebootSensorSoft()
{
    logInfoP("LD2412 soft reboot");
    
    setConfigMode(true);
    sendCommand(LD2412_CMD_RESTART);
    delay(100);
    
    // Restart startup sequence
    mModuleVersion = "";
    mTargetState = 0xFF;  // Sentinel value
    pSensorState = Calibrate;
    mStartupState = LD2412_START_INIT;
}

void SensorLD2412::queryVersion()
{
    sendCommand(LD2412_CMD_QUERY_VERSION);
}

bool SensorLD2412::getPresence()
{
    return (mTargetState != 0);
}

int16_t SensorLD2412::getDistance()
{
    // Return detection distance, or moving/still distance if available
    if (mDetectionDistance > 0)
        return mDetectionDistance;
    
    if (mTargetState & LD2412_MOVE_BITMASK)
        return mMovingTargetDistance;
    
    if (mTargetState & LD2412_STILL_BITMASK)
        return mStillTargetDistance;
    
    return 0;
}

float SensorLD2412::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Pres:
            return getPresence() ? 1.0f : 0.0f;
            
        case Distance:
            return static_cast<float>(getDistance());
            
        default:
            return 0.0f;
    }
}

int16_t SensorLD2412::bytesToInt16(uint8_t lowByte, uint8_t highByte)
{
    return (int16_t)((highByte << 8) | lowByte);
}

bool SensorLD2412::validateHeaderFooter(const uint8_t* headerFooter, const uint8_t* buffer)
{
    return memcmp(headerFooter, buffer, LD2412_HEADER_FOOTER_SIZE) == 0;
}

#endif // HF_SERIAL
#endif // PMMODULE
