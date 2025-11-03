#pragma once

// PicoStageDriver.h: A header file for a minimal device adapter

#include "MMDevice.h"
#include "DeviceBase.h"
#include <mutex>






class CPicoHub : public HubBase<CPicoHub>
{
public:
    CPicoHub();
    ~CPicoHub();

    int Initialize();
    int Shutdown();
    void GetName(char* pszName) const;
    bool Busy();

    bool SupportsDeviceDetection(void);
    MM::DeviceDetectionStatus DetectDevice(void);
    int DetectInstalledDevices();

    // property handlers
    int OnPort(MM::PropertyBase* pPropt, MM::ActionType eAct);

    // custom interface for child devices
    bool IsPortAvailable() { return portAvailable_; }

    int GetIntegerFromDevice(const char* command, int channel, int& value, char* errStr);
    int SendIntegerToDevice(const char* command, int channel, int value, char* errStr);
    int IdentifyAxisChannel(const char* axisLabel, int& channel);

    std::mutex& GetLock() { return mutex_; }

private:
    int GetControllerID();
    std::string port_;
    bool portAvailable_;
    bool initialized_;
    std::mutex mutex_;
    static constexpr char termChar_[] = "\n"; // LF
};


class CPicoXYStage : public CXYStageBase<CPicoXYStage>
{
public:
   CPicoXYStage();
   ~CPicoXYStage();

   // Device API
   int Initialize();
   int Shutdown();
   void GetName(char* name) const;
   bool Busy();

   // XYStage API
   int SetPositionSteps(long x, long y);
   int GetPositionSteps(long& x, long& y);
   double GetStepSizeXUm();
   double GetStepSizeYUm();
   int SetOrigin();
   int Move(double velX, double velY);
   int Home();
   int Stop();
   int GetLimitsUm(double& xMin, double& xMax, double& yMin, double& yMax);
   int GetStepLimits(long& xMin, long& xMax, long& yMin, long& yMax);
   int IsXYStageSequenceable(bool& isSequenceable) const { isSequenceable = false; return DEVICE_OK; }

   // action interface
   int OnStepSizeX(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnStepSizeY(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnVelocityX(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnVelocityY(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnAccelX(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnAccelY(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnRemote(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   int GetIntegerFromDevice(const char* command, int channel, int& value);
   int SendIntegerToDevice(const char* command, int channel, int value);

   CPicoHub* hub_;
   bool initialized_;
   int channelX_; // 0-based channel number for X axis
   int channelY_; // 0-based channel number for Y axis
   double stepSizeXUm_;
   double stepSizeYUm_;
   bool motionInProgress_;
};


class CPicoStage : public CStageBase<CPicoStage>
{
public:
   CPicoStage(const char* deviceName);
   ~CPicoStage();

   // Device API
   int Initialize();
   int Shutdown();
   void GetName(char* name) const;
   bool Busy();

   // Stage API
   int SetPositionUm(double pos);
   int GetPositionUm(double& pos);
   int SetPositionSteps(long steps);
   int GetPositionSteps(long& steps);
   double GetStepSizeUm();
   int SetOrigin();
   int Move(double vel);
   int Home();
   int Stop();
   int GetLimits(double& min, double& max);
//   int GetStepLimits(long& xMin, long& xMax, long& yMin, long& yMax);
   bool IsContinuousFocusDrive() const { return false; }
   int IsStageSequenceable(bool& isSequenceable) const { isSequenceable = false; return DEVICE_OK; }

   // action interface
   int OnID(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnVelocity(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnAccel(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnRemote(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
   int GetIntegerFromDevice(const char* command, int channel, int& value);
   int SendIntegerToDevice(const char* command, int channel, int value);

   CPicoHub* hub_;
   std::string id_;
   bool initialized_;
   int channel_; // 0-based channel number for axis
   double stepSizeUm_;
   long originSteps_;
   bool motionInProgress_;
};
