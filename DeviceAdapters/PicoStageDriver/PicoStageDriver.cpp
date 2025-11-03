// PicoStageDriver.cpp: A minimal device adapter

#include "PicoStageDriver.h"

#include "ModuleInterface.h"

#include <cstring>
#include <string>
#include <sstream>


const char* g_PicoHubName = "Pico-Hub";
const char* g_PicoXYStageName = "Pico-XYStage";
const char* g_PicoZStageName = "Pico-ZStage";
const char* g_PicoAuxStageName = "Pico-AuxStage";


//////////////////////////////////////////////////////////////////////////////
// Error codes
//
#define ERR_BOARD_NOT_FOUND 101
#define ERR_PORT_OPEN_FAILED 102
#define ERR_NO_PORT_SET 103
#define ERR_NO_DEVICE_DETECTED 104
#define ERR_INVALID_NUMBER_OF_DEVICES 105
#define ERR_INVALID_RESPONSE 106
#define ERR_INVALID_RETURN_VAL 107
#define ERR_INVALID_AXIS_LABEL 108
#define ERR_DYNAMIC_DESCRIPTION 109


using namespace std;



///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData()
{
   RegisterDevice(g_PicoHubName, MM::HubDevice, "Hub (required)");
   RegisterDevice(g_PicoXYStageName, MM::XYStageDevice, "XY Stage");
   RegisterDevice(g_PicoZStageName, MM::StageDevice, "Z Stage");
   RegisterDevice(g_PicoAuxStageName, MM::StageDevice, "Aux Stage");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    if (std::strcmp(deviceName, g_PicoHubName) == 0)
    {
       return new CPicoHub();
    }
    if (std::strcmp(deviceName, g_PicoXYStageName) == 0)
    {
       return new CPicoXYStage();
    }
    if (std::strcmp(deviceName, g_PicoZStageName) == 0)
    {
        return new CPicoStage(g_PicoZStageName);
    }
    if (std::strcmp(deviceName, g_PicoAuxStageName) == 0)
    {
       return new CPicoStage(g_PicoAuxStageName);
    }
    return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////
// CPicoHub implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~
//
CPicoHub::CPicoHub() : 
   initialized_(false),
   portAvailable_(false)
{
   InitializeDefaultErrorMessages();

   SetErrorText(ERR_BOARD_NOT_FOUND, "Did not find a Pico board with the correct ID. Is the Pico connected to this serial port?");
   SetErrorText(ERR_PORT_OPEN_FAILED, "Failed opening Pico USB device.");
   SetErrorText(ERR_NO_PORT_SET, "Hub Device not found. The Pico Hub device is needed to create this device.");
   SetErrorText(ERR_NO_DEVICE_DETECTED, "No device was found on the Pico hub.");
   SetErrorText(ERR_INVALID_NUMBER_OF_DEVICES, "Invalid number of channels (allowed: 1..4).");
   SetErrorText(ERR_INVALID_RESPONSE, "Invalid response from the Pico in response to a query.");
   SetErrorText(ERR_INVALID_RETURN_VAL, "Invalid return value from the Pico in response to a query.");
   SetErrorText(ERR_INVALID_AXIS_LABEL, "Invalid label for a stage axis (allowed: X, Y, Z, and Aux).");
   SetErrorText(ERR_DYNAMIC_DESCRIPTION, "TBD dynamically later...");

   CPropertyAction* pAct = new CPropertyAction(this, &CPicoHub::OnPort);
   CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

}


CPicoHub::~CPicoHub()
{
    Shutdown();
}

void CPicoHub::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_PicoHubName);
}


int CPicoHub::Initialize()
{
   if (initialized_) return DEVICE_OK;
   
   // Name
   int ret = CreateProperty(MM::g_Keyword_Name, g_PicoHubName, MM::String, true);
   if (DEVICE_OK != ret) return ret;

   // Give the Pico some time after opening the serial port
   CDeviceUtils::SleepMs(300);

   const std::lock_guard<std::mutex> lock(mutex_);

   // Check that we have a controller:
   PurgeComPort(port_.c_str());
   ret = GetControllerID();
   if (DEVICE_OK != ret) return ret;

   ret = UpdateStatus();
   if (DEVICE_OK != ret) return ret;

   initialized_ = true;
   return DEVICE_OK;
}


int CPicoHub::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}


bool CPicoHub::Busy()
{
    return false;
}


bool CPicoHub::SupportsDeviceDetection(void)
{
    return true;
}


MM::DeviceDetectionStatus CPicoHub::DetectDevice(void)
{
   if (initialized_) return MM::CanCommunicate;

   // all conditions must be satisfied...
   MM::DeviceDetectionStatus result = MM::Misconfigured;
   char answerTO[MM::MaxStrLength];

   try
   {
      std::string portLowerCase = port_;
      for (std::string::iterator its = portLowerCase.begin(); its != portLowerCase.end(); ++its)
      {
         *its = (char)tolower(*its);
      }
      // ensure we’ve been provided with a valid serial port device name
      if (0 < portLowerCase.length() && 0 != portLowerCase.compare("undefined") && 0 != portLowerCase.compare("unknown"))
      {
         result = MM::CanNotCommunicate;
         // record the default answer time out
         GetCoreCallback()->GetDeviceProperty(port_.c_str(), "AnswerTimeout", answerTO);

         // device specific default communication parameters
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_Handshaking, "Off");
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_BaudRate, "115200");
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_StopBits, "1");
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_AnswerTimeout, "300.0");
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_DelayBetweenCharsMs, "0");
         MM::Device* pS = GetCoreCallback()->GetDevice(this, port_.c_str());
         pS->Initialize();
         // give the Pico some time after opening the serial port
         CDeviceUtils::SleepMs(300);
         const std::lock_guard<std::mutex> lock(mutex_);
         PurgeComPort(port_.c_str());
         int ret = GetControllerID();
         if (DEVICE_OK != ret)
         {
            LogMessageCode(ret, true);
         }
         else
         {
            // to succeed must reach here....
            result = MM::CanCommunicate;
         }
         pS->Shutdown();
         // always restore the AnswerTimeout to the default
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_AnswerTimeout, answerTO);

      }
   }
   catch (...)
   {
      LogMessage("Exception in DetectDevice!", false);
   }

   return result;
}


int CPicoHub::DetectInstalledDevices()
{
   int ret = DEVICE_OK;
   // Pico Hub supports up to 4 devices of 5 types: 0->Undef, 1->X, 2->Y, 3->Z, 4->Aux
   std::vector<std::string> chName = { "Undef", "X", "Y", "Z", "Aux"};
   int axType; // 0->Undef, 1->X, 2->Y, 3->Z, 4->Aux
   std::map<std::string, int> channelNumber;

   if (MM::CanCommunicate == DetectDevice())
   {
      // get the number of attached devices
      int numDevices = 0;
      ret = GetIntegerFromDevice("PC_NDEV", -2, numDevices, NULL); // axis <-1 is ignored
      if (ret != DEVICE_OK) return ret;
      if (numDevices<1) return ERR_NO_DEVICE_DETECTED;
      if (numDevices>4) return ERR_INVALID_NUMBER_OF_DEVICES; // Pico Hub supports 1-4 devices
      // get the axis type for each device
      for (int idx = 0; idx < numDevices; idx++)
      {
         ret = GetIntegerFromDevice("MP_TAXI", idx, axType, NULL);
         if (ret != DEVICE_OK) return ret;
         if (axType < 0 || axType >= chName.size()) {
            LogMessage("Pico Hub: Unsupported axis type detected", true);
         }
         else if (channelNumber.find(chName[axType]) != channelNumber.end()) {
            // the axis type already exists in the map
            LogMessage("Pico Hub: Duplicate axis type detected", true);
         } else {
            // add the axis type to the map
            channelNumber[chName[axType]] = idx;
         }
      }

      // Now we can create the devices based on the detected axis types
      if ((channelNumber.find("X") != channelNumber.end())
         && (channelNumber.find("Y") != channelNumber.end()))
      {
          // we have an XY stage
         MM::Device* pXYStage = ::CreateDevice(g_PicoXYStageName);
         if (pXYStage)
         {
            AddInstalledDevice(pXYStage);
         }
      }
      if (channelNumber.find("Z") != channelNumber.end())
      {
         // we have an Z stage
         MM::Device* pZStage = ::CreateDevice(g_PicoZStageName);
         if (pZStage)
         {
            AddInstalledDevice(pZStage);
         }
      }
      if (channelNumber.find("Aux") != channelNumber.end())
      {
         // we have an Aux stage
         MM::Device* pAuxStage = ::CreateDevice(g_PicoAuxStageName);
         if (pAuxStage)
         {
            AddInstalledDevice(pAuxStage);
         }
      }
   }
   
   return DEVICE_OK;
}


int CPicoHub::GetIntegerFromDevice(const char* command, int axis, int& value, char* errStr)
{
   int ret = DEVICE_OK;

   if (errStr) errStr[0] = '\0';
   const std::lock_guard<std::mutex> lock(mutex_);

   // query commands always start with a "G" (Get)
   const int bufSize = 30;
   char buf[bufSize];
   if (axis<-1) {  // if axis is <-1, we query the global value (-1 queries all axis for some commands)
      snprintf(buf, bufSize, "G%s", command);
   } else {
      // otherwise we query the value for a specific axis 
      snprintf(buf, bufSize, "G%s%i", command, axis);
   }
   ret = SendSerialCommand(port_.c_str(), buf, termChar_);
   if (ret != DEVICE_OK) return ret;

   std::string answer;
   ret = GetSerialAnswer(port_.c_str(), termChar_, answer);
   if (ret != DEVICE_OK) return ret;

   // the response should be in the form "command=1234" (no "G" here)
   std::string prefix = std::string(buf + 1) + "="; // skip the "G" in the buffer
   size_t pos = answer.find(prefix);
   if (pos != std::string::npos) {
      std::string valueStr = answer.substr(pos + prefix.length());
      try {
         value = std::stoi(valueStr);
      }
      catch (...) {
         return ERR_INVALID_RETURN_VAL;
      }
   } else {
      LogMessage("Pico Hub: " + answer, false);
      if (errStr) snprintf(errStr, MM::MaxStrLength, "Pico Hub: %s", answer.c_str());
      return ERR_DYNAMIC_DESCRIPTION;
   }

   return ret;
}


int CPicoHub::SendIntegerToDevice(const char* command, int axis, int value, char* errStr)
{
   int ret = DEVICE_OK;

   if (errStr) errStr[0] = '\0';
   const std::lock_guard<std::mutex> lock(mutex_);

   // set commands always start with a "S" (Set)
   const int bufSize = 30;
   char buf[bufSize];
   if (axis < -1)
   {  // if axis is <-1, we query the global value (-1 queries all axis for some commands)
      snprintf(buf, bufSize, "S%s,%i", command, value);
   }
   else
   {
      // otherwise we query the value for a specific axis 
      snprintf(buf, bufSize, "S%s%i,%i", command, axis, value);
   }
   ret = SendSerialCommand(port_.c_str(), buf, termChar_);
   if (ret != DEVICE_OK) return ret;

   std::string answer;
   ret = GetSerialAnswer(port_.c_str(), termChar_, answer);
   if (ret != DEVICE_OK) return ret;

   // the response should be in the form "ERROR=0"
   std::string prefix = std::string("ERROR=0");
   size_t pos = answer.find(prefix);
   if (pos != std::string::npos) {
      return DEVICE_OK;
   }
   else { // we have an error
      
      snprintf(buf, bufSize, "GPC_EMSG");
      SendSerialCommand(port_.c_str(), buf, termChar_);
      GetSerialAnswer(port_.c_str(), termChar_, answer);
      LogMessage("Pico Hub: " + answer, false);
      if (errStr) snprintf(errStr, MM::MaxStrLength, "Pico Hub: %s", answer.c_str());
      return ERR_DYNAMIC_DESCRIPTION;
   }
}


// private and expects caller to:
// 1. guard the port
// 2. purge the port
int CPicoHub::GetControllerID()
{
   int ret = DEVICE_OK;

   ret = SendSerialCommand(port_.c_str(), "*IDN?", termChar_);
   if (ret != DEVICE_OK) return ret;

   std::string answer;
   ret = GetSerialAnswer(port_.c_str(), termChar_, answer);
   if (ret != DEVICE_OK) return ret;

   std::string prefix = "Stage Driver Pico";
   if (answer.size() < prefix.size() || answer.substr(0, prefix.size()) != prefix) {
      return ERR_BOARD_NOT_FOUND;
   }
   return ret;
}


int CPicoHub::IdentifyAxisChannel(const char* axisLabel, int& channel)
{
   int ret = DEVICE_OK;
   std::vector<std::string> chName = { "Undef", "X", "Y", "Z", "Aux" };
   int axType; // 0->Undef, 1->X, 2->Y, 3->Z, 4->Aux
   int numDevices = 0;

   ret = GetIntegerFromDevice("PC_NDEV", -2, numDevices, NULL); // axis parameter <-1 is "ignore"
   if (ret != DEVICE_OK) return ret;
   if (numDevices < 1) return ERR_NO_DEVICE_DETECTED;
   if (numDevices > 4) return ERR_INVALID_NUMBER_OF_DEVICES; // Pico Hub supports 1-4 devices
   // get the axis type for each device
   for (int idx = 0; idx < numDevices; idx++)
   {
      ret = GetIntegerFromDevice("MP_TAXI", idx, axType, NULL);
      if (ret != DEVICE_OK) return ret;
      if (axType < 0 || axType >= chName.size()) {
         LogMessage("Pico Hub: Unsupported axis type detected", true);
         continue; // skip this axis
      }
      if (chName[axType] == string(axisLabel)) {
         // we found the axisLabel in the list
         channel = idx;
         return DEVICE_OK; // found it, return
      }

   }
   // if we get here, the axisLabel was not found in the list
   LogMessage("Pico Hub: Could not find the requested axis", true);
   return ERR_INVALID_AXIS_LABEL;
}


int CPicoHub::OnPort(MM::PropertyBase* pProp, MM::ActionType pAct)
{
   if (pAct == MM::BeforeGet)
   {
      pProp->Set(port_.c_str());
   }
   else if (pAct == MM::AfterSet)
   {
      pProp->Get(port_);
      portAvailable_ = true;
   }
   return DEVICE_OK;
}



///////////////////////////////////////////////////////////////////////////////
// CPicoXYStage implementation
///////////////////////////////////////////////////////////////////////////////
CPicoXYStage::CPicoXYStage() :
   initialized_(false),
   hub_(nullptr),
   stepSizeXUm_(0.1), // um
   stepSizeYUm_(0.1), // um
   channelX_(-1), // channel on the controller, -1 means not set
   channelY_(-1), // channel on the controller, -1 means not set
   motionInProgress_(false)
{
   InitializeDefaultErrorMessages();

   SetErrorText(ERR_BOARD_NOT_FOUND, "Did not find a Pico board with the correct ID. Is the Pico connected to this serial port?");
   SetErrorText(ERR_PORT_OPEN_FAILED, "Failed opening Pico USB device.");
   SetErrorText(ERR_NO_PORT_SET, "Hub Device not found. The Pico Hub device is needed to create this device.");
   SetErrorText(ERR_NO_DEVICE_DETECTED, "No device was found on the Pico hub.");
   SetErrorText(ERR_INVALID_NUMBER_OF_DEVICES, "Invalid number of channels (allowed: 1..4).");
   SetErrorText(ERR_INVALID_RESPONSE, "Invalid response from the Pico in response to a query.");
   SetErrorText(ERR_INVALID_RETURN_VAL, "Invalid return value from the Pico in response to a query.");
   SetErrorText(ERR_INVALID_AXIS_LABEL, "Invalid label for a stage axis (allowed: X, Y, Z, and Aux).");
   SetErrorText(ERR_DYNAMIC_DESCRIPTION, "TBD dynamically later...");

   // create pre-initialization properties that are required for proper startup

   // Description
   int ret = CreateProperty(MM::g_Keyword_Description, "Pico XY Stage", MM::String, true);
   assert(DEVICE_OK == ret);
   // Name
   ret = CreateProperty(MM::g_Keyword_Name, g_PicoXYStageName, MM::String, true);
   assert(DEVICE_OK == ret);

   // parent ID display
   CreateHubIDProperty();

}


CPicoXYStage::~CPicoXYStage()
{
   Shutdown();
}


void CPicoXYStage::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_PicoXYStageName);
}


int CPicoXYStage::Initialize()
{
   hub_ = static_cast<CPicoHub*>(GetParentHub());
   if (!hub_ || !hub_->IsPortAvailable()) {
      return ERR_NO_PORT_SET;
   }
   char hubLabel[MM::MaxStrLength];
   hub_->GetLabel(hubLabel);
   SetParentID(hubLabel); // for backward comp.

   if (initialized_)
      return DEVICE_OK;


   // figure out which axes are the "X" and "Y" axes
   int ret = hub_->IdentifyAxisChannel("X", channelX_);
   if (ret != DEVICE_OK) return ret;
   ret = hub_->IdentifyAxisChannel("Y", channelY_);
   if (ret != DEVICE_OK) return ret;

   // set property list
   CPropertyAction* pAct = new CPropertyAction(this, &CPicoXYStage::OnStepSizeX);
   ret = CreateFloatProperty("StepSizeX [um]", stepSizeXUm_, false, pAct); // um
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoXYStage::OnStepSizeY);
   ret = CreateFloatProperty("StepSizeY [um]", stepSizeYUm_, false, pAct); // um
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoXYStage::OnVelocityX);
   CreateFloatProperty("VelocityX [mm/s]", 0.0, false, pAct); // mm/s
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoXYStage::OnVelocityY);
   CreateFloatProperty("VelocityY [mm/s]", 0.0, false, pAct); // mm/s
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoXYStage::OnAccelX);
   CreateFloatProperty("AccelerationX [mm/s^2]", 0.0, false, pAct); // mm/s^2
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoXYStage::OnAccelY);
   CreateFloatProperty("AccelerationY [mm/s^2]", 0.0, false, pAct); // mm/s^2
   if (ret != DEVICE_OK) return ret;

   ret = CreateIntegerProperty("SettleTime [ms]", 0, false);
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoXYStage::OnRemote);
   ret = CreateProperty("IsRemoteControlled", "0", MM::Integer, false, pAct); // [0 or 1]
   if (ret != DEVICE_OK) return ret;
   AddAllowedValue("IsRemoteControlled", "0");
   AddAllowedValue("IsRemoteControlled", "1");

   ret = UpdateStatus();
   if (ret != DEVICE_OK) return ret;

   // switch to serial mode, just in case a remote is attached and active
   ret = SendIntegerToDevice("RP_ENAB", channelX_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("RP_ENAB", channelY_, 0);
   if (ret != DEVICE_OK) return ret;
   // enable the motors
   ret = SendIntegerToDevice("MS_ENAB", channelX_, 1);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_ENAB", channelY_, 1);
   if (ret != DEVICE_OK) return ret;

   initialized_ = true;
   return DEVICE_OK;
}


int CPicoXYStage::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}


int CPicoXYStage::GetIntegerFromDevice(const char* command, int channel, int& value)
{  
   char errorString[MM::MaxStrLength];

   if (!hub_ || !hub_->IsPortAvailable()) {
      return ERR_NO_PORT_SET;
   }
   int ret = hub_->GetIntegerFromDevice(command, channel, value, errorString);
   if (ret != DEVICE_OK) {
      if (ERR_DYNAMIC_DESCRIPTION == ret) {
         SetErrorText(ERR_DYNAMIC_DESCRIPTION, errorString);
      }
      return ret;
   }
   return DEVICE_OK;
}


int CPicoXYStage::SendIntegerToDevice(const char* command, int channel, int value)
{
   char errorString[MM::MaxStrLength];

   if (!hub_ || !hub_->IsPortAvailable()) {
      return ERR_NO_PORT_SET;
   }
   int ret = hub_->SendIntegerToDevice(command, channel, value, errorString);
   if (ret != DEVICE_OK) {
      if (ERR_DYNAMIC_DESCRIPTION == ret) {
         SetErrorText(ERR_DYNAMIC_DESCRIPTION, errorString);
      }
      return ret;
   }
   return DEVICE_OK;
}


/**
 * Returns true if any axis (X or Y) is still moving.
 */
bool CPicoXYStage::Busy()
{
   int isDoneX, isDoneY;
   long delayTime;

   int ret = GetIntegerFromDevice("MC_POSR", channelX_, isDoneX);
   if (ret != DEVICE_OK) return false;
   ret = GetIntegerFromDevice("MC_POSR", channelY_, isDoneY);
   if (ret != DEVICE_OK) return false;

   if (isDoneX == 1 && isDoneY == 1) {
      // both axes are done
      if (motionInProgress_) { // delay only after motion
         GetProperty("SettleTime [ms]", delayTime);
         CDeviceUtils::SleepMs(delayTime);
      }
      motionInProgress_ = false;
      return false;
   }
   else return true; // at least one axis is still moving
 }

 
/**
 * Sets position in steps.
 */
int CPicoXYStage::SetPositionSteps(long x, long y)
{
   int ret = SendIntegerToDevice("MC_MPOS", channelX_, (int)x);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MC_MPOS", channelY_, (int)y);
   if (ret != DEVICE_OK) return ret;
   motionInProgress_ = true;

   return DEVICE_OK;
}

/**
 * Gets position in steps.
 */
int CPicoXYStage::GetPositionSteps(long& x, long& y)
{
   int xInt, yInt; // even though int and long are the same here, compiler enforces the types

   int ret = GetIntegerFromDevice("MS_XACT", channelX_, xInt);
   if (ret != DEVICE_OK) return ret;
   ret = GetIntegerFromDevice("MS_XACT", channelY_, yInt);
   if (ret != DEVICE_OK) return ret;

   x = (long)xInt; // convert to long
   y = (long)yInt; // convert to long
   return DEVICE_OK;
}


// sets the current position to 0,0
int CPicoXYStage::SetOrigin()
{
   int ret;

   // sequence: disable motors, set the current, target, encoder positions to 0, enable motors again
   // xChannel
   ret = SendIntegerToDevice("MS_ENAB", channelX_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XACT", channelX_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XTAR", channelX_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XENC", channelX_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_ENAB", channelX_, 1);
   if (ret != DEVICE_OK) return ret;

   // yChannel
   ret = SendIntegerToDevice("MS_ENAB", channelY_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XACT", channelY_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XTAR", channelY_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XENC", channelY_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_ENAB", channelY_, 1);
   if (ret != DEVICE_OK) return ret;

   ret = SetAdapterOriginUm(0.0, 0.0); // set the adapter origin to 0,0
   if (ret != DEVICE_OK) return ret;

   return DEVICE_OK;
}


// units are um/s
int CPicoXYStage::Move(double velX, double velY)
{
   int vel = nint(velX / stepSizeXUm_); // convert um/s to steps/s
   int ret = SendIntegerToDevice("MC_MVEL", channelX_, vel);
   if (ret != DEVICE_OK) return ret;
   vel = nint(velY / stepSizeYUm_); // convert um/s to steps/s
   ret = SendIntegerToDevice("MC_MVEL", channelY_, vel);
   if (ret != DEVICE_OK) return ret;

   return DEVICE_OK;
}



int CPicoXYStage::Stop()
{
   return Move(0.0, 0.0);
}


double CPicoXYStage::GetStepSizeXUm()
{
   return stepSizeXUm_;
}

double CPicoXYStage::GetStepSizeYUm()
{
   return stepSizeYUm_;
}


int CPicoXYStage::Home()
{
   return DEVICE_OK;
//   return DEVICE_UNSUPPORTED_COMMAND;
}


int CPicoXYStage::GetLimitsUm(double& xMin, double& xMax, double& yMin, double& yMax)  
{  
    (void)xMin; (void)xMax; (void)yMin; (void)yMax; // Suppress unused parameter warnings  
    return DEVICE_UNSUPPORTED_COMMAND;
}


int CPicoXYStage::GetStepLimits(long& xMin, long& xMax, long& yMin, long& yMax)
{
   (void)xMin; (void)xMax; (void)yMin; (void)yMax; // Suppress unused parameter warnings
   return DEVICE_UNSUPPORTED_COMMAND;
}

//////////////////////////////////////
// action interface
//////////////////////////////////////
int CPicoXYStage::OnStepSizeX(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet) 
   {
      pProp->Set(stepSizeXUm_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(stepSizeXUm_);
   }
   return DEVICE_OK;
}


int CPicoXYStage::OnStepSizeY(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(stepSizeYUm_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(stepSizeYUm_);
   }
   return DEVICE_OK;
}


int CPicoXYStage::OnVelocityX(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int intVal;
   double doubleVal;

   // velocity here is mm/s, in the device it's steps/s
   if (eAct == MM::BeforeGet)
   {
      int ret = GetIntegerFromDevice("MP_RSEV", channelX_, intVal);
      if (ret != DEVICE_OK) return ret;
      pProp->Set(intVal * stepSizeXUm_ * 1.0E-3);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(doubleVal);
      int ret = SendIntegerToDevice("MP_RSEV", channelX_, (int)nint(1000.*doubleVal / stepSizeXUm_));
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}

int CPicoXYStage::OnVelocityY(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int intVal;
   double doubleVal;

   // velocity here is mm/s, in the device it's steps/s
   if (eAct == MM::BeforeGet)
   {
      int ret = GetIntegerFromDevice("MP_RSEV", channelY_, intVal);
      if (ret != DEVICE_OK) return ret;
      pProp->Set(intVal * stepSizeYUm_ * 1.0E-3);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(doubleVal);
      int ret = SendIntegerToDevice("MP_RSEV", channelY_, (int)nint(1000. * doubleVal / stepSizeYUm_));
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}



int CPicoXYStage::OnAccelX(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int intVal;
   double doubleVal;

   // acc here is mm/s^2, in the device it's steps/s^2
   if (eAct == MM::BeforeGet)
   {
      int ret = GetIntegerFromDevice("MP_RSEA", channelX_, intVal);
      if (ret != DEVICE_OK) return ret;
      pProp->Set(intVal * stepSizeXUm_ * 1.0E-3);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(doubleVal);
      int ret = SendIntegerToDevice("MP_RSEA", channelX_, (int)nint(1.0E3 * doubleVal / stepSizeXUm_));
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}


int CPicoXYStage::OnAccelY(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int intVal;
   double doubleVal;

   // acc here is mm/s^2, in the device it's steps/s^2
   if (eAct == MM::BeforeGet)
   {
      int ret = GetIntegerFromDevice("MP_RSEA", channelY_, intVal);
      if (ret != DEVICE_OK) return ret;
      pProp->Set(intVal * stepSizeYUm_ * 1.0E-3);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(doubleVal);
      int ret = SendIntegerToDevice("MP_RSEA", channelY_, (int)nint(1.0E3 * doubleVal / stepSizeYUm_));
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}


int CPicoXYStage::OnRemote(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int ret;
   int isRemoteEnabledX, isRemoteEnabledY;
   long longVal = 0;
   if (eAct == MM::BeforeGet)
   {
      ret = GetIntegerFromDevice("RP_ENAB", channelX_, isRemoteEnabledX);
      if (ret != DEVICE_OK) return ret;
      ret = GetIntegerFromDevice("RP_ENAB", channelY_, isRemoteEnabledY);
      if (ret != DEVICE_OK) return ret;
      pProp->Set((long) (isRemoteEnabledX | isRemoteEnabledY));
      if (ret != DEVICE_OK) return ret;
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(longVal);
      ret = SendIntegerToDevice("RP_ENAB", channelX_, (int)longVal);
      if (ret != DEVICE_OK) return ret;
      ret = SendIntegerToDevice("RP_ENAB", channelY_, (int)longVal);
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////
// CPicoStage implementation
///////////////////////////////////////////////////////////////////////////////
CPicoStage::CPicoStage(const char* deviceName) :
   initialized_(false),
   hub_(nullptr),
   stepSizeUm_(0.1),
   originSteps_(0),
   channel_(-1), // -1 means not set
   id_("-"), // axis ID, e.g. "Z" or "Aux"
   motionInProgress_(false)
{
   InitializeDefaultErrorMessages();

   SetErrorText(ERR_BOARD_NOT_FOUND, "Did not find a Pico board with the correct ID. Is the Pico connected to this serial port?");
   SetErrorText(ERR_PORT_OPEN_FAILED, "Failed opening Pico USB device.");
   SetErrorText(ERR_NO_PORT_SET, "Hub Device not found. The Pico Hub device is needed to create this device.");
   SetErrorText(ERR_NO_DEVICE_DETECTED, "No device was found on the Pico hub.");
   SetErrorText(ERR_INVALID_NUMBER_OF_DEVICES, "Invalid number of channels (allowed: 1..4).");
   SetErrorText(ERR_INVALID_RESPONSE, "Invalid response from the Pico in response to a query.");
   SetErrorText(ERR_INVALID_RETURN_VAL, "Invalid return value from the Pico in response to a query.");
   SetErrorText(ERR_INVALID_AXIS_LABEL, "Invalid label for a stage axis (allowed: X, Y, Z, and Aux).");
   SetErrorText(ERR_DYNAMIC_DESCRIPTION, "TBD dynamically later...");

   // create pre-initialization properties that are required for proper startup

   // Name
   int ret = CreateProperty(MM::g_Keyword_Name, deviceName, MM::String, true, 0, true); // read-only, no action, pre-init
   assert(DEVICE_OK == ret);

   // Description
   char description[MM::MaxStrLength];
   GetDeviceDescription(deviceName, description, MM::MaxStrLength);
   ret = CreateProperty(MM::g_Keyword_Description, description, MM::String, true);
   assert(DEVICE_OK == ret);

   // Axis ID
   if (strncmp(deviceName, g_PicoZStageName, MM::MaxStrLength) == 0) id_ = "Z";
   if (strncmp(deviceName, g_PicoAuxStageName, MM::MaxStrLength) == 0) id_ = "Aux";

   // parent ID display
   CreateHubIDProperty();

}


CPicoStage::~CPicoStage()
{
   Shutdown();
}


void CPicoStage::GetName(char* name) const
{
   char devName[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_Name, devName);
   if (DEVICE_OK != ret) {
      LogMessage("Unable to obtain device name.", false);
      CDeviceUtils::CopyLimitedString(name, "---");
   } else {
      CDeviceUtils::CopyLimitedString(name, devName);
   }
}


int CPicoStage::Initialize()
{
   hub_ = static_cast<CPicoHub*>(GetParentHub());
   if (!hub_ || !hub_->IsPortAvailable()) {
      return ERR_NO_PORT_SET;
   }
   char hubLabel[MM::MaxStrLength];
   hub_->GetLabel(hubLabel);
   SetParentID(hubLabel); // for backward comp.

   if (initialized_)
      return DEVICE_OK;

   // figure out which channel is the id_ axes
   if ("-" == id_) return ERR_INVALID_AXIS_LABEL; // no axis selected
   int ret = hub_->IdentifyAxisChannel(id_.c_str(), channel_);
   if (ret != DEVICE_OK) return ret;

   // set property list
   CPropertyAction* pAct = new CPropertyAction(this, &CPicoStage::OnStepSize);
   ret = CreateFloatProperty("StepSize [um]", stepSizeUm_, false, pAct); // um
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoStage::OnVelocity);
   CreateFloatProperty("Velocity [mm/s]", 0.0, false, pAct); // mm/s
   if (ret != DEVICE_OK) return ret;
   // SetPropertyLimits("Velocity [mm/s]", 0.01, 2.0); // limit checks in the device

   pAct = new CPropertyAction(this, &CPicoStage::OnAccel);
   CreateFloatProperty("Acceleration [mm/s^2]", 0.0, false, pAct); // mm/s^2
   if (ret != DEVICE_OK) return ret;
   // SetPropertyLimits("AccelerationX [mm/s^2]", 0.01, 2.0); // limit checks in the device

   ret = CreateIntegerProperty("SettleTime [ms]", 0, false);
   if (ret != DEVICE_OK) return ret;

   pAct = new CPropertyAction(this, &CPicoStage::OnRemote);
   ret = CreateProperty("IsRemoteControlled", "0", MM::Integer, false, pAct); // [0 or 1]
   if (ret != DEVICE_OK) return ret;
   AddAllowedValue("IsRemoteControlled", "0");
   AddAllowedValue("IsRemoteControlled", "1");

   ret = UpdateStatus();
   if (ret != DEVICE_OK) return ret;

   // switch to serial mode, just in case a remote is attached and active
   ret = SendIntegerToDevice("RP_ENAB", channel_, 0);
   if (ret != DEVICE_OK) return ret;
   // enable the motors
   ret = SendIntegerToDevice("MS_ENAB", channel_, 1);
   if (ret != DEVICE_OK) return ret;

   initialized_ = true;
   return DEVICE_OK;
}


int CPicoStage::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}


int CPicoStage::GetIntegerFromDevice(const char* command, int channel, int& value)
{
   char errorString[MM::MaxStrLength];

   if (!hub_ || !hub_->IsPortAvailable()) {
      return ERR_NO_PORT_SET;
   }
   int ret = hub_->GetIntegerFromDevice(command, channel, value, errorString);
   if (ret != DEVICE_OK) {
      if (ERR_DYNAMIC_DESCRIPTION == ret) {
         SetErrorText(ERR_DYNAMIC_DESCRIPTION, errorString);
      }
      return ret;
   }
   return DEVICE_OK;
}


int CPicoStage::SendIntegerToDevice(const char* command, int channel, int value)
{
   char errorString[MM::MaxStrLength];

   if (!hub_ || !hub_->IsPortAvailable()) {
      return ERR_NO_PORT_SET;
   }
   int ret = hub_->SendIntegerToDevice(command, channel, value, errorString);
   if (ret != DEVICE_OK) {
      if (ERR_DYNAMIC_DESCRIPTION == ret) {
         SetErrorText(ERR_DYNAMIC_DESCRIPTION, errorString);
      }
      return ret;
   }
   return DEVICE_OK;
}


/**
 * Returns true if the axis is still moving.
 */
bool CPicoStage::Busy()
{
   int isDone;
   long delayTime;

   int ret = GetIntegerFromDevice("MC_POSR", channel_, isDone);
   if (ret != DEVICE_OK) return false;

   if (isDone == 1) {
      if (motionInProgress_) { // delay only after motion
         GetProperty("SettleTime [ms]", delayTime);
         CDeviceUtils::SleepMs(delayTime);
      }
      motionInProgress_ = false;
      return false;
   }
   else return true; // axis is still moving
}


int CPicoStage::SetPositionUm(double pos)
{
   long steps = nint(pos / stepSizeUm_);
   return SetPositionSteps(steps);
}

int CPicoStage::SetPositionSteps(long steps)
{
   steps += originSteps_; // add the origin offset in steps
   int ret = SendIntegerToDevice("MC_MPOS", channel_, (int)steps);
   if (ret != DEVICE_OK) return ret;
   return DEVICE_OK;
}


int CPicoStage::GetPositionUm(double& pos)
{
   long steps;
   int ret = GetPositionSteps(steps);
   if (ret != DEVICE_OK) return ret;
   pos = steps * stepSizeUm_;
   return DEVICE_OK;
}


int CPicoStage::GetPositionSteps(long& steps)
{
   int posInt; // even though int and long are the same here, compiler enforces the types
   int ret = GetIntegerFromDevice("MS_XACT", channel_, posInt);
   if (ret != DEVICE_OK) return ret;
   steps = (long)posInt - originSteps_; // convert to long
   return DEVICE_OK;
}



double CPicoStage::GetStepSizeUm()
{
   return stepSizeUm_;
}


int CPicoStage::SetOrigin()
{
   int ret;

   // sequence: disable motors, set the current, target, encoder positions to 0, enable motors again
   ret = SendIntegerToDevice("MS_ENAB", channel_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XACT", channel_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XTAR", channel_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_XENC", channel_, 0);
   if (ret != DEVICE_OK) return ret;
   ret = SendIntegerToDevice("MS_ENAB", channel_, 1);
   if (ret != DEVICE_OK) return ret;

   originSteps_ = 0; // set the adapter origin to 0
   if (ret != DEVICE_OK) return ret;

   return DEVICE_OK;

}



int CPicoStage::Move(double vel)
{
   int velocity = nint(vel / stepSizeUm_); // convert um/s to steps/s
   int ret = SendIntegerToDevice("MC_MVEL", channel_, velocity);
   if (ret != DEVICE_OK) return ret;
   return DEVICE_OK;
}


int CPicoStage::Home()
{
   return DEVICE_OK;
//   return DEVICE_UNSUPPORTED_COMMAND;
}


int CPicoStage::Stop()
{
   return Move(0.0);
}


int CPicoStage::GetLimits(double& min, double& max)
{
   (void)min; (void)max; // Suppress unused parameter warnings
   return DEVICE_UNSUPPORTED_COMMAND;
}


int CPicoStage::OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(stepSizeUm_);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(stepSizeUm_);
   }
   return DEVICE_OK;
}


int CPicoStage::OnVelocity(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int intVal;
   double doubleVal;

   // velocity here is mm/s, in the device it's steps/s
   if (eAct == MM::BeforeGet)
   {
      int ret = GetIntegerFromDevice("MP_RSEV", channel_, intVal);
      if (ret != DEVICE_OK) return ret;
      pProp->Set(intVal * stepSizeUm_ * 1.0E-3);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(doubleVal);
      int ret = SendIntegerToDevice("MP_RSEV", channel_, (int)nint(1000. * doubleVal / stepSizeUm_));
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}


int CPicoStage::OnAccel(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int intVal;
   double doubleVal;

   // acc here is mm/s^2, in the device it's steps/s^2
   if (eAct == MM::BeforeGet)
   {
      int ret = GetIntegerFromDevice("MP_RSEA", channel_, intVal);
      if (ret != DEVICE_OK) return ret;
      pProp->Set(intVal * stepSizeUm_ * 1.0E-3);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(doubleVal);
      int ret = SendIntegerToDevice("MP_RSEA", channel_, (int)nint(1.0E3 * doubleVal / stepSizeUm_));
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}

int CPicoStage::OnRemote(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int intVal;
   long longVal;
   if (eAct == MM::BeforeGet)
   {
      int ret = GetIntegerFromDevice("RP_ENAB", channel_, intVal);
      if (ret != DEVICE_OK) return ret;
      pProp->Set((long)intVal);
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(longVal);
      int ret = SendIntegerToDevice("RP_ENAB", channel_, (int)longVal);
      if (ret != DEVICE_OK) return ret;
   }
   return DEVICE_OK;
}
