#pragma once

#include <algorithm> // for transform()
#include <cmath> // for M_PI
#include "Driver/OniDriverAPI.h"
#include "Utility.hpp"
#include "VideoStream.hpp"


namespace RosDriver
{
  class ColorStream : public VideoStream
  {
  public:
    // from NUI library & converted to radians
    static const float DIAGONAL_FOV = 73.9 * (M_PI / 180);
    static const float HORIZONTAL_FOV = 62 * (M_PI / 180);
    static const float VERTICAL_FOV = 48.6 * (M_PI / 180);

  private:
    static const OniSensorType sensor_type = ONI_SENSOR_COLOR;

    OniStatus setVideoMode(OniVideoMode requested_mode);
    void populateFrame(void* data, OniFrame* frame) const;
    
    bool auto_white_balance;
    bool auto_exposure;

  public:
    ColorStream(Device* pDevice);
    //~ColorStream() { }

    static OniSensorInfo getSensorInfo()
    {
      /*
      FreenectVideoModeMap supported_modes = getSupportedVideoModes();
      OniVideoMode* modes = new OniVideoMode[supported_modes.size()];
      std::transform(supported_modes.begin(), supported_modes.end(), modes, ExtractKey());
      */
      OniVideoMode* modes = 0;
      OniSensorInfo sensors = { sensor_type, 0,0}; //static_cast<int>(supported_modes.size()), modes };
      return sensors;
    }

    // from StreamBase
    OniBool isPropertySupported(int propertyId)
    {
      switch(propertyId)
      {
        default:
          return VideoStream::isPropertySupported(propertyId);
          
        case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
        case ONI_STREAM_PROPERTY_VERTICAL_FOV:
        case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:
        case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:
          return true;
      }
    }

    OniStatus getProperty(int propertyId, void* data, int* pDataSize)
    {
      switch (propertyId)
      {
        default:
          return VideoStream::getProperty(propertyId, data, pDataSize);

        case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:     // float (radians)
        {
          if (*pDataSize != sizeof(float))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_HORIZONTAL_FOV");
            return ONI_STATUS_ERROR;
          }
          *(static_cast<float*>(data)) = HORIZONTAL_FOV;
          return ONI_STATUS_OK;
        }
        case ONI_STREAM_PROPERTY_VERTICAL_FOV:       // float (radians)
        {
          if (*pDataSize != sizeof(float))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_VERTICAL_FOV");
            return ONI_STATUS_ERROR;
          }
          *(static_cast<float*>(data)) = VERTICAL_FOV;
          return ONI_STATUS_OK;
        }
        
        // camera
        case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE: // OniBool
        {
          if (*pDataSize != sizeof(OniBool))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE");
            return ONI_STATUS_ERROR;
          }
          *(static_cast<OniBool*>(data)) = auto_white_balance;
          return ONI_STATUS_OK;
        }
        case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:      // OniBool
        {
          if (*pDataSize != sizeof(OniBool))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_AUTO_EXPOSURE");
            return ONI_STATUS_ERROR;
          }
          *(static_cast<OniBool*>(data)) = auto_exposure;
          return ONI_STATUS_OK;
        }
      }
    }
    
    OniStatus setProperty(int propertyId, const void* data, int dataSize)
    {
      switch (propertyId)
      {
        default:
          return VideoStream::setProperty(propertyId, data, dataSize);
      
        // camera
        case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE: // OniBool
        {
          if (dataSize != sizeof(OniBool))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE");
            return ONI_STATUS_ERROR;
          }
          auto_white_balance = *(static_cast<const OniBool*>(data));
          int ret = 0;//device->setFlag(FREENECT_AUTO_WHITE_BALANCE, auto_white_balance);
          return (ret == 0) ? ONI_STATUS_OK : ONI_STATUS_ERROR;
        }
        case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:      // OniBool
        {
          if (dataSize != sizeof(OniBool))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_AUTO_EXPOSURE");
            return ONI_STATUS_ERROR;
          }
          auto_exposure = *(static_cast<const OniBool*>(data));
          int ret = 0;//device->setFlag(FREENECT_AUTO_WHITE_BALANCE, auto_exposure);
          return (ret == 0) ? ONI_STATUS_OK : ONI_STATUS_ERROR;
        }
        case ONI_STREAM_PROPERTY_MIRRORING:          // OniBool
        {
          if (dataSize != sizeof(OniBool))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_MIRRORING");
            return ONI_STATUS_ERROR;
          }
          //mirroring = *(static_cast<const OniBool*>(data));
          int ret = 0;//device->setFlag(FREENECT_MIRROR_VIDEO, mirroring);
          return (ret == 0) ? ONI_STATUS_OK : ONI_STATUS_ERROR;
        }
      }
    }
  };
}
