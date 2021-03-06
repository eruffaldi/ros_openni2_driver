#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>

#include <image_geometry/pinhole_camera_model.h>

#include <dynamic_reconfigure/server.h>
#include "PS1080.h"

#define LogError(e)

namespace RosDriver
{
  class Device;

  class VideoStream : public oni::driver::StreamBase
  {
  private:
    unsigned int frame_id; // number each frame

    virtual OniStatus setVideoMode(OniVideoMode requested_mode) = 0;
    virtual void populateFrame(void* data, OniFrame* frame) const = 0;


  protected:
    static const OniSensorType sensor_type;
    Device* device;
    bool running; // buildFrame() does something iff true
    OniVideoMode video_mode;

  public:
    VideoStream(Device* device) :
      frame_id(1),
      device(device)
      {
        // joy of structs
        memset(&video_mode, 0, sizeof(video_mode));
      }
    //~VideoStream() { stop();  }

    void buildFrame(void* data, uint32_t timestamp)
    {
      if (!running)
        return;

      OniFrame* frame = getServices().acquireFrame();
      frame->frameIndex = frame_id++;
      frame->timestamp = timestamp;
      frame->videoMode = video_mode;
      frame->width = video_mode.resolutionX;
      frame->height = video_mode.resolutionY;

      populateFrame(data, frame);
      raiseNewFrame(frame);
      getServices().releaseFrame(frame);
    }

    // from StreamBase

    OniStatus start()
    {
      running = true;
      return ONI_STATUS_OK;
    }
    void stop() { running = false; }

    // only add to property handlers if the property is generic to all children
    // otherwise, implement in child and call these in default case
    OniBool isPropertySupported(int propertyId)
    {
      switch(propertyId)
      {
        case ONI_STREAM_PROPERTY_VIDEO_MODE:
        case ONI_STREAM_PROPERTY_CROPPING:
        case ONI_STREAM_PROPERTY_MIRRORING:
          return true;
        default:
          return false;
      }
    }

    virtual OniStatus getProperty(int propertyId, void* data, int* pDataSize)
    {
      switch (propertyId)
      {
        default:
        case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:      // float: radians
        case ONI_STREAM_PROPERTY_VERTICAL_FOV:        // float: radians
        case ONI_STREAM_PROPERTY_MAX_VALUE:           // int
        case ONI_STREAM_PROPERTY_MIN_VALUE:           // int
        case ONI_STREAM_PROPERTY_STRIDE:              // int
        case ONI_STREAM_PROPERTY_NUMBER_OF_FRAMES:    // int
        // camera
        case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:  // OniBool
        case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:       // OniBool
        // xn
        case XN_STREAM_PROPERTY_INPUT_FORMAT:         // unsigned long long
        case XN_STREAM_PROPERTY_CROPPING_MODE:        // XnCroppingMode
          return ONI_STATUS_NOT_SUPPORTED;

        case ONI_STREAM_PROPERTY_VIDEO_MODE:          // OniVideoMode*
          if (*pDataSize != sizeof(OniVideoMode))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_VIDEO_MODE");
            return ONI_STATUS_ERROR;
          }
          *(static_cast<OniVideoMode*>(data)) = video_mode;
          return ONI_STATUS_OK;

        case ONI_STREAM_PROPERTY_CROPPING:            // OniCropping*
          if (*pDataSize != sizeof(OniCropping))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_CROPPING");
            return ONI_STATUS_ERROR;
          }
         // *(static_cast<OniCropping*>(data)) = cropping;
          return ONI_STATUS_OK;

        case ONI_STREAM_PROPERTY_MIRRORING:           // OniBool
          if (*pDataSize != sizeof(OniBool))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_MIRRORING");
            return ONI_STATUS_ERROR;
          }
         // *(static_cast<OniBool*>(data)) = mirroring;
          return ONI_STATUS_OK;
      }
    }
    virtual OniStatus setProperty(int propertyId, const void* data, int dataSize)
    {
      switch (propertyId)
      {
        default:
        case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:      // float: radians
        case ONI_STREAM_PROPERTY_VERTICAL_FOV:        // float: radians
        case ONI_STREAM_PROPERTY_MAX_VALUE:           // int
        case ONI_STREAM_PROPERTY_MIN_VALUE:           // int
        case ONI_STREAM_PROPERTY_STRIDE:              // int
        case ONI_STREAM_PROPERTY_NUMBER_OF_FRAMES:    // int
        // camera
        case ONI_STREAM_PROPERTY_AUTO_WHITE_BALANCE:  // OniBool
        case ONI_STREAM_PROPERTY_AUTO_EXPOSURE:       // OniBool
        // xn
        case XN_STREAM_PROPERTY_INPUT_FORMAT:         // unsigned long long
        case XN_STREAM_PROPERTY_CROPPING_MODE:        // XnCroppingMode
          return ONI_STATUS_NOT_SUPPORTED;

        case ONI_STREAM_PROPERTY_VIDEO_MODE:          // OniVideoMode*
        #if 0
          if (dataSize != sizeof(OniVideoMode))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_VIDEO_MODE");
            return ONI_STATUS_ERROR;
          }
          if (ONI_STATUS_OK != setVideoMode(*(static_cast<const OniVideoMode*>(data))))
          raisePropertyChanged(propertyId, data, dataSize);
          return ONI_STATUS_OK;
          #endif
            return ONI_STATUS_NOT_SUPPORTED;

        case ONI_STREAM_PROPERTY_CROPPING:            // OniCropping*
        #if 0
          if (dataSize != sizeof(OniCropping))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_CROPPING");
            return ONI_STATUS_ERROR;
          }
          cropping = *(static_cast<const OniCropping*>(data));
          raisePropertyChanged(propertyId, data, dataSize);
          return ONI_STATUS_OK;
          #endif
            return ONI_STATUS_NOT_SUPPORTED;

        case ONI_STREAM_PROPERTY_MIRRORING:           // OniBool
        #if 0
          if (dataSize != sizeof(OniBool))
          {
            LogError("Unexpected size for ONI_STREAM_PROPERTY_MIRRORING");
            return ONI_STATUS_ERROR;
          }
          mirroring = *(static_cast<const OniBool*>(data));
          raisePropertyChanged(propertyId, data, dataSize);
          return ONI_STATUS_OK;
        #endif
            return ONI_STATUS_NOT_SUPPORTED;

      }
    }


    /* todo : from StreamBase
    virtual OniStatus convertDepthToColorCoordinates(StreamBase* colorStream, int depthX, int depthY, OniDepthPixel depthZ, int* pColorX, int* pColorY) { return ONI_STATUS_NOT_SUPPORTED; }
    */
  };
}


/* image video modes reference

FREENECT_VIDEO_RGB             = 0, //< Decompressed RGB mode (demosaicing done by libfreenect)
FREENECT_VIDEO_BAYER           = 1, //< Bayer compressed mode (raw information from camera)
FREENECT_VIDEO_IR_8BIT         = 2, //< 8-bit IR mode
FREENECT_VIDEO_IR_10BIT        = 3, //< 10-bit IR mode
FREENECT_VIDEO_IR_10BIT_PACKED = 4, //< 10-bit packed IR mode
FREENECT_VIDEO_YUV_RGB         = 5, //< YUV RGB mode
FREENECT_VIDEO_YUV_RAW         = 6, //< YUV Raw mode
FREENECT_VIDEO_DUMMY           = 2147483647, //< Dummy value to force enum to be 32 bits wide

ONI_PIXEL_FORMAT_RGB888 = 200,
ONI_PIXEL_FORMAT_YUV422 = 201,
ONI_PIXEL_FORMAT_GRAY8 = 202,
ONI_PIXEL_FORMAT_GRAY16 = 203,
ONI_PIXEL_FORMAT_JPEG = 204,
*/
