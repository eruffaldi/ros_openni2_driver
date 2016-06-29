/**
*  RosDricer
*  Copyright 2016 Emanuele Ruffaldi <e.ruffaldi@sssup.it>
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*/
#include <map>
#include <string>
#include "Driver/OniDriverAPI.h"
#include "DepthStream.hpp"
#include "ColorStream.hpp"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>


namespace RosDriver
{
  class Device : public oni::driver::DeviceBase 
  {
  private:
    ColorStream* color;
    DepthStream* depth;

    ros::NodeHandlePtr rgb_nh_;
    boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;

    // Subscriptions
    image_transport::SubscriberFilter sub_depth_, sub_rgb_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ExactSynchronizer> exact_sync_;

    // for Freenect::FreenectDevice
    void DepthCallback(void* data, uint32_t timestamp) {
      depth->buildFrame(data, timestamp);
    }
    void VideoCallback(void* data, uint32_t timestamp) {
      color->buildFrame(data, timestamp);
    }

  public:
    Device(void* fn_ctx, int index) : /* Freenect::FreenectDevice(fn_ctx, index),*/
      color(NULL),
      depth(NULL) {
ros::NodeHandle xx;
    ros::NodeHandle& nh         = xx;//getNodeHandle();
    ros::NodeHandle& private_nh = xx;//getPrivateNodeHandle();
    rgb_nh_.reset( new ros::NodeHandle(nh, "rgb") );
    ros::NodeHandle depth_nh(nh, "depth_registered");
    rgb_it_  .reset( new image_transport::ImageTransport(*rgb_nh_) );
    depth_it_.reset( new image_transport::ImageTransport(depth_nh) );

    // Read parameters
    int queue_size;
    private_nh.param("queue_size", queue_size, 5);
    bool use_exact_sync;
    private_nh.param("exact_sync", use_exact_sync, false);

    // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
    if (use_exact_sync)
    {
      exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
      exact_sync_->registerCallback(boost::bind(&Device::imageCb, this, _1, _2, _3));
    }
    else
    {
      sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
      sync_->registerCallback(boost::bind(&Device::imageCb, this, _1, _2, _3));
    }

    // parameter for depth_image_transport hint
    std::string depth_image_transport_param = "depth_image_transport";

    // depth image can use different transport.(e.g. compressedDepth)
    image_transport::TransportHints depth_hints("raw",ros::TransportHints(), private_nh, depth_image_transport_param);
    sub_depth_.subscribe(*depth_it_, "image_rect",       1, depth_hints);

    // rgb uses normal ros transport hints.
    image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
    sub_rgb_  .subscribe(*rgb_it_,   "image_rect_color", 1, hints);
    sub_info_ .subscribe(*rgb_nh_,   "camera_info",      1);



       }
    ~Device()
    {
      destroyStream(color);
      destroyStream(depth);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg_in,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg)
    {

    }

    // for DeviceBase

    OniBool isImageRegistrationModeSupported(OniImageRegistrationMode mode) { return depth->isImageRegistrationModeSupported(mode); }

    OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
    {
      *numSensors = 2;
      OniSensorInfo * sensors = new OniSensorInfo[*numSensors];
      sensors[0] = depth->getSensorInfo();
      sensors[1] = color->getSensorInfo();
      *pSensors = sensors;
      return ONI_STATUS_OK;
    }

    oni::driver::StreamBase* createStream(OniSensorType sensorType)
    {
      switch (sensorType)
      {
        default:
          LogError("Cannot create a stream of type " + to_string(sensorType));
          return NULL;
        case ONI_SENSOR_COLOR:
          if (! color)
            color = new ColorStream(this);
          return color;
        case ONI_SENSOR_DEPTH:
          if (! depth)
            depth = new DepthStream(this);
          return depth;
        // todo: IR
      }
    }

    void destroyStream(oni::driver::StreamBase* pStream)
    {
      if (pStream == NULL)
        return;

      if (pStream == color)
      {
        //Freenect::FreenectDevice::stopVideo();
        delete color;
        color = NULL;
      }
      if (pStream == depth)
      {
        //Freenect::FreenectDevice::stopDepth();
        delete depth;
        depth = NULL;
      }
    }

    // todo: fill out properties
    OniBool isPropertySupported(int propertyId)
    {
      if (propertyId == ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION)
        return true;
      return false;
    }

    OniStatus getProperty(int propertyId, void* data, int* pDataSize)
    {
      switch (propertyId)
      {
        default:
        case ONI_DEVICE_PROPERTY_FIRMWARE_VERSION:        // string
        case ONI_DEVICE_PROPERTY_DRIVER_VERSION:          // OniVersion
        case ONI_DEVICE_PROPERTY_HARDWARE_VERSION:        // int
        case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:           // string
        case ONI_DEVICE_PROPERTY_ERROR_STATE:             // ?
        // files
        case ONI_DEVICE_PROPERTY_PLAYBACK_SPEED:          // float
        case ONI_DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED: // OniBool
        // xn
        case XN_MODULE_PROPERTY_USB_INTERFACE:            // XnSensorUsbInterface
        case XN_MODULE_PROPERTY_MIRROR:                   // bool
        case XN_MODULE_PROPERTY_RESET_SENSOR_ON_STARTUP:  // unsigned long long
        case XN_MODULE_PROPERTY_LEAN_INIT:                // unsigned long long
        case XN_MODULE_PROPERTY_SERIAL_NUMBER:            // unsigned long long
        case XN_MODULE_PROPERTY_VERSION:                  // XnVersions
          return ONI_STATUS_NOT_SUPPORTED;

        case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:      // OniImageRegistrationMode
          if (*pDataSize != sizeof(OniImageRegistrationMode))
          {
            LogError("Unexpected size for ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION");
            return ONI_STATUS_ERROR;
          }
          *(static_cast<OniImageRegistrationMode*>(data)) = depth->getImageRegistrationMode();
          return ONI_STATUS_OK;
      }
    }
    
    OniStatus setProperty(int propertyId, const void* data, int dataSize)
    {
      switch (propertyId)
      {
        default:
        case ONI_DEVICE_PROPERTY_FIRMWARE_VERSION:        // By implementation
        case ONI_DEVICE_PROPERTY_DRIVER_VERSION:          // OniVersion
        case ONI_DEVICE_PROPERTY_HARDWARE_VERSION:        // int
        case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:           // string
        case ONI_DEVICE_PROPERTY_ERROR_STATE:             // ?
        // files
        case ONI_DEVICE_PROPERTY_PLAYBACK_SPEED:          // float
        case ONI_DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED: // OniBool
        // xn
        case XN_MODULE_PROPERTY_USB_INTERFACE:            // XnSensorUsbInterface
        case XN_MODULE_PROPERTY_MIRROR:                   // bool
        case XN_MODULE_PROPERTY_RESET_SENSOR_ON_STARTUP:  // unsigned long long
        case XN_MODULE_PROPERTY_LEAN_INIT:                // unsigned long long
        case XN_MODULE_PROPERTY_SERIAL_NUMBER:            // unsigned long long
        case XN_MODULE_PROPERTY_VERSION:                  // XnVersions
        // xn commands
        case XN_MODULE_PROPERTY_FIRMWARE_PARAM:           // XnInnerParam
        case XN_MODULE_PROPERTY_RESET:                    // unsigned long long
        case XN_MODULE_PROPERTY_IMAGE_CONTROL:            // XnControlProcessingData
        case XN_MODULE_PROPERTY_DEPTH_CONTROL:            // XnControlProcessingData
        case XN_MODULE_PROPERTY_AHB:                      // XnAHBData
        case XN_MODULE_PROPERTY_LED_STATE:                // XnLedState
          return ONI_STATUS_NOT_SUPPORTED;

        case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:      // OniImageRegistrationMode
          if (dataSize != sizeof(OniImageRegistrationMode))
          {
            LogError("Unexpected size for ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION");
            return ONI_STATUS_ERROR;
          }
          return depth->setImageRegistrationMode(*(static_cast<const OniImageRegistrationMode*>(data)));
      }
    }

    OniBool isCommandSupported(int commandId)
    {
      switch (commandId)
      {
        default:
        case ONI_DEVICE_COMMAND_SEEK:
          return false;
      }
    }
    
    OniStatus invoke(int commandId, void* data, int dataSize)
    {
      switch (commandId)
      {
        default:
        case ONI_DEVICE_COMMAND_SEEK: // OniSeek
          return ONI_STATUS_NOT_SUPPORTED;
      }
    }

    void stop()
    {

    }

    void close()
    {

    }

    /* todo: for DeviceBase
    virtual OniStatus tryManualTrigger() {return ONI_STATUS_OK;}
    */
  };


  class Driver : public oni::driver::DriverBase //, private Freenect::Freenect
  {
  private:
    typedef std::map<OniDeviceInfo, oni::driver::DeviceBase*> OniDeviceMap;
    OniDeviceMap devices;

    static std::string devid_to_uri(int id) {
      return "ros://" + std::to_string(id);
    }

    static int uri_to_devid(const std::string uri) {
      int id;
      std::istringstream is(uri);
      is.seekg(strlen("ros://"));
      is >> id;
      return id;
    }

  public:
    Driver(OniDriverServices* pDriverServices) : DriverBase(pDriverServices)
    {
      //WriteMessage("Using libfreenect v" + to_string(PROJECT_VER));

      //freenect_set_log_level(m_ctx, FREENECT_LOG_DEBUG);
      //freenect_select_subdevices(m_ctx, FREENECT_DEVICE_CAMERA); // OpenNI2 doesn't use MOTOR or AUDIO
      pDriverServices =  (OniDriverServices*)&getServices();
    }
    ~Driver() { shutdown(); }

    // for DriverBase

    OniStatus initialize(oni::driver::DeviceConnectedCallback connectedCallback, oni::driver::DeviceDisconnectedCallback disconnectedCallback, oni::driver::DeviceStateChangedCallback deviceStateChangedCallback, void* pCookie)
    {
      DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie);
#if 0      
      for (int i = 0; i < Freenect::deviceCount(); i++)
      {
        std::string uri = devid_to_uri(i);

        WriteMessage("Found device " + uri);
        
        OniDeviceInfo info;
        strncpy(info.uri, uri.c_str(), ONI_MAX_STR);
        strncpy(info.vendor, "Microsoft", ONI_MAX_STR);
        strncpy(info.name, "Kinect", ONI_MAX_STR);
        devices[info] = NULL;
        deviceConnected(&info);
        deviceStateChanged(&info, 0);

        freenect_device* dev;
        if (freenect_open_device(m_ctx, &dev, i) == 0)
        {
          info.usbVendorId = dev->usb_cam.VID;
          info.usbProductId = dev->usb_cam.PID;
          freenect_close_device(dev);
        }
        else
        {
          WriteMessage("Unable to open device to query VID/PID");
        }
  #endif
      return ONI_STATUS_OK;
    }

    /// TBD
    oni::driver::DeviceBase* deviceOpen(const char* uri, const char* mode = NULL)
    {
      for (OniDeviceMap::iterator iter = devices.begin(); iter != devices.end(); iter++)
      {
        if (strcmp(iter->first.uri, uri) == 0) // found
        {
          if (iter->second) // already open
          {
            return iter->second;
          }
          else 
          {
            //WriteMessage("Opening device " + std::string(uri));
            int id = uri_to_devid(iter->first.uri);
            Device* device = new Device(0,id);
            iter->second = device;
            return 0; // device
          }
        }
      }

      LogError("Could not find device " + std::string(uri));
      return NULL;
    }

    void deviceClose(oni::driver::DeviceBase* pDevice)
    {
      for (OniDeviceMap::iterator iter = devices.begin(); iter != devices.end(); iter++)
      {
        if (iter->second == pDevice)
        {
          Device* device = (Device*)iter->second;
          device->stop();
          device->close();
          devices.erase(iter);
          return;
        }
      }

      LogError("Could not close unrecognized device");
    }

    OniStatus tryDevice(const char* uri)
    {
      oni::driver::DeviceBase* device = deviceOpen(uri);
      if (! device)
        return ONI_STATUS_ERROR;
      deviceClose(device);
      return ONI_STATUS_OK;
    }

    void shutdown()
    {
      for (OniDeviceMap::iterator iter = devices.begin(); iter != devices.end(); iter++)
      {
        if(iter->second)
          deviceClose(iter->second);
      }

      devices.clear();
    }


    /* todo: for DriverBase
    virtual void* enableFrameSync(oni::driver::StreamBase** pStreams, int streamCount);
    virtual void disableFrameSync(void* frameSyncGroup);
    */
  };
}


// macros defined in XnLib (not included) - workaround
#define XN_NEW(type, arg...) new type(arg)
#define XN_DELETE(p) delete(p)
ONI_EXPORT_DRIVER(RosDriver::Driver);
#undef XN_NEW
#undef XN_DELETE
