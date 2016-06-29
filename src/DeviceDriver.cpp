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

#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>


namespace enc = sensor_msgs::image_encodings;

#if 0
static template<typename T>
void convert(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg,
                                      const PointCloud::Ptr& cloud_msg,
                                      int red_offset, int green_offset, int blue_offset, int color_step)
{
  // Use correct principal point from calibration
  float center_x = model_.cx();
  float center_y = model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model_.fx();
  float constant_y = unit_scaling / model_.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

  for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
  {
    for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
    {
      T depth = depth_row[u];

      // Check for invalid measurements
      if (!DepthTraits<T>::valid(depth))
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
      else
      {
        // Fill in XYZ
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = DepthTraits<T>::toMeters(depth);
      }

      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}
#endif


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
    std::string path;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    bool hostedInNode;
    boost::thread worker;
    image_geometry::PinholeCameraModel model_;
  public:
    Device(const char * apath):
      path(apath), 
      color(NULL),
      depth(NULL) ,
      pnh("~"),
      hostedInNode(false)
      {
        hostedInNode = getenv("ROS2OPENNI2") != 0;
        if(!hostedInNode)
        {
          int argc = 1;
          const char * argv[2] = {"none",NULL};
          ros::init(argc, (char**)&argv[0], "rosopenni2");
        }

      // TODO initialize

    ros::NodeHandle& private_nh = pnh;
    rgb_nh_.reset( new ros::NodeHandle(nh, (path + "rgb").c_str()));
    ros::NodeHandle depth_nh(nh, (path + "depth_registered").c_str());
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


      if(!hostedInNode)
      {
          worker = boost::thread(boost::bind(&Device::deviceLoop,this));
      }

    }

    ~Device()
    {
      if(!hostedInNode)
      {
        ros::shutdown(); // triggers the exit from the loop
        worker.join();
      }
      destroyStream(color);
      destroyStream(depth);

    }

    // this is the device thread that invokes ros (for this device or for all, whatever)
    void deviceLoop()
    {
        ros::Rate rate(30);
        while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg_in,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        // DONE inside the worker thread or in some thread of the ROS node
        // if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
        model_.fromCameraInfo(info_msg);

        // Check if the input image has to be resized
        sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
        if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
        {
          sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
          info_msg_tmp.width = depth_msg->width;
          info_msg_tmp.height = depth_msg->height;
          float ratio = float(depth_msg->width)/float(rgb_msg->width);
          info_msg_tmp.K[0] *= ratio;
          info_msg_tmp.K[2] *= ratio;
          info_msg_tmp.K[4] *= ratio;
          info_msg_tmp.K[5] *= ratio;
          info_msg_tmp.P[0] *= ratio;
          info_msg_tmp.P[2] *= ratio;
          info_msg_tmp.P[5] *= ratio;
          info_msg_tmp.P[6] *= ratio;
          model_.fromCameraInfo(info_msg_tmp);

#if 0
          cv_bridge::CvImageConstPtr cv_ptr;
          try
          {
            cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
          }
          catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
          }
          cv_bridge::CvImage cv_rsz;
          cv_rsz.header = cv_ptr->header;
          cv_rsz.encoding = cv_ptr->encoding;
          cv::resize(cv_ptr->image.rowRange(0,depth_msg->height/ratio), cv_rsz.image, cv::Size(depth_msg->width, depth_msg->height));
          if ((rgb_msg->encoding == enc::RGB8) || (rgb_msg->encoding == enc::BGR8) || (rgb_msg->encoding == enc::MONO8))
            rgb_msg = cv_rsz.toImageMsg();
          else
            rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), enc::RGB8)->toImageMsg();
#endif
          //NODELET_ERROR_THROTTLE(5, "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
          //                       depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
          //return;
        } else
          rgb_msg = rgb_msg_in;

        // Supported color encodings: RGB8, BGR8, MONO8
        int red_offset, green_offset, blue_offset, color_step;
        if (rgb_msg->encoding == enc::RGB8)
        {
          red_offset   = 0;
          green_offset = 1;
          blue_offset  = 2;
          color_step   = 3;
        }
        else if (rgb_msg->encoding == enc::BGR8)
        {
          red_offset   = 2;
          green_offset = 1;
          blue_offset  = 0;
          color_step   = 3;
        }
        else if (rgb_msg->encoding == enc::MONO8)
        {
          red_offset   = 0;
          green_offset = 0;
          blue_offset  = 0;
          color_step   = 1;
        }
        else
        {
#if 0
          try
          {
            rgb_msg = cv_bridge::toCvCopy(rgb_msg, enc::RGB8)->toImageMsg();
          }
          catch (cv_bridge::Exception& e)
          {
            NODELET_ERROR_THROTTLE(5, "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
            return;
          }
#endif          
          red_offset   = 0;
          green_offset = 1;
          blue_offset  = 2;
          color_step   = 3;
        }

        if (depth_msg->encoding == enc::TYPE_16UC1)
        {
          //convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
        }
        else if (depth_msg->encoding == enc::TYPE_32FC1)
        {
          //convert<float>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
        }
        else
        {

        }

        // THEN IT IS READY FOR PUSH in the Color and DepthStream
    }

    // we support only registered depth to color
    OniBool isImageRegistrationModeSupported(OniImageRegistrationMode mode) { return depth->isImageRegistrationModeSupported(mode); }

    // color and depth, no IR whatsoever
    OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
    {
      *numSensors = 2;
      OniSensorInfo * sensors = new OniSensorInfo[*numSensors];
      sensors[0] = depth->getSensorInfo();
      sensors[1] = color->getSensorInfo();
      *pSensors = sensors;
      return ONI_STATUS_OK;
    }

    // create the stream  easy
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
      else if (pStream == color)
      {
        delete color;
        color = NULL;
      }
      else if (pStream == depth)
      {
        delete depth;
        depth = NULL;
      }
    }

    // todo: fill out properties
    OniBool isPropertySupported(int propertyId)
    {
      if (propertyId == ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION)
        return true;
      else
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

  public:
    Driver(OniDriverServices* pDriverServices) : DriverBase(pDriverServices)
    {
          pDriverServices =  (OniDriverServices*)&getServices();
    }
    ~Driver() 
    {
       shutdown(); 
    }

    // NO AUTOMATIC LOOKUP
    OniStatus initialize(oni::driver::DeviceConnectedCallback connectedCallback, oni::driver::DeviceDisconnectedCallback disconnectedCallback, oni::driver::DeviceStateChangedCallback deviceStateChangedCallback, void* pCookie)
    {
      DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie);
      return ONI_STATUS_OK;
    }

    /// we support devices via prefixes of the topics assuming we have the ros:// prefix in the uri
    oni::driver::DeviceBase* deviceOpen(const char* uri, const char* mode = NULL)
    {
      for (OniDeviceMap::iterator iter = devices.begin(); iter != devices.end(); iter++)
      {
        if (strcmp(iter->first.uri, uri) == 0) // found
        {
            return iter->second;
        }
      }

      if(strncmp(uri,"ros://",6) == 0)
      {
        Device * device = new Device(uri + 6);

        OniDeviceInfo info;
        strncpy(info.uri, uri, ONI_MAX_STR);
        strncpy(info.vendor, "ROS", ONI_MAX_STR);
        strncpy(info.name, "Kinect", ONI_MAX_STR);
        devices[info] = device;
        deviceConnected(&info);
        deviceStateChanged(&info, 0);
        return device;
      }
      else
        return NULL;
    }

    // close a given device by pointer
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
          delete device;
          return;
        }
      }
      LogError("Could not close unrecognized device");
    }

    /// try opening the given device
    OniStatus tryDevice(const char* uri)
    {
      oni::driver::DeviceBase* device = deviceOpen(uri);
      if (!device)
      {
        return ONI_STATUS_ERROR;
      }
      else
      {
        deviceClose(device);
        return ONI_STATUS_OK;
      }
    }

    void shutdown()
    {
      for (OniDeviceMap::iterator iter = devices.begin(); iter != devices.end(); iter++)
      {
          Device* device = (Device*)iter->second;
          device->stop();
          device->close();
          iter->second = NULL;
          delete device;
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
