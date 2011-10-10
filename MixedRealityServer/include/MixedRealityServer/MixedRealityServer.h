#ifndef MIXED_REALITY_SERVER_H
#define MIXED_REALITY_SERVER_H

#include "MixedRealityServer/VirtualLayer.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "MixedRealityServer/DrawObject.h"

//Maximum number of server sockets to listen.
#define MAX_NUM_SOCKETS    100
#define IO_BUFFER_SIZE     256
#define BUFFER_SIZE        1024

namespace MixedRealityServer
{
    // The webserver determines between these values for an answer.
    typedef enum
    {
        ANSWER_UNKNOWN,
        ANSWER_STREAM,
        ANSWER_SNAPSHOT,
    } AnswerType;


    // The client sends information with each request and
    // this structure is used to store the important parts.
    typedef struct
    {
        AnswerType type;
        char *parameter;
        char *client;
        char *credentials;
    } HTTP_Request;

    // The iobuffer structure is used to read from the HTTP-client
    typedef struct
    {
        int level;                   // how full is the buffer
        char buffer[IO_BUFFER_SIZE]; // the data
    } IO_Buffer;

    class ImageBuffer
    {
        public:
          ImageBuffer() {}
          sensor_msgs::Image Msg;
          boost::condition_variable Condition;
          boost::mutex Mutex;
    };

    class MRServer
    {
        public:
          MRServer(ros::NodeHandle& node);
          virtual ~MRServer();
          void Execute();
          void Spin();
          void Stop();
          void CleanUp();
          void Clicked(float x, float y);

          // Client thread function
          // Serves a connected TCP-client. This thread function is called
          // for each connect of a HTTP client like a webbrowser. It determines
          // if it is a valid HTTP request and dispatches between the different
          // response options.
          void Client(int fd);
          
          void SetMap(IplImage* img, float res);

		  void DrawCommand(MixedRealityServer::DrawObject obj);

        private:
          typedef std::map<std::string, ImageBuffer*> ImageBufferMap;
          typedef std::map<std::string, image_transport::Subscriber> ImageSubscriberMap;
          typedef std::map<std::string, std::string> ParameterMap;
          typedef std::map<std::string, VirtualLayer*> VirtualLayerMap;
          
          void getOdometry();
		  
          ImageBuffer* getImageBuffer(const std::string& topic);

          void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic);

          void copyBuffer(std::vector<uchar>& buffer, ImageBuffer* image_buffer, const ros::Time& timestamp);

          // Rotate input image by 180 degrees.
          void invertImage(const cv::Mat& input, cv::Mat& output);

          // Send an error message.
          void sendError(int fd, int which, const char *message);

          // Send a complete HTTP response and a stream of JPG-frames.
          void sendStream(int fd, const char *parameter);

          // Send a complete HTTP response and a single JPG-frame.
          void sendSnapshot(int fd, const char *parameter);

          //Initializes the iobuffer structure properly.
          void initIOBuffer(IO_Buffer *iobuf);

          //Initializes the request structure properly.
          void initRequest(HTTP_Request *req);

          // If strings were assigned to the different members free them.
          // This will fail if strings are static, so always use strdup().
          void freeRequest(HTTP_Request *req);

          // Read with timeout, implemented without using signals.
          // Tries to read len bytes and returns if enough bytes were read
          // or the timeout was triggered. In case of timeout the return
          // value may differ from the requested bytes "len".
          // fd     - fildescriptor to read from
          // iobuf  - iobuffer that allows to use this functions from multiple
          //          threads because the complete context is the iobuffer.
          // buffer - The buffer to store values at, will be set to zero
          //          before storing values.
          // len    - the length of buffer
          // timeout- seconds to wait for an answer
          //
          // Results:
          // buffer - will become filled with bytes read
          // iobuf  - May get altered to save the context for future calls.
          // Returns:
          // Bytes copied to buffer or -1 in case of error
          int readWithTimeout(int fd, IO_Buffer *iobuf, char *buffer, size_t len, int timeout);

          // Read a single line from the provided fildescriptor.
          // This funtion will return under two conditions:
          //     - line end was reached
          //     - timeout occured
          // fd     - fildescriptor to read from
          // iobuf  - iobuffer that allows to use this functions from multiple
          //          threads because the complete context is the iobuffer.
          // buffer - The buffer to store values at, will be set to zero
          //          before storing values.
          // len    - the length of buffer
          // timeout- seconds to wait for an answer
          //
          // Results:
          // buffer - will become filled with bytes read
          // iobuf  - May get altered to save the context for future calls.
          // Returns:
          // Bytes copied to buffer or -1 in case of error
          int readLineWithTimeout(int fd, IO_Buffer *iobuf, char *buffer, size_t len, int timeout);

          // Decodes the data and stores the result to the same buffer.
          // The buffer will be large enough, because base64 requires more space then plain text.
          void decodeBase64(char *data);

          // Convert a hexadecimal ASCII character to integer
          // Returns:
          // The corresponding value between 0 and 15, or -1 in case of error
          int hexCharToInt(char in);

          // Replaces %XX with the character code it represents, URI
          // Returns:
          // 0 if everything is ok, -1 in case of error
          int unescape(char *string);

          // Split string into a list of tokens
          // Returns:
          // Vector of tokens
          void splitString(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiter = " ");

          // Convert a string to an integer
          // Returns:
          // corresponding value, default_value in case of error
          int stringToInt(const std::string& str, const int default_value = 0);

          // Decodes URI parameters in form of ?parameter=value
          // parameter - URI string
          // Results:
          // parameter_map - a map of parameter/value pairs
          void decodeParameter(const std::string& parameter, ParameterMap& parameter_map);

          ros::NodeHandle node_;
          image_transport::ImageTransport image_transport_;
          int port_;

          int sd[MAX_NUM_SOCKETS];
          int sd_len;

          bool stop_requested_;
          char* www_folder_;

          ImageBufferMap image_buffers_;
          VirtualLayerMap virtual_layers_;
          ImageSubscriberMap image_subscribers_;
          boost::mutex image_maps_mutex_;
          
          IplImage* map;
          float ppm;

          IplImage* pos_map;
          boost::mutex 		odom_mutex_;
          boost::condition 	odom_condition_;

		  boost::mutex		virtual_layer_mutex_;
		  boost::condition  virtual_layer_condition_;

    };
}

#endif

