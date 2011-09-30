#include "MixedRealityServer/MixedRealityServer.h"

#include "MixedRealityServer/Contour2D.h"

#include "tf/transform_listener.h"

#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <syslog.h>
#include <netdb.h>
#include <errno.h>
#include <string>

#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include<iostream>

#define ABS(a) (((a) < 0) ? -(a) : (a))
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#define LENGTH_OF(x) (sizeof(x)/sizeof(x[0]))

// the boundary is used for the M-JPEG stream, it separates the multipart stream of pictures */
#define BOUNDARY "boundarydonotcross"

// this defines the buffer size for a JPG-frame
#define MAX_FRAME_SIZE (256*1024)
#define TEN_K (10*1024)

//Standard header to be send along with other header information like mimetype.
#define STD_HEADER "Connection: close\r\n" \
    "Server: MixedRealityServer\r\n" \
    "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n" \
    "Pragma: no-cache\r\n" \
    "Expires: Fri, 10 Dec 2010 18:22:23 GMT\r\n"

#define MAP_ZOOM_FACTOR 3.2f

namespace MixedRealityServer
{
    MRServer::MRServer(ros::NodeHandle& node) :
            node_(node), image_transport_(node), stop_requested_(false), www_folder_(NULL)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("port", port_, 8080);
		
		pos_map = cvCreateImage(cvSize(320, 320), IPL_DEPTH_8U,1);
        boost::thread t(boost::bind( &MRServer::getOdometry, this));
        t.detach();
        
        //image_subscribers_[topic] = image_transport_.subscribe(topic, 1, boost::bind(&MRServer::imageCallback, this, _1, topic));
        
    }

    MRServer::~MRServer()
    {
    
    	cvReleaseImage(&map);
        CleanUp();
    }
	
	void MRServer::getOdometry()
	{
		ros::Rate rate(5.0);

		tf::TransformListener listener;
	 	while (!stop_requested_)
	 	{	 
	 		tf::StampedTransform transform;	
			try
			{
		  		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
		  		ROS_ERROR("%s", ex.what());
		  		rate.sleep();
		  		continue;
			}
			double x = transform.getOrigin().x() / ppm + map->width / 2;
			double y = map->height - (transform.getOrigin().y() / ppm + map->height / 2);

			//ROS_INFO("%d %d", (int)x, (int)y);

			boost::unique_lock<boost::mutex> lock(odom_mutex_);

			pos_map = cvCloneImage(map);
			cvCircle(pos_map, cvPoint((int)x,(int)y), 10, cvScalar(0,0,0), 1);			
			odom_condition_.notify_all();

			rate.sleep();
		}
	}
    void MRServer::copyBuffer(std::vector<uchar>& buffer, ImageBuffer* image_buffer, const ros::Time& timestamp)
    {
        //  int buffer_size = buffer.size();
        //  if(buffer_size == 0)
        //    return;
        //
        //  // check if image buffer is large enough, increase it if necessary
        //  if(buffer_size > image_buffer->buffer_size_) {
        //    ROS_DEBUG("increasing buffer size to %d", buffer_size);
        //    image_buffer->buffer_ = (char*)realloc(image_buffer->buffer_, buffer_size);
        //    image_buffer->buffer_size_ = buffer_size;
        //  }
        //
        //  // copy image buffer
        //  memcpy(image_buffer->buffer_, &buffer[0], buffer_size);
        //  image_buffer->time_stamp_ = timestamp.toSec();
    }

    void MRServer::imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic)
    {
        //ROS_INFO("imageCallback");
        ImageBuffer* image_buffer = getImageBuffer(topic);
        boost::unique_lock<boost::mutex> lock(image_buffer->Mutex);
        // copy image
        image_buffer->Msg = *msg;
        // notify senders
        image_buffer->Condition.notify_all();
    }


    void MRServer::splitString(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiter)
    {
        // Skip delimiters at beginning.
        std::string::size_type lastPos = str.find_first_not_of(delimiter, 0);
        // Find first "non-delimiter".
        std::string::size_type pos     = str.find_first_of(delimiter, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos)
        {
            // Found a token, add it to the vector.
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            // Skip delimiters.  Note the "not_of"
            lastPos = str.find_first_not_of(delimiter, pos);
            // Find next "non-delimiter"
            pos = str.find_first_of(delimiter, lastPos);
        }
    }

    int MRServer::stringToInt(const std::string& str, const int default_value)
    {
        int value;
        int res;
        if (str.length() == 0)
            return default_value;
        res = sscanf(str.c_str(),"%i",&value);
        if (res == 1)
            return value;
        return default_value;
    }

    void MRServer::initIOBuffer(IO_Buffer *iobuf)
    {
        memset(iobuf->buffer, 0, sizeof(iobuf->buffer));
        iobuf->level = 0;
    }

    void MRServer::initRequest(HTTP_Request *req)
    {
        req->type        = ANSWER_UNKNOWN;
        req->parameter   = NULL;
        req->client      = NULL;
        req->credentials = NULL;
    }

    void MRServer::freeRequest(HTTP_Request *req)
    {
        if (req->parameter != NULL) free(req->parameter);
        if (req->client != NULL) free(req->client);
        if (req->credentials != NULL) free(req->credentials);
    }

    int MRServer::readWithTimeout(int fd, IO_Buffer *iobuf, char *buffer, size_t len, int timeout)
    {
        unsigned int copied = 0, i;
        int rc;
        fd_set fds;
        struct timeval tv;

        memset(buffer, 0, len);

        while (copied < len)
        {
            i = MIN((unsigned)iobuf->level, (unsigned)(len - copied));
            memcpy(buffer + copied, iobuf->buffer + IO_BUFFER_SIZE - iobuf->level, i);

            iobuf->level -= i;
            copied += i;
            if (copied >= len)
                return copied;

            /* select will return in case of timeout or new data arrived */
            tv.tv_sec = timeout;
            tv.tv_usec = 0;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            if ((rc = select(fd + 1, &fds, NULL, NULL, &tv)) <= 0)
            {
                if (rc < 0)
                    exit(EXIT_FAILURE);

                /* this must be a timeout */
                return copied;
            }

            initIOBuffer(iobuf);

            /*
             * there should be at least one byte, because select signalled it.
             * But: It may happen (very seldomly), that the socket gets closed remotly between
             * the select() and the following read. That is the reason for not relying
             * on reading at least one byte.
             */
            if ((iobuf->level = read(fd, &iobuf->buffer, IO_BUFFER_SIZE)) <= 0)
            {
                /* an error occured */
                return -1;
            }

            /* align data to the end of the buffer if less than IO_BUFFER_SIZE bytes were read */
            memmove(iobuf->buffer + (IO_BUFFER_SIZE - iobuf->level), iobuf->buffer, iobuf->level);
        }

        return 0;
    }

    int MRServer::readLineWithTimeout(int fd, IO_Buffer *iobuf, char *buffer, size_t len, int timeout)
    {
        char c = '\0', *out = buffer;
        unsigned int i;

        memset(buffer, 0, len);

        for (i = 0; i < len && c != '\n'; i++)
        {
            if (readWithTimeout(fd, iobuf, &c, 1, timeout) <= 0)
            {
                /* timeout or error occured */
                return -1;
            }
            *out++ = c;
        }

        return i;
    }

    void MRServer::decodeBase64(char *data)
    {
        const unsigned char *in = (const unsigned char *)data;
        /* The decoded size will be at most 3/4 the size of the encoded */
        unsigned ch = 0;
        int i = 0;

        while (*in)
        {
            int t = *in++;

            if (t >= '0' && t <= '9')
                t = t - '0' + 52;
            else if (t >= 'A' && t <= 'Z')
                t = t - 'A';
            else if (t >= 'a' && t <= 'z')
                t = t - 'a' + 26;
            else if (t == '+')
                t = 62;
            else if (t == '/')
                t = 63;
            else if (t == '=')
                t = 0;
            else
                continue;

            ch = (ch << 6) | t;
            i++;
            if (i == 4)
            {
                *data++ = (char)(ch >> 16);
                *data++ = (char)(ch >> 8);
                *data++ = (char) ch;
                i = 0;
            }
        }
        *data = '\0';
    }

    int MRServer::hexCharToInt(char in)
    {
        if (in >= '0' && in <= '9')
            return in - '0';

        if (in >= 'a' && in <= 'f')
            return (in - 'a') + 10;

        if (in >= 'A' && in <= 'F')
            return (in - 'A') + 10;

        return -1;
    }

    int MRServer::unescape(char *string)
    {
        char *source = string, *destination = string;
        int src, dst, length = strlen(string), rc;

        /* iterate over the string */
        for (dst = 0, src = 0; src < length; src++)
        {

            /* is it an escape character? */
            if (source[src] != '%')
            {
                /* no, so just go to the next character */
                destination[dst] = source[src];
                dst++;
                continue;
            }

            /* yes, it is an escaped character */

            /* check if there are enough characters */
            if (src + 2 > length)
            {
                return -1;
                break;
            }

            /* perform replacement of %## with the corresponding character */
            if ((rc = hexCharToInt(source[src+1])) == -1) return -1;
            destination[dst] = rc * 16;
            if ((rc = hexCharToInt(source[src+2])) == -1) return -1;
            destination[dst] += rc;

            /* advance pointers, here is the reason why the resulting string is shorter */
            dst++;
            src += 2;
        }

        /* ensure the string is properly finished with a null-character */
        destination[dst] = '\0';

        return 0;
    }

    void MRServer::sendError(int fd, int which, const char *message)
    {
        char buffer[BUFFER_SIZE] = {0};

        if (which == 401)
        {
            sprintf(buffer, "HTTP/1.0 401 Unauthorized\r\n" \
                    "Content-type: text/plain\r\n" \
                    STD_HEADER \
                    "WWW-Authenticate: Basic realm=\"MJPG-Streamer\"\r\n" \
                    "\r\n" \
                    "401: Not Authenticated!\r\n" \
                    "%s", message);
        }
        else if (which == 404)
        {
            sprintf(buffer, "HTTP/1.0 404 Not Found\r\n" \
                    "Content-type: text/plain\r\n" \
                    STD_HEADER \
                    "\r\n" \
                    "404: Not Found!\r\n" \
                    "%s", message);
        }
        else if (which == 500)
        {
            sprintf(buffer, "HTTP/1.0 500 Internal Server Error\r\n" \
                    "Content-type: text/plain\r\n" \
                    STD_HEADER \
                    "\r\n" \
                    "500: Internal Server Error!\r\n" \
                    "%s", message);
        }
        else if (which == 400)
        {
            sprintf(buffer, "HTTP/1.0 400 Bad Request\r\n" \
                    "Content-type: text/plain\r\n" \
                    STD_HEADER \
                    "\r\n" \
                    "400: Not Found!\r\n" \
                    "%s", message);
        }
        else
        {
            sprintf(buffer, "HTTP/1.0 501 Not Implemented\r\n" \
                    "Content-type: text/plain\r\n" \
                    STD_HEADER \
                    "\r\n" \
                    "501: Not Implemented!\r\n" \
                    "%s", message);
        }

        if (write(fd, buffer, strlen(buffer)) < 0)
        {
            ROS_DEBUG("write failed, done anyway");
        }
    }

    void MRServer::decodeParameter(const std::string& parameter, ParameterMap& parameter_map)
    {
        std::vector<std::string> parameter_value_pairs;
        splitString(parameter,parameter_value_pairs, "?&");

        for (size_t i=0; i<parameter_value_pairs.size(); i++)
        {
            std::vector<std::string> parameter_value;
            splitString(parameter_value_pairs[i], parameter_value, "=");
            if (parameter_value.size()==1)
            {
                parameter_map.insert(std::make_pair(parameter_value[0],std::string("")));
            }
            else if (parameter_value.size()==2)
            {
                parameter_map.insert(std::make_pair(parameter_value[0],parameter_value[1]));
            }
        }
    }

    ImageBuffer* MRServer::getImageBuffer(const std::string& topic)
    {
        boost::unique_lock<boost::mutex> lock(image_maps_mutex_);
        ImageSubscriberMap::iterator it = image_subscribers_.find(topic);
        if (it == image_subscribers_.end())
        {
          	if(topic.compare("/map") != 0)
		    {
		        ROS_INFO("Subscribing to topic %s", topic.c_str());
		        image_subscribers_[topic] = image_transport_.subscribe(topic, 1, boost::bind(&MRServer::imageCallback, this, _1, topic));
		        image_buffers_[topic] = new ImageBuffer();
		    }
		    virtual_layers_[topic] = new VirtualLayer();
		    ROS_INFO("Streaming...");
        }
        
        ImageBuffer* image_buffer = image_buffers_[topic];
        return image_buffer;
    }

    // rotate input image at 180 degrees
    void MRServer::invertImage(const cv::Mat& input, cv::Mat& output)
    {

        cv::Mat_<cv::Vec3b>& input_img = (cv::Mat_<cv::Vec3b>&)input; //3 channel pointer to image
        cv::Mat_<cv::Vec3b>& output_img = (cv::Mat_<cv::Vec3b>&)output; //3 channel pointer to image
        cv::Size size = input.size();

        for (int j = 0; j < size.height; ++j)
            for (int i = 0; i < size.width; ++i)
            {
                //outputImage.imageData[size.height*size.width - (i + j*size.width) - 1] = inputImage.imageData[i + j*size.width];
                output_img(size.height-j-1, size.width-i-1) = input_img(j,i);
            }
        return;
    }

    void MRServer::sendStream(int fd, const char *parameter)
    {
        unsigned char *frame = NULL, *tmp = NULL;
        int frame_size = 0, max_frame_size = 0;
        char buffer[BUFFER_SIZE] = {0};
        struct timeval timestamp;
        sensor_msgs::CvBridge image_bridge;

        ROS_DEBUG("Decoding parameter");

        std::string params = parameter;
        ParameterMap parameter_map;
        decodeParameter(params, parameter_map);

        ParameterMap::iterator itp = parameter_map.find("topic");
        if (itp == parameter_map.end()) return;

        std::string topic = itp->second;
        ImageBuffer* image_buffer = getImageBuffer(topic);

        ROS_DEBUG("preparing header");
        sprintf(buffer, "HTTP/1.0 200 OK\r\n" \
                STD_HEADER \
                "Content-Type: multipart/x-mixed-replace;boundary=" BOUNDARY "\r\n" \
                "\r\n" \
                "--" BOUNDARY "\r\n");

        if (write(fd, buffer, strlen(buffer)) < 0)
        {
            free(frame);
            return;
        }

        ROS_DEBUG("Headers send, sending stream now");
	
        while (!stop_requested_)
        {
            {
            	cv::Mat img;
            	IplImage* image;
            	
            	if(topic.compare("/mappos") == 0)
				{	
      			 	boost::unique_lock<boost::mutex> lock(odom_mutex_);
		            odom_condition_.wait(lock);
  					image = cvCreateImage(cvSize(map->height,map->width),IPL_DEPTH_8U, 3);
					image = cvCloneImage(pos_map);
				}
				else if(topic.compare("/map") == 0)
				{
					image = cvCloneImage(map);
				}
				else
				{
				
		            /* wait for fresh frames */
		            boost::unique_lock<boost::mutex> lock(image_buffer->Mutex);
		            image_buffer->Condition.wait(lock);


		            try
		            {
		                if (image_bridge.fromImage(image_buffer->Msg, "bgr8"))
		                {
		                    image = image_bridge.toIpl();
		                }
		                else
		                {
		                    ROS_ERROR("Unable to convert %s image to bgr8", image_buffer->Msg.encoding.c_str());
		                    return;
		                }
		            }
		            catch (...)
		            {
		                ROS_ERROR("Unable to convert %s image to ipl format", image_buffer->Msg.encoding.c_str());
		                return;
		            }
				}
	            //Draw the virtual 
				boost::unique_lock<boost::mutex> v_lock(virtual_layer_mutex_);
				//ROS_INFO("Objects: %d; %s", virtual_layers_[topic]->GetObjectsCount(), topic.c_str());
	            virtual_layers_[topic]->DrawLayer(image);
				virtual_layer_condition_.notify_all();
				
				img = image;
			
                // encode image
           
                std::vector<uchar> encoded_buffer;
                std::vector<int> encode_params;

                // invert
                if (parameter_map.find("invert") != parameter_map.end())
                {
                    cv::Mat cloned_image = img.clone();
                    invertImage(cloned_image, img);
                }

                // quality
                int quality = 95;
                if (parameter_map.find("quality") != parameter_map.end())
                {
                    quality = stringToInt(parameter_map["quality"]);
                }
                encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                encode_params.push_back(quality);

                // resize image
                if (parameter_map.find("width") != parameter_map.end() && parameter_map.find("height") != parameter_map.end())
                {
                    int width = stringToInt(parameter_map["width"]);
                    int height = stringToInt(parameter_map["height"]);
                    if (width > 0 && height > 0)
                    {
                        cv::Mat img_resized;
                        cv::Size new_size(width,height);
                        cv::resize(img, img_resized, new_size);
                        cv::imencode(".jpeg", img_resized, encoded_buffer, encode_params);
                    }
                    else
                    {
                        cv::imencode(".jpeg", img, encoded_buffer, encode_params);
                    }
                }
                else
                {
                    cv::imencode(".jpeg", img, encoded_buffer, encode_params);
                }

                // copy encoded frame buffer
                frame_size = encoded_buffer.size();

                /* check if frame buffer is large enough, increase it if necessary */
                if (frame_size > max_frame_size)
                {
                    ROS_DEBUG("increasing frame buffer size to %d", frame_size);

                    max_frame_size = frame_size + TEN_K;
                    if ((tmp = (unsigned char*)realloc(frame, max_frame_size)) == NULL)
                    {
                        free(frame);
                        sendError(fd, 500, "not enough memory");
                        return;
                    }
                    frame = tmp;
                }

                /* copy v4l2_buffer timeval to user space */
                timestamp.tv_sec = ros::Time::now().toSec();

                memcpy(frame, &encoded_buffer[0], frame_size);
                ROS_DEBUG("got frame (size: %d kB)", frame_size / 1024);
                cvReleaseImage(&image);
            }

            /*
             * print the individual mimetype and the length
             * sending the content-length fixes random stream disruption observed
             * with firefox
             */
            sprintf(buffer, "Content-Type: image/jpeg\r\n" \
                    "Content-Length: %d\r\n" \
                    "X-Timestamp: %d.%06d\r\n" \
                    "\r\n", frame_size, (int)timestamp.tv_sec, (int)timestamp.tv_usec);
            ROS_DEBUG("sending intemdiate header");
            if (write(fd, buffer, strlen(buffer)) < 0) break;

            ROS_DEBUG("sending frame");
            if (write(fd, frame, frame_size) < 0) break;

            ROS_DEBUG("sending boundary");
            sprintf(buffer, "\r\n--" BOUNDARY "\r\n");
            if (write(fd, buffer, strlen(buffer)) < 0) break;
        }
		
		image_subscribers_.erase (topic);
        image_buffers_.erase(topic);
        virtual_layers_.erase(topic);

		
        free(frame);
    }

    void MRServer::sendSnapshot(int fd, const char *parameter)
    {
        unsigned char *frame = NULL;
        int frame_size = 0;
        char buffer[BUFFER_SIZE] = {0};
        struct timeval timestamp;
        sensor_msgs::CvBridge image_bridge;

        std::string params = parameter;
        ParameterMap parameter_map;
        decodeParameter(params, parameter_map);

        ParameterMap::iterator itp = parameter_map.find("topic");
        if (itp == parameter_map.end()) return;
		
		std::string topic = itp->second;
		
		cv::Mat img;
		if(topic.compare("/map") == 0)
		{
			img = cvCloneImage(map);
		}
		else
		{
        	ImageBuffer* image_buffer = getImageBuffer(topic);
        
			/* wait for fresh frames */
			boost::unique_lock<boost::mutex> lock(image_buffer->Mutex);
			image_buffer->Condition.wait(lock);

			IplImage* image;
			try
			{
				if (image_bridge.fromImage(image_buffer->Msg, "bgr8"))
				{
				    image = image_bridge.toIpl();
				}
				else
				{
				    ROS_ERROR("Unable to convert %s image to bgr8", image_buffer->Msg.encoding.c_str());
				    return;
				}
			}
			catch (...)
			{
				ROS_ERROR("Unable to convert %s image to ipl format", image_buffer->Msg.encoding.c_str());
				return;
			}
			img = image;
		}
			
		std::vector<uchar> encoded_buffer;
		std::vector<int> encode_params;

		// invert
		if (parameter_map.find("invert") != parameter_map.end())
		{
			cv::Mat cloned_image = img.clone();
			invertImage(cloned_image, img);
		}

		// quality
		int quality = 95;
		if (parameter_map.find("quality") != parameter_map.end())
		{
			quality = stringToInt(parameter_map["quality"]);
		}
		encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
		encode_params.push_back(quality);

		// resize image
		if (parameter_map.find("width") != parameter_map.end() && parameter_map.find("height") != parameter_map.end())
		{
			int width = stringToInt(parameter_map["width"]);
			int height = stringToInt(parameter_map["height"]);
			if (width > 0 && height > 0)
			{
			    cv::Mat img_resized;
			    cv::Size new_size(width,height);
			    cv::resize(img, img_resized, new_size);
			    cv::imencode(".jpeg", img_resized, encoded_buffer, encode_params);
			}
			else
			{
			    cv::imencode(".jpeg", img, encoded_buffer, encode_params);
			}
		}
		else
		{
			cv::imencode(".jpeg", img, encoded_buffer, encode_params);
		}

		// copy encoded frame buffer
		frame_size = encoded_buffer.size();

		// resize buffer
		if ((frame = (unsigned char*)malloc(frame_size)) == NULL)
		{
			free(frame);
			sendError(fd, 500, "not enough memory");
			return;
		}

		/* copy v4l2_buffer timeval to user space */
		timestamp.tv_sec = ros::Time::now().toSec();

		memcpy(frame, &encoded_buffer[0], frame_size);
		ROS_DEBUG("got frame (size: %d kB)", frame_size / 1024);

		/* write the response */
		sprintf(buffer, "HTTP/1.0 200 OK\r\n" \
			    STD_HEADER \
			    "Content-type: image/jpeg\r\n" \
			    "X-Timestamp: %d.%06d\r\n" \
			    "\r\n", (int) timestamp.tv_sec, (int) timestamp.tv_usec);

		/* send header and image now */
		if (write(fd, buffer, strlen(buffer)) < 0 || \
			    write(fd, frame, frame_size) < 0)
		{
			free(frame);
			return;
	
		}
		image_subscribers_.erase (topic);
        image_buffers_.erase(topic);
        virtual_layers_.erase(topic);
        free(frame);
    }

    void MRServer::Client(int fd)
    {
        int cnt;
        char buffer[BUFFER_SIZE] = {0}, *pb = buffer;
        IO_Buffer iobuf;
        HTTP_Request req;

        /* initializes the structures */
        initIOBuffer(&iobuf);
        initRequest(&req);
		
		
        /* What does the client want to receive? Read the request. */
        memset(buffer, 0, sizeof(buffer));
        if ((cnt = readLineWithTimeout(fd, &iobuf, buffer, sizeof(buffer) - 1, 5)) == -1)
        {
            close(fd);
            return;
        }

        /* determine what to deliver */
        if (strstr(buffer, "GET /?") != NULL)
        {
            req.type = ANSWER_STREAM;

            /* advance by the length of known string */
            if ((pb = strstr(buffer, "GET /")) == NULL)
            {
                ROS_DEBUG("HTTP request seems to be malformed");
                sendError(fd, 400, "Malformed HTTP request");
                close(fd);
                return;
            }
            pb += strlen("GET /"); // a pb points to the string after the first & after command
            int len = MIN(strspn(pb, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ._/-1234567890?="), 100);
            req.parameter = (char*)malloc(len + 1);
            if (req.parameter == NULL)
            {
                exit(EXIT_FAILURE);
            }
            memset(req.parameter, 0, len + 1);
            strncpy(req.parameter, pb, len);

            ROS_DEBUG("requested image topic: \"%s\"", req.parameter);
        }
        else if (strstr(buffer, "GET /stream?") != NULL)
        {
            req.type = ANSWER_STREAM;

            /* advance by the length of known string */
            if ((pb = strstr(buffer, "GET /stream")) == NULL)
            {
                ROS_DEBUG("HTTP request seems to be malformed");
                sendError(fd, 400, "Malformed HTTP request");
                close(fd);
                return;
            }
            pb += strlen("GET /stream"); // a pb points to the string after the first & after command
            int len = MIN(strspn(pb, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ._/-1234567890?="), 100);
            req.parameter = (char*)malloc(len + 1);
            if (req.parameter == NULL)
            {
                exit(EXIT_FAILURE);
            }
            memset(req.parameter, 0, len + 1);
            strncpy(req.parameter, pb, len);

            ROS_DEBUG("requested image topic: \"%s\"", req.parameter);
        }
        else if (strstr(buffer, "GET /snapshot?") != NULL)
        {
            req.type = ANSWER_SNAPSHOT;

            /* advance by the length of known string */
            if ((pb = strstr(buffer, "GET /snapshot")) == NULL)
            {
                ROS_DEBUG("HTTP request seems to be malformed");
                sendError(fd, 400, "Malformed HTTP request");
                close(fd);
                return;
            }
            pb += strlen("GET /snapshot"); // a pb points to the string after the first & after command
            int len = MIN(strspn(pb, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ._/-1234567890?="), 100);
            req.parameter = (char*)malloc(len + 1);
            if (req.parameter == NULL)
            {
                exit(EXIT_FAILURE);
            }
            memset(req.parameter, 0, len + 1);
            strncpy(req.parameter, pb, len);

            ROS_DEBUG("requested image topic: \"%s\"", req.parameter);
        }

        /*
         * parse the rest of the HTTP-request
         * the end of the request-header is marked by a single, empty line with "\r\n"
         */
        do
        {
            memset(buffer, 0, sizeof(buffer));

            if ((cnt = readLineWithTimeout(fd, &iobuf, buffer, sizeof(buffer) - 1, 5)) == -1)
            {
                freeRequest(&req);
                close(fd);
                return;
            }

            if (strstr(buffer, "User-Agent: ") != NULL)
            {
                req.client = strdup(buffer + strlen("User-Agent: "));
            }
            else if (strstr(buffer, "Authorization: Basic ") != NULL)
            {
                req.credentials = strdup(buffer + strlen("Authorization: Basic "));
                decodeBase64(req.credentials);
                ROS_DEBUG("username:password: %s", req.credentials);
            }

        }
        while (cnt > 2 && !(buffer[0] == '\r' && buffer[1] == '\n'));

        /* now it's time to answer */
        switch (req.type)
        {
        case ANSWER_STREAM:
        {
            ROS_INFO("Request for streaming");
            sendStream(fd, req.parameter);
            break;
        }
        case ANSWER_SNAPSHOT:
        {
            ROS_INFO("Request for snapshot");
            sendSnapshot(fd, req.parameter);
            break;
        }
        default:
            ROS_DEBUG("unknown request");
        }

        close(fd);
        freeRequest(&req);
	
        ROS_INFO("Disconnecting HTTP client");
        return;
    }

    void MRServer::Execute()
    {

        ROS_INFO("Starting mjpeg server");

        struct addrinfo *aip, *aip2;
        struct addrinfo hints;
        struct sockaddr_storage client_addr;
        socklen_t addr_len = sizeof(struct sockaddr_storage);
        fd_set selectfds;
        int max_fds = 0;

        char name[NI_MAXHOST];

        bzero(&hints, sizeof(hints));
        hints.ai_family = PF_UNSPEC;
        hints.ai_flags = AI_PASSIVE;
        hints.ai_socktype = SOCK_STREAM;

        int err;
        snprintf(name, sizeof(name), "%d", port_);
        if ((err = getaddrinfo(NULL, name, &hints, &aip)) != 0)
        {
            perror(gai_strerror(err));
            exit(EXIT_FAILURE);
        }

        for (int i = 0; i < MAX_NUM_SOCKETS; i++)
            sd[i] = -1;


        /* open sockets for server (1 socket / address family) */
        int i = 0;
        int on;
        for (aip2 = aip; aip2 != NULL; aip2 = aip2->ai_next)
        {
            if ((sd[i] = socket(aip2->ai_family, aip2->ai_socktype, 0)) < 0)
            {
                continue;
            }

            /* ignore "socket already in use" errors */
            on = 1;
            if (setsockopt(sd[i], SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
            {
                perror("setsockopt(SO_REUSEADDR) failed");
            }

            /* IPv6 socket should listen to IPv6 only, otherwise we will get "socket already in use" */
            on = 1;
            if (aip2->ai_family == AF_INET6 && setsockopt(sd[i], IPPROTO_IPV6, IPV6_V6ONLY, (const void *)&on , sizeof(on)) < 0)
            {
                perror("setsockopt(IPV6_V6ONLY) failed");
            }

            /* perhaps we will use this keep-alive feature oneday */
            /* setsockopt(sd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on)); */

            if (bind(sd[i], aip2->ai_addr, aip2->ai_addrlen) < 0)
            {
                perror("bind");
                sd[i] = -1;
                continue;
            }

            if (listen(sd[i], 10) < 0)
            {
                perror("listen");
                sd[i] = -1;
            }
            else
            {
                i++;
                if (i >= MAX_NUM_SOCKETS)
                {
                    ROS_ERROR("Maximum number of server sockets exceeded");
                    i--;
                    break;
                }
            }
        }

        sd_len = i;

        if (sd_len < 1)
        {
            ROS_ERROR("Bind(%d) failed", port_);
            closelog();
            exit(EXIT_FAILURE);
        }
        else
        {
            ROS_INFO("Bind(%d) succeeded", port_);
        }

        ROS_INFO("waiting for clients to connect");

        while (!stop_requested_)
        {	
            do
            {
                FD_ZERO(&selectfds);

                for (i = 0; i < MAX_NUM_SOCKETS; i++)
                {
                    if (sd[i] != -1)
                    {
                        FD_SET(sd[i], &selectfds);

                        if (sd[i] > max_fds)
                            max_fds = sd[i];
                    }
                }

                err = select(max_fds + 1, &selectfds, NULL, NULL, NULL);

                if (err < 0 && errno != EINTR)
                {
                    perror("select");
                    exit(EXIT_FAILURE);
                }
            }
            while (err <= 0);

            ROS_INFO("Client connected");

            for (i = 0; i < max_fds + 1; i++)
            {
                if (sd[i] != -1 && FD_ISSET(sd[i], &selectfds))
                {
                    int fd = accept(sd[i], (struct sockaddr *)&client_addr, &addr_len);

                    /* start new thread that will handle this TCP connected client */
                    ROS_DEBUG("create thread to handle client that just established a connection");

                    if (getnameinfo((struct sockaddr *)&client_addr, addr_len, name, sizeof(name), NULL, 0, NI_NUMERICHOST) == 0)
                    {
                        syslog(LOG_INFO, "serving client: %s\n", name);
                    }

                    boost::thread t(boost::bind( &MRServer::Client, this, fd ));
                    t.detach();
                }
            }
        }

        ROS_INFO("leaving server thread, calling cleanup function now");
        CleanUp();
    }

    void MRServer::CleanUp()
    {
        ROS_INFO("cleaning up ressources allocated by server thread");

        for (int i = 0; i < MAX_NUM_SOCKETS; i++)
            close(sd[i]);
    }

    void MRServer::Spin()
    {
        boost::thread t(boost::bind( &MRServer::Execute, this ));
        t.detach();
        
        ros::spin();
        ROS_INFO("stop requested");
        Stop();
    }

    void MRServer::Stop()
    {
        stop_requested_ = true;
    }
    
    void MRServer::SetMap(IplImage* img, float res)
    {
    	IplImage *tmp = cvCreateImage(cvSize(img->height, img->width),IPL_DEPTH_8U, 3);
		cvConvertImage(img, tmp);
		map = cvCreateImage(cvSize(MAP_ZOOM_FACTOR * img->height, MAP_ZOOM_FACTOR * img->width), IPL_DEPTH_8U, 3);
		cvResize(tmp, map);
    	ppm = res;
    	cvReleaseImage(&tmp);
    }

	void MRServer::DrawCommand(MixedRealityServer::DrawObject obj)
	{
		if(virtual_layers_.find(obj.topic) != virtual_layers_.end())
		{
			boost::unique_lock<boost::mutex> lock(virtual_layer_mutex_);
			if(!obj.command.compare("ADD"))
			{	
				float x = obj.x / ppm + map->width / 2;
				float y = map->height - (obj.y / ppm + map->height / 2);
				float w = obj.width / ppm;
				float h = obj.height / ppm;
				
				std::string reds = obj.clr.substr(0, 2);    
				std::string greens = obj.clr.substr(2, 2);    
				std::string blues = obj.clr.substr(4, 2);    
				int red = strtoul(reds.c_str(), NULL, 16);
				int green = strtoul(greens.c_str(), NULL, 16);
				int blue = strtoul(blues.c_str(), NULL, 16);

				Contour2D* cntr;
				cntr = new Contour2D(obj.id, 
				  					 (ContourType) obj.type, 
				  					 obj.label, 
				  					 cvPoint((int)x, (int)y), 
				  					 cvSize((int)w, (int)h), 
				  					 obj.angle, 
				  					 cvScalar(red, green, blue));
				  					 
				virtual_layers_[obj.topic]->AddObject(cntr);
			}
			if(!obj.command.compare("REMOVE"))
			{
				if(obj.id == -10)
					virtual_layers_[obj.topic]->RemoveAllObjects();
				else
					virtual_layers_[obj.topic]->RemoveObject(obj.id);
			}
			virtual_layer_condition_.notify_all();
		}
	}
}



