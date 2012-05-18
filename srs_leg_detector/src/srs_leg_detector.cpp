

#include "ros/ros.h"

#include "laser_processor.h"
#include "calc_leg_features.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

//#include "rosrecord/Player.h"
//#include "rosbag/bag.h"

#include "srs_msgs/PositionMeasurement.h"
#include "srs_msgs/HS_distance.h"

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"

#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "srs_people_tracking_filter/tracker_kalman.h"
//#include "srs_people_tracking_filter/tracker_particle.h" // particle
#include "srs_people_tracking_filter/gaussian_pos_vel.h"

#include "srs_people_tracking_filter/state_pos_vel.h"
#include "srs_people_tracking_filter/rgb.h"

#include <algorithm>


#include "actionlib/client/simple_action_client.h"
#include "srs_decision_making/ExecutionAction.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

// services
#include <srs_leg_detector/DetectLegs.h>

typedef actionlib::SimpleActionClient <srs_decision_making::ExecutionAction> Client;

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;


static const double no_observation_timeout_s = 0.7;
static const double max_second_leg_age_s     = 2.0;
static const double max_track_jump_m         = 1; //1.0;
static const double max_meas_jump_m          = 1; //0.75; // 1.0
static const double leg_pair_separation_m    = 0.5;
//static const string fixed_frame              = "odom_combined";
static const string fixed_frame              = "/map";

static const double det_dist__for_pause      = 0.5; // the distance to the person when the robot decides to pause
static const double det_dist__for_resume      = 2.5; // the distance to the person when the robot decides to resume
//
//static const unsigned int num_particles=100; // particle





class SavedFeature
{
public:
	static int nextid;
	TransformListener& tfl_;

	BFL::StatePosVel sys_sigma_;
	  TrackerKalman filter_;
	//TrackerParticle filter_;  // particle

	string id_;
	string object_id;
	ros::Time time_;
	ros::Time meas_time_;

	Stamped<Point> position_;
	float dist_to_person_;

	//  leg tracker
	SavedFeature(Stamped<Point> loc, TransformListener& tfl)
	: tfl_(tfl),
	  sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
	    filter_("tracker_name",sys_sigma_)
	 // filter_("tracker_name",num_particles,sys_sigma_) // particle
	{
		char id[100];
		snprintf(id,100,"legtrack%d", nextid++);
		id_ = std::string(id);

		object_id = "";
		time_ = loc.stamp_;
		meas_time_ = loc.stamp_;

		try {
			tfl_.transformPoint(fixed_frame, loc, loc);
		} catch(...) {
			ROS_WARN("TF exception spot 6.");
		}
		StampedTransform pose( btTransform(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
	//	cout<<loc.m_floats[0]<<", "<<loc.m_floats[1]<<","<<loc.m_floats[2]<<endl;
		filter_.initialize(loc, prior_sigma, time_.toSec());

		StatePosVel est;
		filter_.getEstimate(est);

		updatePosition();
	}

	void propagate(ros::Time time)
	{
		time_ = time;

		filter_.updatePrediction(time.toSec());

		updatePosition();
	}

	void update(Stamped<Point> loc)
	{
		StampedTransform pose( btTransform(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
		tfl_.setTransform(pose);

		meas_time_ = loc.stamp_;
		time_ = meas_time_;

		SymmetricMatrix cov(3);
		cov = 0.0;
		cov(1,1) = 0.0025;
		cov(2,2) = 0.0025;
		cov(3,3) = 0.0025;

		filter_.updateCorrection(loc, cov);

		updatePosition();
	}

	double getLifetime()
	{
		return filter_.getLifetime();
	}

private:
	void updatePosition()
	{
		StatePosVel est;
		filter_.getEstimate(est);

		position_[0] = est.pos_[0];
		position_[1] = est.pos_[1];
		position_[2] = est.pos_[2];
		position_.stamp_ = time_;
		position_.frame_id_ = fixed_frame;
	}
};

int SavedFeature::nextid = 0;



/*
class MatchedFeature
{
public:
  SampleSet* candidate_;
  SavedFeature* closest_;
  float distance_;

  MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance)
  : candidate_(candidate)
  , closest_(closest)
  , distance_(distance)
  {}

  inline bool operator< (const MatchedFeature& b) const
  {
    return (distance_ <  b.distance_);
  }
};
 */

class Legs
{
public:
	Stamped<Point> loc_;
	string type_;
	Legs(Stamped<Point> loc, string type)
	: loc_(loc)
	, type_(type)
	{};

};

class Pair
{
public:
	Point pos1_;
	Point pos2_;
	float dist_sep_m;
	Pair(Point pos1,Point pos2,float dist)
	:pos1_(pos1)
	,pos2_(pos2)
	,dist_sep_m(dist)
	{};
};

class MatchedFeature
{
public:
	Legs* candidate_;
	SavedFeature* closest_;
	float distance_;

	MatchedFeature(Legs* candidate, SavedFeature* closest, float distance)
	: candidate_(candidate)
	, closest_(closest)
	, distance_(distance)
	{}

	inline bool operator< (const MatchedFeature& b) const
	{
		return (distance_ <  b.distance_);
	}
};


int g_argc;
char** g_argv;
string scan_topic = "scan_front";



// actual legdetector node
class LegDetector
{
 
 
 srs_decision_making::ExecutionGoal goal;  // goal that will be sent to the actionserver
 

 bool pauseSent;
 int counter;
 srs_msgs::HS_distance  distance_msg;
 
 Client client; // actionLib client for connection with DM action server


 ros::ServiceClient client_map;  // clent for the getMap service
 nav_msgs::GetMap srv_map;
        
 int ind_x, ind_y;

 short int map_data []; 

 double tmp;

 int indtmp;
  

public:
	NodeHandle nh_;

	TransformListener tfl_;

	ScanMask mask_;

	int mask_count_;

	CvRTrees forest;

	float connected_thresh_;

	int feat_count_;

	char save_[100];

	list<SavedFeature*> saved_features_;
	boost::mutex saved_mutex_;

	int feature_id_;
        
        // topics
	ros::Publisher leg_cloud_pub_ , leg_detections_pub_ , human_distance_pub_ ;  //ROS topic publishers

	ros::Publisher tracker_measurements_pub_;

	message_filters::Subscriber<srs_msgs::PositionMeasurement> people_sub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	tf::MessageFilter<srs_msgs::PositionMeasurement> people_notifier_;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

        // services
        ros::ServiceServer service_server_detect_legs_;


        
	           



	LegDetector(ros::NodeHandle nh) :
		nh_(nh),
		mask_count_(0),
		connected_thresh_(0.06),
		feat_count_(0),
		people_sub_(nh_,"people_tracker_filter",10),
	//	//laser_sub_(nh_,"scan",10),
	        laser_sub_(nh_,scan_topic,10),
        //	laser_sub_(nh_,"scan_front",10),
		people_notifier_(people_sub_,tfl_,fixed_frame,10),
		laser_notifier_(laser_sub_,tfl_,fixed_frame,10),
                client ("srs_decision_making_actions",true)
                
	{
		
                if (g_argc > 1) {
			forest.load(g_argv[1]);
			feat_count_ = forest.get_active_var_mask()->cols;
			printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
		} else {
			printf("Please provide a trained random forests classifier as an input.\n");
			shutdown();
		}

		// advertise topics
		//    leg_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("kalman_filt_cloud",10);
		leg_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("tracked_people",10);
                leg_detections_pub_ = nh_.advertise<sensor_msgs::PointCloud>("leg_detections_cloud",10);
		tracker_measurements_pub_ = nh_.advertise<srs_msgs::PositionMeasurement>("people_tracker_measurements",1);
                human_distance_pub_= nh_.advertise<srs_msgs::HS_distance>("HS_distance",10);                
 
		//		people_notifier_.registerCallback(boost::bind(&LegDetector::peopleCallback, this, _1));
		people_notifier_.setTolerance(ros::Duration(0.01));
		laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1));
		laser_notifier_.setTolerance(ros::Duration(0.01));
                 
                //services
                service_server_detect_legs_ = nh_.advertiseService("detect_legs", &LegDetector::detectLegsCallback, this);

		feature_id_ = 0;
                pauseSent = false;
                counter = 1;
                
	        client_map = nh_.serviceClient<nav_msgs::GetMap>("/static_map"); // geting the clent for the map ready
                       
                if (client_map.call(srv_map)) {  // call to srv_map OK
                  printf ("The cells in the ocupancy grid with size %i are:\n", srv_map.response.map.data.size());
                  printf (" width %i are:\n", srv_map.response.map.info.width);
                  printf (" height %i are:\n", srv_map.response.map.info.height);
                  printf (" resolution %f is:\n", srv_map.response.map.info.resolution);
                  geometry_msgs::Pose pose = srv_map.response.map.info.origin;
                  printf (" x position is %f \n", pose.position.x);
                  printf (" y position is %f \n", pose.position.y);
                  printf (" z position is %f \n", pose.position.z);
                  printf (" x orientation is %f \n", pose.orientation.x);
                  printf (" y orientation is %f \n", pose.orientation.y);
                  printf (" z orientation is %f \n", pose.orientation.z);
                  printf (" w orientation is %f \n", pose.orientation.w);

                   
              
                  // printing the map
                  for (int i = 0 ; i<320 ; i++) {
                    for (int k = 0; k<320 ; k++) {
                    if   (srv_map.response.map.data [i*320+k]<0) printf (".");
                    else if (srv_map.response.map.data [i*320+k]>=0 && srv_map.response.map.data [i*320+k]<100) printf ("#");
                    else if (srv_map.response.map.data [i*320+k]=100) printf ("@");
                   }
                   printf ("\n");
                  }

                 

                  

                  
                   

             //     for (int i=1; i<srv_map.response.map.data.size(); i++) {
             //       if   (srv_map.response.map.data [i]>0)
             //       printf("ocupancy grid %i is %i \n",i, srv_map.response.map.data [i]);    
             //     }
                                    

  
                } 
                else
	        {
		   ROS_ERROR("Failed to call service GetMap");
	        }
       }


	~LegDetector()
	{
	}


// actionlib Pause Call
   bool sendActionLibGoalPause()
        {
         if (pauseSent) // Pause Action has been  sent already
                return true;

         
              

       //      Client client ("srs_decision_making_actions",true);


                if (!client.waitForServer(ros::Duration(10))) // ros::Duration(5)
                                   
 
                   {
                      printf(" Unable to establish connection with ActionServer in DM !!! Sending goals to DM when person is detected is disabled\n");
                      return false;
                   }   
   

                goal.action="pause";
                goal.parameter="";
                goal.priority=counter++;
                client.sendGoal(goal);
                pauseSent = true;
           //wait for the action to return
                bool finished_before_timeout = client.waitForResult(ros::Duration(2));

               if (finished_before_timeout)
                {
                 actionlib::SimpleClientGoalState state = client.getState();
                 ROS_INFO("Action call to DM finished with state: %s",state.toString().c_str()); 
                return true;
                }
              else
                ROS_INFO("Action call to DM did not finish before timeout");
              
            return false;

       }




// alerts when the distance is bigger that a specified treshold              
void measure_distance (double dist) {  
                      printf ("Distance %f \n" , dist);  // the distance to the detected human
                       
                       distance_msg.distance = dist*100;
                       human_distance_pub_.publish(distance_msg);
                      
                       if ( !pauseSent && dist < det_dist__for_pause )
                          {
                            printf ("Local user too close ! Sending Pause ActionLibGoal to the server. Waiting for Action server responce \n"); 
                            sendActionLibGoalPause();
                           } 
                        else if ( dist > det_dist__for_resume && pauseSent ) {
                            printf ("Local user is far away now ! Sending Resume ActionLibGoal to the server. Waiting for Action server responce \n"); 
                            sendActionLibGoalResume();

                           }

}

// actionlib Resume Call
 bool sendActionLibGoalResume()
        {
         if (!pauseSent ) // Resume Action has been  sent already
                return true;


            
              

   //          Client client ("srs_decision_making_actions",true);


                if (!client.waitForServer())  // ros::Duration(5)
                                   
 
                   {
                      printf(" Unable to establish connection with ActionServer in DM !!! Sending goals to DM when person is detected is disabled\n");
                      return false;
                   }   
   

                goal.action="resume";
                goal.parameter="";
                goal.priority=counter++;
                client.sendGoal(goal);
                pauseSent = false;
           //wait for the action to return
                bool finished_before_timeout = client.waitForResult(ros::Duration(2));

               if (finished_before_timeout)
                {
                 actionlib::SimpleClientGoalState state = client.getState();
                 ROS_INFO("Action call to DM finished with state: %s",state.toString().c_str()); 
                return true;
                }
              else
                ROS_INFO("Action call to DM did not finish before timeout");
              
            return false;

       }



      // calback for the DetectLegs service        
        bool detectLegsCallback(srs_leg_detector::DetectLegs::Request &req, srs_leg_detector::DetectLegs::Response &res)
        {
         //vector<geometry_msgs::Point32> pts(5);
         geometry_msgs::Point32 pt1;
         geometry_msgs::Point32 pt2;
         geometry_msgs::Point32 pt3;
         geometry_msgs::Point32 pt4;
         geometry_msgs::Point32 pt5;

         pt1.x=1.0; pt1.y=1.0;
         pt2.x=-1.0; pt2.y=1.0;
         pt3.x=0.5; pt3.y=-1.0;
         pt4.x=-1.0; pt4.y=-1.0;
         pt5.x=1.0; pt5.y=3.0;
         

         res.leg_list.header.stamp = ros::Time::now();

         //res.leg_list.points.insert (pts);
         res.leg_list.points.push_back(pt1);
         res.leg_list.points.push_back(pt2);
         res.leg_list.points.push_back(pt3);
         res.leg_list.points.push_back(pt4);
         res.leg_list.points.push_back(pt5);

         return true;
        }



	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
                float map_value;
		ScanProcessor processor(*scan, mask_);

		processor.splitConnected(connected_thresh_);
		processor.removeLessThan(5);

		CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

		// if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
		ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);
		list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
		while (sf_iter != saved_features_.end())
		{
			if ((*sf_iter)->meas_time_ < purge)
			{
				delete (*sf_iter);
				saved_features_.erase(sf_iter++);
			}
			else
				++sf_iter;
		}


		// System update of trackers, and copy updated ones in propagate list
		list<SavedFeature*> propagated;
		for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
				sf_iter != saved_features_.end();
				sf_iter++)
		{
			(*sf_iter)->propagate(scan->header.stamp);
			propagated.push_back(*sf_iter);
		}

		// Detection step: build up the set of leg candidates clusters
		list<SampleSet*> leg_candidates;
		for (list<SampleSet*>::iterator i = processor.getClusters().begin();
				i != processor.getClusters().end();
				i++)
		{
			vector<float> f = calcLegFeatures(*i, *scan);

			for (int k = 0; k < feat_count_; k++)
				tmp_mat->data.fl[k] = (float)(f[k]);

			if (forest.predict( tmp_mat ) > 0)
			{
				leg_candidates.push_back(*i); // adds a new element
			}
		}
		// build list of positions
		list<Point> positions;
		for (list<SampleSet*>::iterator i = leg_candidates.begin();
				i != leg_candidates.end();
				i++)
		{
			Point pos=(*i)->center();
			positions.push_back(pos);
             
                        measure_distance ((*i)->center().distance(Point(0,0,0)));  // measures the distance from the robot to the detected human and pause if needed
                  

                  

                }

		// Build up the set of pair of closest positions
		list<Pair*> candidates;


		while (positions.size()!=0){
			if (positions.size()==1){
				list<Point>::iterator it1=positions.begin();

				list<Pair*>::iterator new_c=candidates.insert(candidates.end(),new Pair((*it1),(*it1),float(0)));

				positions.erase(it1);

			}
			else{
				list<Point>::iterator it1=positions.begin();
				list<Point>::iterator it2=it1;
				++it2;

				float closest_dist;
				closest_dist=(*it1).distance(*it2);

				float dist;
				list<Point>::iterator it3;
				it3=it2;
				++it2;
				for (;it2!= positions.end();it2++){
					dist=(*it1).distance(*it2);

					if (dist<closest_dist){
						closest_dist=dist;
						it3=it2;
					}
				}

				list<Pair*>::iterator new_c=candidates.insert(candidates.end(),new Pair((*it1),(*it3),closest_dist));
				positions.erase(it1);
				positions.erase(it3);

			}

		}

		//Build the set of pair of legs and single legs
		list<Legs*> legs;

		for(list<Pair*>::iterator i = candidates.begin();i != candidates.end();i++){

			if((*i)->dist_sep_m==0){
				Stamped<Point> loc1((*i)->pos1_,scan->header.stamp, scan->header.frame_id);
				list<Legs*>::iterator new_pair=legs.insert(legs.end(),new Legs(loc1,"single"));

			} else {
				if ((*i)->dist_sep_m<=leg_pair_separation_m){
					Stamped<Point> loc((((*i)->pos1_).operator +=((*i)->pos2_)).operator /=(btScalar(2)),scan->header.stamp, scan->header.frame_id);
					list<Legs*>::iterator new_pair=legs.insert(legs.end(),new Legs(loc,"pair"));
				}
				else{
					Stamped<Point> loc1((*i)->pos1_,scan->header.stamp, scan->header.frame_id);
					Stamped<Point> loc2((*i)->pos2_,scan->header.stamp, scan->header.frame_id);
					list<Legs*>::iterator new_s=legs.insert(legs.end(),new Legs(loc1,"single"));
					new_s=legs.insert(legs.end(),new Legs(loc2,"single"));
				}
			}

		}


	// For each candidate, find the closest tracker (within threshold) and add to the match list
	// If no tracker is found, start a new one
	multiset<MatchedFeature> matches;

        vector<geometry_msgs::Point32> detections_visualize(legs.size()); // used to visuallise the leg candidates

        int i = 0;
	for (list<Legs*>::iterator cf_iter = legs.begin();cf_iter != legs.end(); cf_iter++,i++) {
		
               Stamped<Point> loc=(*cf_iter)->loc_;
		try {
			tfl_.transformPoint(fixed_frame, loc, loc);
		} catch(...) {
			ROS_WARN("TF exception spot 3.");
		}
                
               // int y111 = (*loc);
                

                


                ind_x = (loc[0] / srv_map.response.map.info.resolution + srv_map.response.map.info.width / 2);  //x index in the ocupancy map
                ind_y = (loc[1] / srv_map.response.map.info.resolution + srv_map.response.map.info.height / 2); //y index in the ocupancy map

                indtmp = ind_y*160+ind_x; //printing for checking
                printf ("map index: %i \n",indtmp );

                printf ( "map x: %f, map y: %f, index_x: %i, index_y:%i, comb index:%i ",loc[0], loc[1], ind_x , ind_y, ind_y*160+ind_x );
                map_value =  srv_map.response.map.data [ind_y*160+ind_x];
                printf ("map_value: %f \n",map_value); 
          
                if (map_value = 100)  { 
                  printf ("the point is on occupied cell of the map \n");
                 }

                if (map_value < 0)  {
                  printf ("the point is outside the map \n");
                 } 
                 
                if (map_value > 0 && map_value < 100)  {
                  printf ("GOOD - the point is inside the map \n");
                   if (pauseSent)

                                  detections_visualize[i].z=1.0;

                                else

                                  detections_visualize[i].z=0.0;


                 } else {
                  detections_visualize[i].z =-1.0;
                  }

                           
 
                
                detections_visualize[i].x =  loc[0];
                detections_visualize[i].y = loc[1];
                

                  


		list<SavedFeature*>::iterator closest = propagated.end();
		float closest_dist = max_track_jump_m;

		for (list<SavedFeature*>::iterator pf_iter = propagated.begin();
				pf_iter != propagated.end();
				pf_iter++)
		{
			// find the closest distance between candidate and trackers
			float dist = loc.distance((*pf_iter)->position_);
			if ( dist < closest_dist )
			{
				closest = pf_iter;
				closest_dist = dist;
			}
		}
		// Nothing close to it, start a new track
		if (closest == propagated.end())
		{
			list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
		}
		// Add the candidate, the tracker and the distance to a match list
		else
			matches.insert(MatchedFeature(*cf_iter,*closest,closest_dist));
	}




	// loop through _sorted_ matches list
	// find the match with the shortest distance for each tracker
	while (matches.size() > 0)
	{
		multiset<MatchedFeature>::iterator matched_iter = matches.begin();
		bool found = false;
		list<SavedFeature*>::iterator pf_iter = propagated.begin();
		while (pf_iter != propagated.end())
		{
			// update the tracker with this candidate
			if (matched_iter->closest_ == *pf_iter)
			{
				// Transform candidate to fixed frame
				Stamped<Point> loc(matched_iter->candidate_->loc_, scan->header.stamp, scan->header.frame_id);
				try {
					tfl_.transformPoint(fixed_frame, loc, loc);
				} catch(...) {
					ROS_WARN("TF exception spot 4.");
				}

				// Update the tracker with the candidate location
				matched_iter->closest_->update(loc);

				// remove this match and
				matches.erase(matched_iter);
				propagated.erase(pf_iter++);
				found = true;
				break;
			}
			// still looking for the tracker to update
			else
			{
				pf_iter++;
			}
		}

		// didn't find tracker to update, because it was deleted above
		// try to assign the candidate to another tracker
		if (!found)
		{
			Stamped<Point> loc(matched_iter->candidate_->loc_, scan->header.stamp, scan->header.frame_id);
			try {
				tfl_.transformPoint(fixed_frame, loc, loc);
			} catch(...) {
				ROS_WARN("TF exception spot 5.");
			}

			list<SavedFeature*>::iterator closest = propagated.end();
			float closest_dist = max_track_jump_m;

			for (list<SavedFeature*>::iterator remain_iter = propagated.begin();
					remain_iter != propagated.end();
					remain_iter++)
			{
				float dist = loc.distance((*remain_iter)->position_);
				if ( dist < closest_dist )
				{
					closest = remain_iter;
					closest_dist = dist;
				}
			}

			// no tracker is within a threshold of this candidate
			// so create a new tracker for this candidate
			if (closest == propagated.end())
				list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
			else
				matches.insert(MatchedFeature(matched_iter->candidate_,*closest,closest_dist));
			matches.erase(matched_iter);
		}
	}

			cvReleaseMat(&tmp_mat); tmp_mat = 0;

			vector<geometry_msgs::Point32> filter_visualize(saved_features_.size());
                        

			vector<float> weights(saved_features_.size());
			sensor_msgs::ChannelFloat32 channel;
			
                        i = 0;

			for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
					sf_iter != saved_features_.end();
					sf_iter++,i++)
			{
				// reliability
				StatePosVel est;
				(*sf_iter)->filter_.getEstimate(est);
				double reliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));

				// publish filter result
                                
				filter_visualize[i].x = est.pos_[0];
				filter_visualize[i].y = est.pos_[1];
				//	filter_visualize[i].z = est.pos_[2];
				if (pauseSent)
                                  filter_visualize[i].z=1.0;
                                else
                                  filter_visualize[i].z=0.0;

				weights[i] = *(float*)&(rgb[min(998, max(1, (int)trunc( reliability*999.0 )))]);

				srs_msgs::PositionMeasurement pos;
				pos.header.stamp = (*sf_iter)->time_;
				pos.header.frame_id = fixed_frame;
				pos.name = "leg_detector";
				pos.object_id = (*sf_iter)->object_id;
				pos.pos.x = est.pos_[0];
				pos.pos.y = est.pos_[1];
				pos.pos.z = est.pos_[2];
				pos.covariance[0] = pow(0.3 / reliability,2.0);
				pos.covariance[1] = 0.0;
				pos.covariance[2] = 0.0;
				pos.covariance[3] = 0.0;
				pos.covariance[4] = pow(0.3 / reliability,2.0);
				pos.covariance[5] = 0.0;
				pos.covariance[6] = 0.0;
				pos.covariance[7] = 0.0;
				pos.covariance[8] = 10000.0;
				pos.initialization = 0;

				// If I've already seen this leg, publish its position.
				if ((*sf_iter)->object_id != "")
					tracker_measurements_pub_.publish(pos);
			}

			// visualize all trackers
			
                        channel.name = "laser";
                        channel.values = weights;
			sensor_msgs::PointCloud  people_cloud;  //from the filter
                        sensor_msgs::PointCloud  detections_cloud;  //directly from detections

			people_cloud.channels.push_back(channel);
			people_cloud.header.frame_id = fixed_frame;//scan_.header.frame_id;
			people_cloud.header.stamp = scan->header.stamp;
			people_cloud.points  = filter_visualize;
			leg_cloud_pub_.publish(people_cloud);

                        detections_cloud.channels.push_back(channel);
			detections_cloud.header.frame_id = fixed_frame;//scan_.header.frame_id;
			detections_cloud.header.stamp = scan->header.stamp;
			detections_cloud.points  = detections_visualize;
			leg_detections_pub_.publish(detections_cloud);



}

};





int main(int argc, char **argv)
{
	ros::init(argc, argv,"laser_processor");
	g_argc = argc;
	g_argv = argv;
      
        if (g_argc > 2) {
			scan_topic = g_argv[2];
			
			printf("Listening on topic %s : \n", g_argv[2]);
		} else {
			printf("Please provide the input topic as a parameter,e.g. scan_front. Assuming scan_front ! \n");
			shutdown();
		}
       
      
       

        


	ros::NodeHandle nh;
	LegDetector ld(nh);
        

        ros::spin();
        

	return 0;
}

