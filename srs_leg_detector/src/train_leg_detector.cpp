/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "laser_processor.h"
#include "calc_leg_features.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

//#include "rosrecord/Player.h"
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "srs_msgs/PositionMeasurement.h"
#include "sensor_msgs/LaserScan.h"



using namespace std;
using namespace laser_processor;
using namespace ros;

enum LoadType {LOADING_NONE, LOADING_POS, LOADING_NEG, LOADING_TEST};

class TrainLegDetector
{
public:
  ScanMask mask_;
  int mask_count_; // number of added scans to the mask

  vector< vector<float> > pos_data_;
  vector< vector<float> > neg_data_;
  vector< vector<float> > test_data_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  TrainLegDetector() : mask_count_(0), connected_thresh_(0.06), feat_count_(0)
  {
  }

  void loadData(LoadType load, char* file)
  {
   printf("In loadData ...\n");
    if (load != LOADING_NONE)
    {
      switch (load)
      {
      case LOADING_POS:
        printf("Loading positive training data from file: %s\n",file); break;
      case LOADING_NEG:
        printf("Loading negative training data from file: %s\n",file); break;
      case LOADING_TEST:
        printf("Loading test data from file: %s\n",file); break;
      default:
        break;
      }


  rosbag::Bag bag;


  bag.open(file, rosbag::bagmode::Read);
  
    std::vector<std::string> topics;
    topics.push_back(std::string("/scan_front"));
   // topics.push_back(std::string("/scan_rear"));


   rosbag::View view(bag, rosbag::TopicQuery(topics));
   printf("scan size %i:", view.size());
      

   BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
 
           sensor_msgs::LaserScan::ConstPtr scanptr = m.instantiate<sensor_msgs::LaserScan>();
           
           if (scanptr != NULL)
           

   switch (load)
      {
      case LOADING_POS:
        loadCb ( &(sensor_msgs::LaserScan(*scanptr)),&pos_data_);
        break;
      case LOADING_NEG:
        mask_count_ = 1000; // effectively disable masking
        loadCb ( &(sensor_msgs::LaserScan(*scanptr)),&neg_data_);
        break;
      case LOADING_TEST:
       loadCb ( &(sensor_msgs::LaserScan(*scanptr)),&test_data_);
        break;
      default:
        break;
      }

  //            printf("Laser data %i:",(*scanptr).ranges.size());
 
           
     }
   
       bag.close();



    }
  }


 
void loadCb(sensor_msgs::LaserScan* scan, vector< vector<float> >* data)
 {
    printf (".");

    if (mask_count_++ < 20)  // for the first 20 scans it only builds the mask (expands it)
    {
      mask_.addScan(*scan);
    }
    else  // and then does the processing
    {
      ScanProcessor processor(*scan,mask_);
      processor.splitConnected(connected_thresh_);
      processor.removeLessThan(5);
    
      for (list<SampleSet*>::iterator i = processor.getClusters().begin();
           i != processor.getClusters().end();
           i++)
        data->push_back( calcLegFeatures(*i, *scan));
    }
  }

  void train()
  {
printf("In train()....");
    int sample_size = pos_data_.size() + neg_data_.size();
    feat_count_ = pos_data_[0].size();

    CvMat* cv_data = cvCreateMat( sample_size, feat_count_, CV_32FC1); // sample data
    CvMat* cv_resp = cvCreateMat( sample_size, 1, CV_32S);  // responces

    // Put positive data in opencv format.
    int j = 0;
    for (vector< vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = 1;  // positive responce
      j++;
    }

    // Put negative data in opencv format.
    for (vector< vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step*j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];
      
      cv_resp->data.i[j] = -1; // negative responce
      j++;
    }

    CvMat* var_type = cvCreateMat( 1, feat_count_ + 1, CV_8U );
    cvSet( var_type, cvScalarAll(CV_VAR_ORDERED));
    cvSetReal1D( var_type, feat_count_, CV_VAR_CATEGORICAL );
    
    float priors[] = {1.0, 1.0};
    
    CvRTParams fparam(8,  // _max_depth: max_categories until pre-clustering
                     20,  // _min_sample_count: Don’t split a node if less
                      0,  // _regression_accuracy: One of the “stop splitting” criteria
                  false,  // _use_surrogates: Alternate splits for missing data
                     10,  // _max_categories: 
                 priors,  // priors 
                  false,  // _calc_var_importance
                      5,  // _nactive_vars
                     50,  // max_tree_count
                 0.001f,  // forest_accuracy
       CV_TERMCRIT_ITER   // termcrit_type
       );
    fparam.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1);
    
    forest.train( cv_data, // train_data
            CV_ROW_SAMPLE, // tflag --> each row is a data point consisting of a vector of features that make up the columns of the matrix
                  cv_resp, // responses  --> a floating-point vector of values to be predicted given the data features
                        0, // comp_idx
                        0, // sample_idx
                 var_type, // var_type
                        0, // missing_mask
                  fparam); // the structure from above


    cvReleaseMat(&cv_data);
    cvReleaseMat(&cv_resp);
    cvReleaseMat(&var_type);
  }

  void test()
  {
    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

    int pos_right = 0;
    int pos_total = 0;    
    for (vector< vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict( tmp_mat) > 0)
        pos_right++;
      pos_total++;
    }

    int neg_right = 0;
    int neg_total = 0;
    for (vector< vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict( tmp_mat ) < 0)
        neg_right++;
      neg_total++;
    }

    int test_right = 0;
    int test_total = 0;
    for (vector< vector<float> >::iterator i = test_data_.begin();
         i != test_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict( tmp_mat ) > 0)
        test_right++;
      test_total++;
    }

    printf(" Pos train set: %d/%d %g\n",pos_right, pos_total, (float)(pos_right)/pos_total);
    printf(" Neg train set: %d/%d %g\n",neg_right, neg_total, (float)(neg_right)/neg_total);
    printf(" Test set:      %d/%d %g\n",test_right, test_total, (float)(test_right)/test_total);

    cvReleaseMat(&tmp_mat);

  }

  void save(char* file)
  {
    forest.save(file);
  }
};

int main(int argc, char **argv)
{
  if (argc < 2) 
    {
     printf("Usage: train_leg_detector --train file1 --neg file2 --test file3 --save conf_file\n");
     exit (0);
    }

  TrainLegDetector tld;

  LoadType loading = LOADING_NONE;

  char save_file[100];
  save_file[0] = 0;

  printf("Loading data...\n");
  for (int i = 1; i < argc; i++)
  {
    if (!strcmp(argv[i],"--train"))
      loading = LOADING_POS;
    else if (!strcmp(argv[i],"--neg"))
      loading = LOADING_NEG;
    else if (!strcmp(argv[i],"--test"))
      loading = LOADING_TEST;
    else if (!strcmp(argv[i],"--save"))
    {
      if (++i < argc)
        strncpy(save_file,argv[i],100);
      continue;
    }
    else
      tld.loadData(loading, argv[i]);
  }

  printf("Training classifier...\n");
  tld.train();

  printf("Evlauating classifier...\n");
  tld.test();
  
  if (strlen(save_file) > 0)
  {
    printf("Saving classifier as: %s\n", save_file);
    tld.save(save_file);
  }
}
