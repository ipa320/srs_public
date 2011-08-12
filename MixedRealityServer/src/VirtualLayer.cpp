#include "MixedRealityServer/VirtualLayer.h"

#include <ros/ros.h>

namespace MixedRealityServer
{
    VirtualLayer::VirtualLayer(CvSize img_size)
    {
        this->imageSize = img_size;
        this->objectMap = cvCreateImage(this->imageSize,IPL_DEPTH_16U,1);
    }

    VirtualLayer::~VirtualLayer()
    {
        Objects.clear();
        cvReleaseImage(&objectMap);
    }

    void VirtualLayer::AddObject(Contour2D* obj)
    {
        Objects.push_back(*obj);
    }

    void VirtualLayer::RemoveObject(int obj_id)
    {
        int sz = Objects.size();
        for(int i = 0; i < sz; i++)
        {
            if(Objects[i].GetID() == obj_id)
            {
                Objects.erase(Objects.begin()+i);
            }
        }
    }

    void VirtualLayer::DrawLayer(IplImage* img)
    {
        int sz = Objects.size();
        for(int i = 0; i < sz; i++)
        {
            switch(Objects[i].GetContourType())
            {
                case CONTOUR_RECTANGLE:
                cvRectangle(img, Objects[i].GetDrawPoint1(), Objects[i].GetDrawPoint2(), Objects[i].GetColor(), Objects[i].GetContourWidth());
                break;
            }
        }
    }

    void VirtualLayer::HitTest(int x, int y)
    {
        int obj_id = objectMap->imageData[y * objectMap -> widthStep + x];

        int sz = Objects.size();
        for(int i = 0; i < sz; i++)
        {
            switch(Objects[i].GetContourType())
            {
                case CONTOUR_RECTANGLE:
                    if(x >= Objects[i].GetDrawPoint1().x && x <= Objects[i].GetDrawPoint2().x &&
                       y >= Objects[i].GetDrawPoint1().y && y <= Objects[i].GetDrawPoint2().y)
                    {
                       ROS_INFO("Click: %d %d ; Hit: %d", x, y, Objects[i].GetID());
                       Objects[i].Select();
                    }
                    else
                    {
                       Objects[i].Deselect();
                    }


                break;
            }
        }
    }

}
