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
		int new_id = obj->GetID();
		bool exist = false;
		int sz = Objects.size();
        for(int i = 0; i < sz; i++)
        {
            if(Objects[i].GetID() == new_id)
            {
               	exist = true;
				break;
            }
        }
		if(!exist)
		{
        	Objects.push_back(*obj);
		}
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
    
    void VirtualLayer::RemoveAllObjects()
    {
		Objects.clear();
    }

    void VirtualLayer::DrawLayer(IplImage* img)
    {
        int sz = Objects.size();
        for(int i = 0; i < sz; i++)
        {	
            switch(Objects[i].GetContourType())
            {
                case CONTOUR_RECTANGLE:
				{
					CvPoint2D32f* pts = (CvPoint2D32f*) malloc(4 * sizeof(CvPoint));
					CvBox2D box;
					
					box.center.x = Objects[i].GetPosition().x;
					box.center.y = Objects[i].GetPosition().y;
					box.size.width = Objects[i].GetSize().width;
					box.size.height = Objects[i].GetSize().height;						
					box.angle = Objects[i].GetAngle();
					
					cvBoxPoints(box, pts);
					
					cvLine(img, cvPoint(pts[0].x, pts[0].y), cvPoint(pts[1].x, pts[1].y), Objects[i].GetColor(), 2);
					cvLine(img, cvPoint(pts[1].x, pts[1].y), cvPoint(pts[2].x, pts[2].y), Objects[i].GetColor(), 2);
					cvLine(img, cvPoint(pts[2].x, pts[2].y), cvPoint(pts[3].x, pts[3].y), Objects[i].GetColor(), 2);
					cvLine(img, cvPoint(pts[3].x, pts[3].y), cvPoint(pts[0].x, pts[0].y), Objects[i].GetColor(), 2);
					
                	break;
				}
				case CONTOUR_ELIPSE:
				{
					CvBox2D box;
					
					box.center.x = Objects[i].GetPosition().x;
					box.center.y = Objects[i].GetPosition().y;
					box.size.width = Objects[i].GetSize().width;
					box.size.height = Objects[i].GetSize().height;						
					box.angle = Objects[i].GetAngle();
					
					cvEllipseBox(img, box, Objects[i].GetColor(), 2);

					
				}
					break;
				case CONTOUR_UNKNOWN:
					break;
            }
			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.7, 0.0, 1);
			cvPutText (img, Objects[i].GetLabel().c_str(), Objects[i].GetPosition(), &font, Objects[i].GetColor());
        }
    }

    void VirtualLayer::HitTest(int x, int y)
    {
        
    }
	int VirtualLayer::GetObjectsCount()
	{
		return Objects.size();
	}

}
