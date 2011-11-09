#include "MixedRealityServer/Contour2D.h"

#include <opencv2/highgui/highgui.hpp>


namespace MixedRealityServer
{
	Contour2D::Contour2D(int id, ContourType t, std::string lbl, CvPoint pos, CvSize sz, float ang, CvScalar clr, int line_width)
    {
        this->id = id;
        this->type = t;
		this->label = lbl;
        this->selected = false;
        this->color = clr;
        this->contour_width = line_width;
		this->position = pos;
		this->size = sz;
		this->angle = ang;
    }

    void Contour2D::Select()
    {
        color=cvScalar(0, 0, 255);
        contour_width = 4;
    }

    void Contour2D::Deselect()
    {
        color=cvScalar(255, 0, 0);
        contour_width = 1;
    }

    int Contour2D::GetID()
    {
        return id;
    }

    ContourType Contour2D::GetContourType()
    {
        return this->type;
    }
	std::string Contour2D::GetLabel()
	{
		return label;
	}
    CvScalar Contour2D::GetColor()
    {
        return this->color;
    }
    int Contour2D::GetContourWidth()
    {
        return this->contour_width;
    }

    bool Contour2D::IsSelected()
    {
        return selected;
    }
	CvPoint Contour2D::GetPosition()
	{
		return position;
	}	
	CvSize Contour2D::GetSize()
	{
		return size;
	}
	float Contour2D::GetAngle()
	{
		return angle;
	}

}
