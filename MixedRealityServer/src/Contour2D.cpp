#include "MixedRealityServer/Contour2D.h"

#include <opencv2/highgui/highgui.hpp>

namespace MixedRealityServer
{
    int Contour2D::ID_Generator = 1;

    Contour2D::Contour2D(ContourType t, CvScalar clr, int line_width)
    {
        this->id = ID_Generator++;
        this->type = t;
        this->selected = false;
        this->color = clr;
        this->contour_width = line_width;
    }

    void Contour2D::SetAsRectangle(CvPoint p, CvSize s)
    {
        if(this->type == CONTOUR_RECTANGLE)
        {
            this->position = p;
            this->size = s;
            this->drawPoint1 = cvPoint(p.x, p.y);
            this->drawPoint2 = cvPoint(p.x + s.width, p.y + s.height);
        }
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
    CvPoint Contour2D::GetDrawPoint1()
    {
        return this->drawPoint1;
    }

    CvPoint Contour2D::GetDrawPoint2()
    {
        return this->drawPoint2;
    }

}
