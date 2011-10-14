#ifndef CONTOUR2D_H
#define CONTOUR2D_H

#include <opencv2/highgui/highgui.hpp>

namespace MixedRealityServer
{
    typedef enum
    {
        CONTOUR_UNKNOWN = 0,
        CONTOUR_RECTANGLE = 1,
        CONTOUR_ELIPSE = 2,
    } ContourType;

    class Contour2D
    {
        private:
            unsigned int id;
            bool selected;
            ContourType type;
			std::string label;
            CvScalar color;
            int contour_width;
            CvPoint position;
            CvSize size;
			float angle;
 
        public:

            Contour2D(int id, ContourType t, std::string lbl, CvPoint pos, CvSize sz, float ang, CvScalar clr = cvScalar(255, 0, 0), int line_width=1);

            void Select();
            void Deselect();

            int GetID();
            bool IsSelected();
            ContourType GetContourType();	
			std::string GetLabel();
            CvScalar GetColor();
            int GetContourWidth();
			CvPoint GetPosition();
			CvSize GetSize();
			float GetAngle();
    };
}

#endif
