#ifndef CONTOUR2D_H
#define CONTOUR2D_H

#include <opencv2/highgui/highgui.hpp>

namespace MixedRealityServer
{
    typedef enum
    {
        CONTOUR_UNKNOWN,
        CONTOUR_RECTANGLE,
        CONTOUR_CIRCLE,
        CONTOUR_ELIPSE,
        CONTOUR_POLY
    } ContourType;

    class Contour2D
    {
        private:
            static int ID_Generator;

            unsigned int id;
            bool selected;
            ContourType type;
            CvScalar color;
            int contour_width;
            CvPoint position;
            CvSize size;
            //Rectangle Data
            CvPoint drawPoint1;
            CvPoint drawPoint2;

        public:

            Contour2D(ContourType t, CvScalar clr=cvScalar(255, 0, 0), int line_width=1);
            void SetAsRectangle(CvPoint p, CvSize s);

            void Select();
            void Deselect();

            int GetID();
            ContourType GetContourType();
            CvScalar GetColor();
            int GetContourWidth();
            bool IsSelected();
            CvPoint GetDrawPoint1();
            CvPoint GetDrawPoint2();

    };
}

#endif
