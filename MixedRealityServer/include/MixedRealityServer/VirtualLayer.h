#ifndef VIRTUAL_LAYER_H
#define VIRTUAL_LAYER_H

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "MixedRealityServer/Contour2D.h"



namespace MixedRealityServer
{
    const CvSize VIRTUAL_BUFFER_SIZE = cvSize(4000, 4000);


    class VirtualLayer
    {
        public:
            std::vector<Contour2D> Objects;

            VirtualLayer(CvSize img_size = VIRTUAL_BUFFER_SIZE);
            ~VirtualLayer();

            void AddObject(Contour2D* obj);
            void RemoveObject(int obj_id);
            void RemoveAllObjects();
            void DrawLayer(IplImage* img);
            void HitTest(int x, int y);
			int GetObjectsCount();
        private:
            CvSize imageSize;
            IplImage* objectMap;
    };
}

#endif
