/* 
 * File:   obj_point_types.h
 * Author: cja58
 * Adapted from a similar file by Abhishek Anand
 *
 * Created on Apr 25, 2012, 07:59 PM
 */

#ifndef OBJ_POINT_TYPES_H
#define	OBJ_POINT_TYPES_H
#include <pcl/point_types.h>

namespace pcl {

    struct PointXYGRGBCam {
        PCL_ADD_POINT4D;
        float rgb;
        PCL_ADD_NORMAL4D;
        uint32_t cameraIndex;
        float distance;
        float curvature;

    };
    
    struct PointPLY {
        PCL_ADD_POINT4D;
        uint32_t r;
        uint32_t g;
        uint32_t b;

    };

    struct PointXYInt {
        int x;
        int y;
        int z;
        int segment;
        int label;
    };

    struct PointXYZRGBCamSL {
        PCL_ADD_POINT4D;

        union {

            struct {
                float rgb;
            };
            float data_c[4];
        };
        uint32_t cameraIndex;
        float distance;
        uint32_t segment;
        uint32_t label;

        //      inline PointXYZRGBCamSL (float _x, float _y, float _z) { x = _x; y = _y; z = _z; data[3] = 1.0f; }

        void clone(const struct PointXYZRGB & rhs) {
            for (int i = 0; i < 4; i++)
                data[i] = rhs.data[i];
            rgb = rhs.rgb;
            segment = 0;
            label = 0;
            cameraIndex = 0;
            distance = 0.0;

        }

        void clone(const struct PointXYZ & rhs) {
          for (int i = 0; i < 4; i++)
            data[i] = rhs.data[i];
          rgb = 0;
          segment = 0;
          label = 0;
          cameraIndex = 0;
          distance = 0.0;
        }
        
        void clone(const struct PointXYZRGBCamSL & rhs) {
            *this=rhs;
        }
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYGRGBCam,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        )

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYInt,
        (int, x, x)
        (int, y, y)
        (int, z, z)
        (int, segment, segment)
        (int, label, label)
        )
        
POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointPLY,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (int, r, r)
        (int, g, g)
        (int, b, b)
        )
        

POINT_CLOUD_REGISTER_POINT_STRUCT(
        pcl::PointXYZRGBCamSL,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        (uint32_t, segment, segment)
        (uint32_t, label, label)
        )

//PCL_INSTANTIATE_initTree(pcl::PointXYZRGBCamSL)

#endif	/* OBJ_POINT_TYPES_H */

