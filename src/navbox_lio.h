#pragma once
// C/C++ 
#include <omp.h>
#include <math.h>

#include <ikd-Tree/ikd_Tree.h>
#include "use-ikfom.h"
#include <common_lib.h>

// eigen&& pcl
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class NavBoxLio
{
public:
   NavBoxLio();
   ~NavBoxLio();

   static void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
   void Process(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, 
                PointCloudXYZI::Ptr &featsUndistort,
                PointCloudXYZI::Ptr &featsDownBody);

   void set_filter_size(const double &mapMin, const double &surfMin);
   void set_localMap_param(const float &detRange, const double &cubeLen);
   void set_calibr_ext(const bool &extCalibr);

   PointCloudXYZI::Ptr feats_down_body;

private:
   void pointBodyToWorld(PointType const * const pi, PointType * const po);
   void lasermap_fov_segment(const V3D &pos_LiD);
   void points_cache_collect();
   void map_incremental();
   
    float calc_dist(PointType p1, PointType p2){
        float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        return d;
    }


private:
   static NavBoxLio* pThis;

   BoxPointType LocalMap_Points;
   std::vector<BoxPointType> cub_needrm;

   KD_TREE<PointType> ikdtree;
   std::vector<PointVector>  Nearest_Points; 
   pcl::VoxelGrid<PointType> downSizeFilterSurf;

   PointCloudXYZI::Ptr laserCloudOri;
   PointCloudXYZI::Ptr normvec;
   PointCloudXYZI::Ptr corr_normvect;
   PointCloudXYZI::Ptr feats_down_world;

   bool   point_selected_surf[100000] = {0};
   float res_last[100000] = {0.0};

   float DET_RANGE;
   double cube_len;
   double filter_size_map_min;
   double filter_size_surf_min;
   bool extrinsic_est_en;

   int    feats_down_size;
   int    kdtree_delete_counter;
   bool extrinsic_est_enss;
   bool Localmap_Initialized;

   state_ikfom state_point;
};

