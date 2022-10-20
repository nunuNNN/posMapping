#include "navbox_lio.h"
#include <common_lib.h>

#define LASER_POINT_COV     (0.001)
#define MOV_THRESHOLD       (1.5)

NavBoxLio* NavBoxLio::pThis = NULL;

NavBoxLio::NavBoxLio()
{
   pThis=this;

   laserCloudOri.reset(new PointCloudXYZI(100000, 1));
   normvec.reset(new PointCloudXYZI(100000, 1));
   corr_normvect.reset(new PointCloudXYZI(100000, 1));

   feats_down_body.reset(new PointCloudXYZI());
   feats_down_world.reset(new PointCloudXYZI());

   memset(point_selected_surf, true, sizeof(point_selected_surf));
   memset(res_last, -1000.0f, sizeof(res_last));


   DET_RANGE = 100.0f;
   cube_len = 1000;
   filter_size_map_min = 0.5;

   extrinsic_est_en = false;

   feats_down_size = 0;
   kdtree_delete_counter = 0;
   Localmap_Initialized = false;

    filter_size_surf_min = 0.5;
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
}

NavBoxLio::~NavBoxLio() {}

void NavBoxLio::set_filter_size(const double &mapMin, const double &surfMin)
{
    filter_size_map_min = mapMin;
    filter_size_surf_min = surfMin;
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
}

void NavBoxLio::set_localMap_param(const float &detRange, const double &cubeLen)
{
    DET_RANGE = detRange;
    cube_len = cubeLen;
}

void NavBoxLio::set_calibr_ext(const bool &extCalibr)
{
    extrinsic_est_en = extCalibr;
}

void NavBoxLio::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    pThis->laserCloudOri->clear(); 
    pThis->corr_normvect->clear(); 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < pThis->feats_down_size; i++)
    {
        PointType &point_body  = pThis->feats_down_body->points[i]; 
        PointType &point_world = pThis->feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = pThis->Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            pThis->ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            pThis->point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!pThis->point_selected_surf[i]) continue;

        VF(4) pabcd;
        pThis->point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                pThis->point_selected_surf[i] = true;
                pThis->normvec->points[i].x = pabcd(0);
                pThis->normvec->points[i].y = pabcd(1);
                pThis->normvec->points[i].z = pabcd(2);
                pThis->normvec->points[i].intensity = pd2;
                pThis->res_last[i] = abs(pd2);
            }
        }
    }
    
    int effct_feat_num = 0;

    for (int i = 0; i < pThis->feats_down_size; i++)
    {
        if (pThis->point_selected_surf[i])
        {
            pThis->laserCloudOri->points[effct_feat_num] = pThis->feats_down_body->points[i];
            pThis->corr_normvect->points[effct_feat_num] = pThis->normvec->points[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        printf("No Effective Points! \r\n");
        return;
    }
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = pThis->laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = pThis->corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (pThis->extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

void NavBoxLio::points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}

void NavBoxLio::lasermap_fov_segment(const V3D &pos_LiD)
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;

    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

void NavBoxLio::pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void NavBoxLio::map_incremental()
{
    int add_point_size = 0;
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty())
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

void NavBoxLio::Process(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, 
                        PointCloudXYZI::Ptr &featsUndistort,
                        PointCloudXYZI::Ptr &featsDownBody)
{
    /*** Get predicted estimated position ***/
    state_point = kf_state.get_x();
    V3D pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

   /*** Segment the map in lidar FOV ***/
   lasermap_fov_segment(pos_lid);

   /*** downsample the feature points in a scan ***/
   downSizeFilterSurf.setInputCloud(featsUndistort);
   downSizeFilterSurf.filter(*feats_down_body);
   feats_down_size = feats_down_body->points.size();
   featsDownBody = feats_down_body;
   /*** initialize the map kdtree ***/
   if(ikdtree.Root_Node == nullptr)
   {
         if(feats_down_size > 5)
         {
            ikdtree.set_downsample_param(filter_size_map_min);
            feats_down_world->resize(feats_down_size);
            for(int i = 0; i < feats_down_size; i++)
            {
               pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            }
            ikdtree.Build(feats_down_world->points);
         }
         return;
   }

   /*** ICP and iterated Kalman filter update ***/
   if (feats_down_size < 5)
   {
         printf("No point, skip this scan!\r\n");
         return;
   }

   feats_down_world->resize(feats_down_size);
   Nearest_Points.resize(feats_down_size);
   normvec->resize(feats_down_size);

   /*** iterated state estimation ***/
   double solve_H_time = 0;
   kf_state.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
   state_point = kf_state.get_x();

   /*** add the feature points to map kdtree ***/
   map_incremental();   
}

