#ifndef _PROCESS_H_
#define _PROCESS_H_

#include "common.h"
#include "typesdef.h"

typedef std::vector<PointCloudPlane>::size_type v_pcp_size_type;

typedef std::vector<pcl::PointIndices> VecPointIndices;

 /** \brief 将点云按墙面进行聚类
   * \param[in] pointcloud 输入点云
   * \param[out] vv_pnt 聚类后的点云
   */
bool cluster_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, VecVecPoint &vv_pnt);

/** \brief 	随机采样分割出平面
   * \param[in] pointcloud  待分割的点云 
   * \param[out] v_planecoeff  存储分割后的每个平面的系数
   * \param[out] v_vec_point  存储分割后的每个平面所包含的点
   * \param[out] is_savefile  是否保存分割后的点云文件
   */
bool segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, 
	VecPointIndices &v_pointindices, VecPlaneCoeff &v_planecoeff, std::vector<VecPoint> &v_vec_point, bool is_savefile);

/** \brief 	随机采样分割出平面
   * \param[in] pointcloud  待分割的点云 
   * \param[out] v_planecoeff  存储分割后的每个平面的系数
   * \param[out] v_vec_point  存储分割后的每个平面所包含的点
   * \param[out] is_savefile  是否保存分割后的点云文件
   */
bool segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, 
	VecPointIndices &v_pointindices, VecPlaneCoeff &v_planecoeff, std::vector<VecPoint> &v_vec_point, bool is_savefile, size_t pln_index);

 /** \brief  构建一个Vector，里面存储PointCloudPlane
   * \param[in]  v_planecoeff  平面方程系数
   * \param[in]  v_vec_point  平面所包含的点云
   * \param[out]  v_pcp  存储PointCloudPlane的容器
   */
bool build_vec_pointcloudplane(const std::vector<VecPoint> &v_vec_point, const VecPlaneCoeff &v_planecoeff, VecPointCloudPlane &v_pcp);

 /** \brief  将平行且间隔短的平面聚类到一起
   * \param[in]  v_pntcldpln_before 聚类之前的平面
   * \param[out]  v_vec_pntcldpln_after 聚类之后的平面族
   */
bool cluster_parallel_plane(const VecPointCloudPlane &v_pntcldpln_before, std::vector<VecPointCloudPlane> &v_vec_pntcldpln_after);

 /** \brief  遍历存储非平行的平面族，将各平行平面族中的平面归并
   * \param[in]  v_vec_pntcldpln 存储各平行平面族
   * \param[out]  v_pntcldpln 归并之后的非平行平面族
   */
bool traverse_merge_planes(std::vector<VecPointCloudPlane> & v_vec_pntcldpln, VecPointCloudPlane & v_pntcldpln);

 /** \brief  将PointCloudPlane转换为PointCloud
   * \param[in]  in_pcp 输入的PointCloudPlane
   * \param[out]  cld 输出的PointCloud
   */
bool pointcloudplane2pointcloud(const PointCloudPlane & in_pcp, PointCloud<PointXYZ>::Ptr cld);

 /** \brief  遍历确定各平面的顶点
 * \param[in]  pntcld_original 原始点云
   * \param[in]  v_pntcldpln PointCloudPlane型平面
   * \param[out]  v_rect 各平面所对应的矩形
   */
bool traverse_determin_planes_verticles(const PointCloud<PointXYZ>::Ptr pntcld_original, /*const*/ VecPointCloudPlane & v_pntcldpln, VecRect & v_rect);

 /** \brief  从矩形3个顶点确定一个点云平面
   * \param[in]  rect 输入的Rect，包括矩形的3个顶点
   * \param[out]  cld 得到的点云PointCloud
   */
bool determin_plane_from_rect(Rect & rect, PointCloud<PointXYZ>::Ptr cld);

 /** \brief  从矩形4个顶点确定一个点云平面，只画出轮廓
   * \param[in]  rect 输入的Rect，包括矩形的4个顶点
   * \param[out]  cld 得到的点云PointCloud
   */
bool determin_plane_from_rect_only_contour(Rect & rect, PointCloud<PointXYZ>::Ptr cld);

#endif