/*
 *	lming_08@hotmail.com
 */
#ifndef _WINDOW_MODEL_H_
#define _WINDOW_MODEL_H_

#include <Eigen/Dense>
#include <pcl/features/boundary.h>
#include "common.h"
#include "typesdef.h"

class WindowModel
{
public:
	static const float Z_DELTA;
	static const float SIGMA;
	typedef PointCloud<PointXYZ>::Ptr PointCloudPtr;

	WindowModel() : m_knn_radius(0.0f){}	

	void setInputCloud(const PointCloudPtr pntcld)
	{
		m_input = pntcld;
	}

	void setInputPlane(const PlaneCoeff &plncoeff)
	{
		m_plane = plncoeff;
	}

	/*设置KNN的半径，主要用于判断点是否是边界点*/
	void setKNNRadius(const float radius)
	{
		m_knn_radius = radius;
	}

    /*检查所有点是否是边界点*/
    void checkAllPointsIsBoundary();

	/*计算窗户宽度和距离边缘的距离*/
	void computeWinWidthAndMarginDist(float &width, float &margin_lr_dist, float &horizon_wins_dist);

	/*计算窗户高度和距离边缘的距离*/
	void computeWinHeightAndMarginDist(float &height, float &margin_ud_dist, float &vertical_wins_dist);

private:
	/*从点云中获取Z值最大和最小的索引*/
	void getZMaxMinIndices(size_t &max_indice, size_t &min_indice);

	/*从点云中获取平面两侧最大和最小的索引*/
	void getBilateralIndices(size_t & indice1, size_t & indice2);

	/*计算平面点云法向量*/
	void computPointNormals(Normal &normal);

	/*获取平面水平方向的向量*/
	void getHorizonVector(Eigen::Vector3f &vec_horizon);

	/*获取平面垂直方向的向量*/
	void getVerticalVector(Eigen::Vector3f &vec_vertical);

	/*获取点到直线的距离*/
	inline float getPntToLineDist(const PointXYZ &pnt, const PointXYZ &pntinline, const Eigen::Vector3f &vec_direction);

private:
	PointCloudPtr m_input;
	PlaneCoeff m_plane;
	float m_knn_radius;
    pcl::PointCloud<pcl::Boundary> m_boundaries;
};

typedef struct _WindowModelParams
{
    float width;  //!窗户的宽度
    float margin_lr_dist;  //!窗户在左右方向上离墙面边缘的距离
    float horizon_wins_dist;  //!水平方向上，窗户间的距离

    float height;  //!窗户的高度
    float margin_ud_dist;  //!窗户在上下方向上离墙面边缘的距离
    float vertical_wins_dist;  //!垂直方向上，窗户间的距离

    _WindowModelParams() : width(0.0f), margin_lr_dist(0.0f), horizon_wins_dist(0.0f), 
		height(0.0f), margin_ud_dist(0.0f), vertical_wins_dist(0.0f){  }

}WindowModelParams;

#endif
