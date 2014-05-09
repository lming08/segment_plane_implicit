/*
 *	lming_08@hotmail.com
 */

#ifndef _WINS_H_
#define _WINS_H_

#include <Eigen/Dense>
#include <pcl/features/boundary.h>
#include "common.h"
#include "typesdef.h"

class Wins
{
public:
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudPointXYZ;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<pcl::Boundary> PointCloudBoundary;

	Wins() : m_knn_radius(0){	}

	inline void setInputCloud(const PointCloudPtr pntcld)
	{
		m_input = pntcld;
	}

	/*设置KNN的半径，主要用于判断点是否是边界点*/
	inline void setKNNRadius(const float radius)
	{
		m_knn_radius = radius;
	}

	inline void setInputPlane(const PlaneCoeff &plncoeff)
	{
		m_plane = plncoeff;
	}

	void computeWins(VecRect &v_rect);

private:
	/*检查所有点是否是边界点*/
	void checkAllPointsIsBoundary(PointCloudBoundary &boundaries);

	/*计算平面点云法向量*/
	void computPointNormals(pcl::Normal &normal);

	/*获取边界点*/
	void getBoundaryPoints(const PointCloudBoundary &boundaries, PointCloudPtr pntcld);

	/*将点聚类，使得一个窗户成一类*/
	void clusterPoints(const PointCloudPtr pntcld, VecVecPoint &vv_pnt);

private:
	PointCloudPtr m_input;
	float m_knn_radius;
	PlaneCoeff m_plane;
};

#endif  //  _WINS_H_
