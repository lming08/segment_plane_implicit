/*
 *	lming_08@hotmail.com
 */

#include "common.h"
#include "typesdef.h"
#include "wins.h"

extern string g_work_dir_path;
bool determin_plane_verticles(/*const*/ PointCloudPlane & pntcldpln, Rect &rect);

void Wins::computeWins(VecRect &v_rect, bool is_savefile)
{
	PointCloudBoundary boundaries;
	PointCloudPtr cld_bnd(new pcl::PointCloud<pcl::PointXYZ>);
	VecVecPoint vv_pnt;

	checkAllPointsIsBoundary(boundaries);	
	getBoundaryPoints(boundaries, cld_bnd);
	clusterPoints(cld_bnd, vv_pnt, is_savefile);

	size_t win_count = vv_pnt.size();
	VecPointCloudPlane v_pntcldpln;
	
	v_pntcldpln.resize(win_count);
	for (size_t i = 0; i < win_count; ++i)
	{
		v_pntcldpln[i].m_indice = i;
		v_pntcldpln[i].m_coeff = m_plane;
		v_pntcldpln[i].mv_pointcloud = vv_pnt[i];		
	}
	
	v_rect.resize(win_count);
	for (size_t i = 0; i < win_count; ++i)
	{
		determin_plane_verticles(v_pntcldpln[i], v_rect[i]);
	}
}

void Wins::checkAllPointsIsBoundary(PointCloudBoundary &boundaries)
{
	if ( -0.000001 < m_knn_radius && m_knn_radius < 0.000001)
	{
		PCL_ERROR("setKNNRadius() must be called!\n");
		throw "setKNNRadius() must be called";
	}

	size_t pntcld_count = m_input->points.size();
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 

	normals->resize(pntcld_count);
	for (size_t i = 0; i < pntcld_count; ++i)
	{
		computPointNormals(normals->points[i]);
	}	

	boundEst.setInputCloud(m_input); 
	boundEst.setInputNormals(normals); 
	boundEst.setRadiusSearch(m_knn_radius); 
	boundEst.setAngleThreshold(M_PI/2); 
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); 
	boundEst.compute(boundaries); 
}

void Wins::computPointNormals(pcl::Normal &normal)
{
	//comm_oper::computPointNormals(m_input, m_normals, 150);
	normal.normal_x = m_plane.a;
	normal.normal_y = m_plane.b;
	normal.normal_z = m_plane.c;
	normal.curvature = 0.0f;  //! 平面的曲率为0
}

void Wins::getBoundaryPoints(const PointCloudBoundary &boundaries, PointCloudPtr pntcld)
{
	size_t bndpnt_count = boundaries.points.size();
	for (size_t i = 0; i < bndpnt_count; ++i)
	{
		if (boundaries.points[i].boundary_point)
		{
			pntcld->points.push_back(m_input->points[i]);
		}
	}
	pntcld->width = static_cast<uint32_t>(pntcld->points.size());
	pntcld->height = 1;
}

void Wins::clusterPoints(const PointCloudPtr pntcld, VecVecPoint &vv_pnt,  bool is_savefile)
{
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (pntcld);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	ec.setClusterTolerance (0.60f);
	// 窗户含点的个数大概是(10, 220)
	ec.setMinClusterSize (10);
	ec.setMaxClusterSize (220);
	ec.setSearchMethod (tree);
	ec.setInputCloud (pntcld);
	ec.extract (cluster_indices);

	int j = 0;
	size_t wins_count = cluster_indices.size();
	vv_pnt.resize(wins_count);
	for (size_t i = 0; i < wins_count; ++i)
	{
		size_t pnt_count = cluster_indices[i].indices.size();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

		vv_pnt[i].resize(pnt_count);
		cloud_cluster->points.resize(pnt_count);
		for (size_t k = 0; k < pnt_count; ++k)
		{
			pcl::PointXYZ &pnt = pntcld->points[ cluster_indices[i].indices[k] ];
			cloud_cluster->points[k] = pnt;
			vv_pnt[i][k] = Point(pnt.x, pnt.y, pnt.z);
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		if (is_savefile)
		{
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			std::stringstream ss;
			ss << "wins_" << j << ".pcd";

			string str = g_work_dir_path;
			str += "/";
			str += string(ss.str());
			pcl::io::savePCDFileASCII(str, *cloud_cluster);

			j++;
		}
	}
}
