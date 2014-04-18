//http://www.pointclouds.org/documentation/tutorials/extract_indices.php

#include "common.h"
#include "typesdef.h"
#include "process.h"
#include "other.hpp"
#include "meanshift.h"
#include "gaussfilter.h"

string g_str;

int main (int argc, char** argv)
{
	//从硬盘读取pcd格式的点云文件，点云数据含有RGB信息
	//pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	io::loadPCDFile(argv[1], *cloud);
	
	get_output_dir(argv[1], g_str);

	// 下采样滤波，在保持点云数据形状不变的前提下，减少点云数据，其中叶子尺寸值越大，点云数据量越少
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
#pragma region R_VOXELGRID
	//pcl::VoxelGrid<pcl::PointXYZ> vg;
	//vg.setInputCloud (cloud);
	//vg.setLeafSize (0.35f, 0.35f, 0.35f);
	//vg.filter (*cloud_filtered);
	//std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 
#pragma endregion

#pragma region R_GAUSSFILTER
	//GaussFilter gf;
	//gf.setInput(cloud);
	//gf.setRadius(0.8);
	//gf.setSigma(0.4);
	//gf.filter(cloud_filtered);
	//io::savePCDFileASCII("gauss_filtered", *cloud_filtered);
#pragma endregion
	*cloud_filtered = *cloud;

	VecPointIndices v_pnt_indices;//存储每个点云平面所在的下标索引
	VecPlaneCoeff v_coeff;//存储各个平面的系数
	std::vector<VecPoint> v_cloud_plane;//存储各个平面的点云
	segment_plane(cloud_filtered, v_pnt_indices, v_coeff, v_cloud_plane, false);

	VecPointCloudPlane v_pntcldplane;
	std::vector<VecPointCloudPlane> v_vec_pntcldplane_clustered;
	build_vec_pointcloudplane(v_cloud_plane, v_coeff, v_pntcldplane);
	cluster_parallel_plane(v_pntcldplane, v_vec_pntcldplane_clustered);

	VecPointCloudPlane v_merged_pntcldplane;
	traverse_merge_planes(v_vec_pntcldplane_clustered, v_merged_pntcldplane);

	int index = 0;
	for (size_t i = 0, cnt = v_merged_pntcldplane.size(); i < cnt; ++i)
	{
		PointCloud<PointXYZ>::Ptr cld(new PointCloud<PointXYZ>());

		pointcloudplane2pointcloud(v_merged_pntcldplane[i], cld);

		std::stringstream ss;
		ss << "merge_" << index << ".pcd";
		index++;

		string str = g_str;
		str += "/";
		str += string(ss.str());
		io::savePCDFileASCII(str, *cld);
	}
	
	VecRect v_rect;
	traverse_determin_planes_verticles(cloud_filtered, v_merged_pntcldplane, v_rect);

	PointCloud<PointXYZ>::Ptr cld_rebuilded(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr cld_contour_rebuilded(new PointCloud<PointXYZ>());
	for (size_t i = 0; i < v_rect.size(); ++i)
	{
		determin_plane_from_rect(v_rect[i], cld_rebuilded);
		determin_plane_from_rect_only_contour(v_rect[i], cld_contour_rebuilded);
	}

	{
		string str, str2;
		str = g_str;

		str2 = (str += "/");
		str += "cloud_rebuiled.pcd";

		io::savePCDFileASCII(str, *cld_rebuilded);
		str2 += "cloud_contour_rebuiled.pcd";
		io::savePCDFileASCII(str2, *cld_contour_rebuilded);
	}

#if defined EUCLIDEAN_CLUSTER_EXTRACTION

	// 创建近临搜索对象，聚类分割对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;//类型是向量，存储分割后的各个聚类
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;//聚类分割对象
	ec.setClusterTolerance (0.1); // 2cm设置ClusterTolerance，该值为近临搜索半径
	ec.setMinClusterSize (500);//聚类中点的数量下限
	ec.setMaxClusterSize (25000);//聚类中点的数量上限
	ec.setSearchMethod (tree);//搜索方法，radiusSearch
	ec.setInputCloud ( cloud_filtered);
	ec.extract (cluster_indices);//提取搜索半径内的近临索引即为聚类分割出的聚类

	int j = 0;

	//显示各个分割后的聚类
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)//迭代查询聚类向量cluster_indices中所存储的聚类
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)//将cluster_indices中第pit个聚类的点云数据写入cloud_cluster
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //将聚类点云文件写入当前目录下
		 j++;
		pcl::visualization::PCLVisualizer viewer("Cloud Viewer"); //窗口显示第pit个聚类
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> color(cloud_cluster); //显示点云RGB信息
		viewer.addPointCloud(cloud_cluster,"Cloud Viewer", 0);
		viewer.setBackgroundColor(1,1,1);

		while (!viewer.wasStopped ())
		{
		//?ú?????éò?ìí?ó??????àí
		 viewer.spinOnce();
		}
	}
#endif //EUCLIDEAN_CLUSTER_EXTRACTION

	system("pause");
	return (0);
}
