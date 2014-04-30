//http://www.pointclouds.org/documentation/tutorials/extract_indices.php

#include "common.h"
#include "typesdef.h"
#include "process.h"
#include "other.hpp"
#include "meanshift.h"
#include "gaussfilter.h"
#include "window_model.h"

string g_str;

int main (int argc, char** argv)
{
	if (argc < 2)
	{
		printf("please execute command line：%s  xx.pcd [-win]\n", argv[0]);
		exit(-1);
	}

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

	//对是否有窗户进行建模
	if (strcmp(argv[2], "-win") != 0)
	{
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
	} 
	else
	{
		size_t cnt = v_merged_pntcldplane.size();
		std::vector<WindowModelParams> v_wmparams;
		v_wmparams.resize(cnt);
		for (size_t i = 0; i < cnt; ++i)
		{
			PointCloud<PointXYZ>::Ptr cld(new PointCloud<PointXYZ>());

			pointcloudplane2pointcloud(v_merged_pntcldplane[i], cld);

			//对各个墙面建模
			WindowModel winmodel;
			WindowModelParams wmparams;
			winmodel.setInputCloud(cld);
			winmodel.setInputPlane(v_merged_pntcldplane[i].m_coeff);
			winmodel.setKNNRadius(1.0f);
			winmodel.checkAllPointsIsBoundary();
			winmodel.computeWinWidthAndMarginDist(wmparams.width, wmparams.margin_lr_dist, wmparams.horizon_wins_dist);
			winmodel.computeWinWidthAndMarginDist(wmparams.height, wmparams.margin_ud_dist, wmparams.vertical_wins_dist);

			v_wmparams[i] = wmparams;
		}

		VecRect v_rect;
		traverse_determin_planes_verticles(cloud_filtered, v_merged_pntcldplane, v_rect);

		PointCloud<PointXYZ>::Ptr cld_rebuilded(new PointCloud<PointXYZ>());
		for (size_t i = 0; i < v_rect.size(); ++i)
		{
			determin_plane_from_rect(v_rect[i], cld_rebuilded);
		}

		{
			string str, str2;
			str = g_str;

			str2 = (str += "/");
			str += "cloud_rebuiled.pcd";

			io::savePCDFileASCII(str, *cld_rebuilded);
		}
	}


	system("pause");
	return (0);
}
