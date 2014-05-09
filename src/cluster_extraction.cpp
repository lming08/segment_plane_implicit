//http://www.pointclouds.org/documentation/tutorials/extract_indices.php

#include "common.h"
#include "typesdef.h"
#include "process.h"
#include "other.hpp"
#include "meanshift.h"
#include "gaussfilter.h"

#if defined _SCAN
#include "window_model.h"
#elif defined _CLUSTER
#include "wins.h"
#else
#error "必须要定义宏_SCAN或_CLUSTER"
#endif

#include <gflags/gflags.h>

string g_work_dir_path;  //!work directory
extern void save_vecpntcldpln(const char * filename, const VecPointCloudPlane &v_pntcldplane);
extern void load_vecpntcldpln(const char * filename, VecPointCloudPlane &v_pntcldplane);

DEFINE_string(execfile, "", "executable file path name");
DEFINE_string(workdir, "", "work directory name");
DEFINE_bool(particular, false, "either generate xml file particularly or mot");
DEFINE_bool(win, false, "either windows model or mot");
DEFINE_bool(ar, false, "either archive xml file or mot");
DEFINE_string(pcdfile, "", "pcd file name");
DEFINE_string(xmlfile, "", "xml file name");

int main (int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("please execute command line：%s  -particular=false -win=false -workdir=pcd_0 -pcdfile=0.pcd -xmlfile=archive_vecpntcldpln.xml\n", argv[0]);
		exit(-1);
	}
	//parse commandline parameters
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	//get_output_dir(string(argv[0]), FLAGS_workdir, g_work_dir_path);
	g_work_dir_path = "./" + FLAGS_workdir;
	string pcdfile_path = g_work_dir_path + "/" + FLAGS_pcdfile;
	string xmlfile_path = g_work_dir_path + "/" + FLAGS_xmlfile;

	cout<<g_work_dir_path<<endl<<FLAGS_workdir<<endl<<FLAGS_pcdfile<<endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	io::loadPCDFile(pcdfile_path, *cloud);
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

	VecPointCloudPlane v_merged_pntcldplane;
	if (FLAGS_particular)
	{
		VecPointIndices v_pnt_indices;//存储每个点云平面所在的下标索引
		VecPlaneCoeff v_coeff;//存储各个平面的系数
		std::vector<VecPoint> v_cloud_plane;//存储各个平面的点云
		segment_plane(cloud_filtered, v_pnt_indices, v_coeff, v_cloud_plane, false);

		VecPointCloudPlane v_pntcldplane;
		std::vector<VecPointCloudPlane> v_vec_pntcldplane_clustered;
		build_vec_pointcloudplane(v_cloud_plane, v_coeff, v_pntcldplane);
		cluster_parallel_plane(v_pntcldplane, v_vec_pntcldplane_clustered);

		traverse_merge_planes(v_vec_pntcldplane_clustered, v_merged_pntcldplane);

		string archive_vecpntcldpln = g_work_dir_path + "/" +"archive_vecpntcldpln.xml";
		save_vecpntcldpln(archive_vecpntcldpln.c_str(), v_merged_pntcldplane);
	}
	else
	{
		load_vecpntcldpln(xmlfile_path.c_str(), v_merged_pntcldplane);
	}

	VecRect v_rect;  //!墙面矩形的4个顶点
	traverse_determin_planes_verticles(cloud_filtered, v_merged_pntcldplane, v_rect);

	//对是否有窗户进行建模
	if ( !FLAGS_win)
	{
		int index = 0;
		for (size_t i = 0, cnt = v_merged_pntcldplane.size(); i < cnt; ++i)
		{
			PointCloud<PointXYZ>::Ptr cld(new PointCloud<PointXYZ>());

			pointcloudplane2pointcloud(v_merged_pntcldplane[i], cld);

			std::stringstream ss;
			ss << "merge_" << index << ".pcd";
			index++;

			string str = g_work_dir_path;
			str += "/";
			str += string(ss.str());
			io::savePCDFileASCII(str, *cld);
		}
		
		PointCloud<PointXYZ>::Ptr cld_rebuilded(new PointCloud<PointXYZ>());
		PointCloud<PointXYZ>::Ptr cld_contour_rebuilded(new PointCloud<PointXYZ>());
		for (size_t i = 0; i < v_rect.size(); ++i)
		{
			determin_plane_from_rect(v_rect[i], cld_rebuilded);
			determin_plane_from_rect_only_contour(v_rect[i], cld_contour_rebuilded);
		}

		{
			string str, str2;
			str = g_work_dir_path;

			str2 = (str += "/");
			str += "cloud_rebuiled.pcd";

			io::savePCDFileASCII(str, *cld_rebuilded);
			str2 += "cloud_contour_rebuiled.pcd";
			io::savePCDFileASCII(str2, *cld_contour_rebuilded);
		}
	} 
	else
	{
		PointCloud<PointXYZ>::Ptr cld_rebuilded(new PointCloud<PointXYZ>());
		size_t cnt = v_merged_pntcldplane.size();

#if defined _SCAN
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
			winmodel.setKNNRadius(WIN_BNDARY_KNN_RADIUS);
			WindowModel::PointCloudBoundary boundaries;
			winmodel.checkAllPointsIsBoundary(boundaries);
			winmodel.computeWinWidthAndMarginDist(boundaries, wmparams.width, wmparams.margin_lr_dist, wmparams.horizon_wins_dist);
			winmodel.computeWinHeightAndMarginDist(boundaries, wmparams.height, wmparams.margin_ud_dist, wmparams.vertical_wins_dist);

			v_wmparams[i] = wmparams;
		}

		for (size_t i = 0; i < v_rect.size(); ++i)
		{
			determin_plane_from_rect_winmodel(v_rect[i], v_wmparams[i], cld_rebuilded);
		}

#elif defined _CLUSTER
		VecVecRect vv_wins_rect;
		vv_wins_rect.resize(cnt);
		for (size_t i = 0; i < cnt; ++i)
		{
			PointCloud<PointXYZ>::Ptr cld(new PointCloud<PointXYZ>());

			pointcloudplane2pointcloud(v_merged_pntcldplane[i], cld);

			//对各个窗户建模
			Wins win;
			win.setInputCloud(cld);
			win.setInputPlane(v_merged_pntcldplane[i].m_coeff);
			win.setKNNRadius(WIN_BNDARY_KNN_RADIUS);
			win.computeWins(vv_wins_rect[i], false);
		}

		PointCloud<PointXYZ>::Ptr cld_contour_rebuilded(new PointCloud<PointXYZ>());
		for (size_t i = 0; i < v_rect.size(); ++i)
		{
			//determin_plane_from_rect(v_rect[i], cld_rebuilded);
			determin_plane_from_rect_only_contour(v_rect[i], vv_wins_rect[i], cld_contour_rebuilded);
		}
#else
#error "必须要定义宏_SCAN或_CLUSTER"
#endif

		{
			string str, str2;
			str = g_work_dir_path;

			str2 = (str += "/");
			str += "cloud_windows_rebuiled.pcd";
			//pcl::io::savePCDFileASCII(str, *cld_rebuilded);

			str2 += "cloud_windows_contour_rebuiled.pcd";
			io::savePCDFileASCII(str2, *cld_contour_rebuilded);
		}
	}

	system("pause");
	return (0);
}
