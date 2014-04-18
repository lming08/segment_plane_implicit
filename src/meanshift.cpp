// 使用了预编译头文件，因此必须包含以下两个头文件 [4/17/2014 pc]
#include "common.h"
#include "typesdef.h"
#include "meanshift.h"

const float MeanShift::NEAREST_ZERO = 0.01;
const float MeanShift::C = 2.0f;

bool MeanShift::setInputCloud(const PointCloud<PointXYZ>::Ptr pPntCloud)
{
	m_size = pPntCloud->points.size();
	mv_pntcld.resize(m_size);
	mv_local_mode.resize(m_size);
	
	mp_pointcloud = pPntCloud;

	for (size_t i = 0; i < m_size; ++i)
	{
		mv_pntcld[i].x = pPntCloud->points[i].x;
		mv_pntcld[i].y = pPntCloud->points[i].y;
		mv_pntcld[i].z = pPntCloud->points[i].z;
	}

	return true;
}

inline bool MeanShift::execMeanShiftEachPoint(const PointXYZ &in_pnt, Point &out_pnt)
{
	// Set up KDTree
	pcl::KdTreeFLANN<PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<PointXYZ>);
	tree->setInputCloud (mp_pointcloud);

	// Main Loop
	PointXYZ pnt = in_pnt;

	while (1)
	{
		// Neighbors containers
		std::vector<int> k_indices;
		std::vector<float> k_distances;

		float sum_weigh = 0;
		float x = 0.0f, y = 0.0f, z = 0.0f;
		float dist_pnts = 0.0f;

		tree->radiusSearch (pnt, R, k_indices, k_distances);

		for (int i = 0, pnumber = k_indices.size(); i < pnumber; ++i)
		{
			size_t index = k_indices[i];
			PointXYZ &nbhd_pnt = mp_pointcloud->points[index];
			float sqr_dist = k_distances[i];
			float gauss_param = sqr_dist / (R * R);
			float w = gauss(gauss_param);

			x += nbhd_pnt.x * w;
			y += nbhd_pnt.y * w;
			z += nbhd_pnt.z * w;
			sum_weigh += w;
		}
		x = x / sum_weigh;
		y = y / sum_weigh;
		z = z / sum_weigh;

		float diff_x = x - pnt.x, diff_y = y - pnt.y, diff_z = z - pnt.z;

		dist_pnts = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

		if (dist_pnts <= 0.0001/*MeanShift::NEAREST_ZERO*/)
		{
			break;
		}
		pnt.x = x;
		pnt.y = y;
		pnt.z = z;
	};

	out_pnt.x = pnt.x;
	out_pnt.y = pnt.y;
	out_pnt.z = pnt.z;

	return true;
}

bool MeanShift::mergeSameLocalModePoint(const VecPoint &v_localmode, VecVecPoint &vv_pnt)
{
	assert(v_localmode.size() == m_size);

	std::vector<bool> v_iscluster(m_size, false);

	for (size_t i = 0; i < m_size; ++i)
	{
		for (size_t j = i + 1; j < m_size; ++j)
		{
			const Point & lmpnt1 = v_localmode[i];
			const Point & lmpnt2 = v_localmode[j];
			Point  pnt1(mp_pointcloud->points[i].x, mp_pointcloud->points[i].y, mp_pointcloud->points[i].z);
			Point  pnt2(mp_pointcloud->points[j].x, mp_pointcloud->points[j].y, mp_pointcloud->points[j].z);
			float dist = 0.0f;

			dist = getPointsDist(lmpnt1, lmpnt2);
			if (dist <= MeanShift::NEAREST_ZERO)
			{
				//两个点可近似重合
				VecPoint v_pnt;

				if ( !v_iscluster[i] &&  !v_iscluster[j])
				{
					v_iscluster[i] = true;
					v_pnt.push_back(pnt1);

					v_iscluster[j] = true;
					v_pnt.push_back(pnt2);

					vv_pnt.push_back(v_pnt);
				}
				else if ( v_iscluster[i] &&  !v_iscluster[j])
				{
					size_t clustered_count = vv_pnt.size();
					size_t m, n;
					for (m = 0; m < clustered_count; ++m)
					{
						size_t count = vv_pnt[m].size();
						for (n = 0; n < count; ++n)
						{
							if (pnt1 == vv_pnt[m][n])
							{
								vv_pnt[m].push_back(pnt2);
								v_iscluster[j] = true;
								break;
							}
						}  // for n
					}  // for m
				}  // else if
				else if ( !v_iscluster[i] &&  v_iscluster[j])
				{
					size_t clustered_count = vv_pnt.size();
					size_t m, n;
					for (m = 0; m < clustered_count; ++m)
					{
						size_t count = vv_pnt[m].size();
						for (n = 0; n < count; ++n)
						{
							if (pnt2 == vv_pnt[m][n])
							{
								vv_pnt[m].push_back(pnt1);
								v_iscluster[i] = true;
								break;
							}
						}  // for n
					}  // for m
				}
				else
				{
					//都已经聚类，就不做处理
				}
			}  //  if (dist <= NEAREST_ZERO)

		}  //  for j
	}  //  for i

	for (size_t i = 0; i < m_size; ++i)
	{
		if (!v_iscluster[i])
		{
			Point  pnt(mp_pointcloud->points[i].x, mp_pointcloud->points[i].y, mp_pointcloud->points[i].z);
			VecPoint v_pnt;

			v_iscluster[i] = true;
			v_pnt.push_back(pnt);
			vv_pnt.push_back(v_pnt);
		}
	}

	return true;
}

bool MeanShift::process()
{
	for (size_t i = 0; i < m_size; ++i)
	{
		const PointXYZ &pnt = mp_pointcloud->points[i];
		execMeanShiftEachPoint(pnt, mv_local_mode[i]);
	}

	mergeSameLocalModePoint(mv_local_mode, vv_pnt);

	return true;
}

bool MeanShift::SaveFile(const char *dir_name, const char *prex_name)
{
	size_t cluster_count = vv_pnt.size();

	for (size_t i = 0; i < cluster_count; ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

		for (size_t j = 0, grp_pnt_count = vv_pnt[i].size(); j < grp_pnt_count; ++j)
		{
			pcl::PointXYZ pt;
			pt.x = vv_pnt[i][j].x;
			pt.y = vv_pnt[i][j].y;
			pt.z = vv_pnt[i][j].z;

			p_pnt_cloud->points.push_back(pt);
		}

		p_pnt_cloud->width = static_cast<size_t>(vv_pnt[i].size());
		p_pnt_cloud->height = 1;

		char newFileName[256] = {0};
		char indexStr[16] = {0};

		strcat(newFileName, dir_name);
		strcat(newFileName, "/");
		strcat(newFileName, prex_name);
		strcat(newFileName, "-");
		sprintf(indexStr, "%d", i + 1);
		strcat(newFileName, indexStr);
		strcat(newFileName, ".pcd");
		io::savePCDFileASCII(newFileName, *p_pnt_cloud);
	}

	return true;
}
