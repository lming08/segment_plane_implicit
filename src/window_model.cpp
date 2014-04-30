/*
 *	lming_08@hotmail.com
 */

#include "common.h"
#include "typesdef.h"
#include "common_operate.h"
#include "window_model.h"

const float WindowModel::Z_DELTA = 0.3f;
const float WindowModel::SIGMA = 0.14f;

extern inline bool get_parallel_ground_vector(const PlaneCoeff & plncoeff, Vector &vec);

void WindowModel::computPointNormals(Normal &normal)
{
	//comm_oper::computPointNormals(m_input, m_normals, 150);
	normal.normal_x = m_plane.a;
	normal.normal_y = m_plane.b;
	normal.normal_z = m_plane.c;
	normal.curvature = 0.0f;  //! 平面的曲率为0
}

void WindowModel::	getZMaxMinIndices(size_t &max_indice, size_t &min_indice)
{
	size_t pntcld_count = m_input->points.size();
	float z_max = FLT_MIN, z_min = FLT_MAX;

	for (size_t i = 0; i < pntcld_count; ++i)
	{
		float z = m_input->points[i].z;
		if (z_max < z)
		{
			z_max = z;
			max_indice = i;
		}
		if (z_min > z)
		{
			z_min = z;
			min_indice = i;
		}		
	}	
}

void WindowModel::getBilateralIndices(size_t & indice1, size_t & indice2)
{
	size_t pntcld_count = m_input->points.size();
#ifdef _RAND
	srand((unsigned)time(NULL));
#endif
	size_t rand_index = static_cast<size_t>(rand() % pntcld_count);

	Eigen::Vector3f vec_horizon;  //!平行于水平面且与窗户平面共面的向量
	getHorizonVector(vec_horizon);

	float max_dist1 = 0, max_dist2 = 0;
	for (size_t i = 0; i < pntcld_count; ++i)
	{
		PointXYZ &pnt = m_input->points[i];
		PointXYZ &rand_pnt = m_input->points[rand_index];
		Eigen::Vector3f between_points_vector(pnt.x - rand_pnt.x, pnt.y - rand_pnt.y, pnt.z - rand_pnt.z);

		float dot_product = between_points_vector.dot(vec_horizon);
		float dist = 0;

		if (dot_product > 0)
		{
			dist = dot_product;
			if (max_dist1 < dist)
			{
				max_dist1 = dist;
				indice1 = i;
			}
		} 
		else if (dot_product < 0)
		{
			dist = -dot_product;
			if (max_dist2 < dist)
			{
				max_dist2 = dist;
				indice2 = i;
			}
		}
	}  // for (size_t i = 0; i < pntcld_count; ++i)
}

void WindowModel::computeWinWidthAndMarginDist(float &width, float &margin_lr_dist, float &horizon_wins_dist)
{
	if ( m_boundaries.points.empty())
	{
		PCL_ERROR("checkAllPointsIsBoundary() must be called!\n");
		throw "checkAllPointsIsBoundary() must be called";
	}

	size_t pntcld_count = m_input->points.size();
	size_t z_max_indice = 0, z_min_indice = 0;

	getZMaxMinIndices(z_max_indice, z_min_indice);

	Eigen::Vector3f vec_horizon;  //!平行于水平面且与窗户平面共面的向量
	getHorizonVector(vec_horizon);
	Eigen::Vector3f vec_vertical;
	getVerticalVector(vec_vertical);

	PointXYZ highest_pnt(m_input->points[z_max_indice].x, m_input->points[z_max_indice].y, m_input->points[z_max_indice].z);
	PointXYZ lowest_pnt(m_input->points[z_min_indice].x, m_input->points[z_min_indice].y, m_input->points[z_min_indice].z);

	Eigen::Vector3f vec(highest_pnt.x - lowest_pnt.x, highest_pnt.y - lowest_pnt.y, highest_pnt.z - lowest_pnt.z);
	float wall_height = vec.dot(vec_vertical);  //!这里的vec_horizon为单位向量
	size_t iters = (size_t)(fabsf(wall_height) / WindowModel::Z_DELTA);

	PointXYZ pntinline = (wall_height > 0 ? lowest_pnt : highest_pnt);

	VecPointCloud v_lines;
	for (size_t it = 0; it < iters; ++it)
	{
		PointCloud<PointXYZ> pointcloud;
		for (size_t i = 0; i < pntcld_count; ++i)
		{
			//计算点到直线的距离
			PointXYZ &pnt = m_input->points[i];
			float pnt2line_dist;

			pnt2line_dist = getPntToLineDist(pnt, pntinline, vec_horizon);
			if (pnt2line_dist <= WIN_PNT2LINE_DIST_THRSHLD)
			{
				if (m_boundaries.points[i].boundary_point)
				{
					pointcloud.points.push_back(pnt);
				}				
			}			
		}	
		if ( !pointcloud.points.empty() && pointcloud.points.size() > 4)
		{
			v_lines.push_back(pointcloud);
		}	
	    pntinline.x += vec_vertical[0] * WindowModel::Z_DELTA;
		pntinline.y += vec_vertical[1] * WindowModel::Z_DELTA;
		pntinline.z += vec_vertical[2] * WindowModel::Z_DELTA;		
	}

	//对扫描结果进行处理
	float wid = 0, mar_dist = 0.0f, wins_dist = 0.0f;
	size_t wid_iter_count = 0, mar_iter_count = 0, wins_iter_count = 0;
	for (size_t i = 0; i < v_lines.size(); ++i)
	{
		size_t count_inlines = v_lines[i].points.size();
		bool flag_bilateral = false;
		for (size_t j = 0; j < count_inlines - 1; ++j)
		{
			float dist = comm_oper::getDistBetweenPoints(v_lines[i].points[j], v_lines[i].points[j + 1]);
			if (j < WIN_RELAXATION_FACTOR) //0 == j || count_inlines - 2 == j
			{
				if ( !flag_bilateral)
				{
					if (WIN_PNT2LR_BILATERAL_DIST_MIN < dist && dist < WIN_PNT2LR_BILATERAL_DIST_MAX)
					{
						mar_dist += dist;
						++mar_iter_count;
						flag_bilateral = true;
					}
				}
			} 
			else
			{
				if (dist < WIN_PNT2PNT_DIST_MIN)
				{
					//扫描线刚好与窗户的某个边重合
					break; 
				}
				else if (dist > WIN_PNT2PNT_DIST_MAX)
				{
					//中间有一块数据采集不够完整
					continue;
				}
				
				if ( (j & 1) != 0)  //  j % 2 != 0
				{
					wid += dist;
					++wid_iter_count;
				} 
				else
				{
					wins_dist += dist;
					++wins_iter_count;
				}				
				
			}
		}		
	}
	wid /= wid_iter_count;
	mar_dist /=mar_iter_count; 
	wins_dist /= wins_iter_count;

	width = wid;
	margin_lr_dist = mar_dist;
	horizon_wins_dist = wins_dist;
}

void WindowModel::computeWinHeightAndMarginDist(float &height, float &margin_ud_dist, float &vertical_wins_dist)
{
	if ( m_boundaries.points.empty())
	{
		PCL_ERROR("checkAllPointsIsBoundary() must be called!\n");
		throw "checkAllPointsIsBoundary() must be called";
	}

	size_t pntcld_count = m_input->points.size();
	size_t bilateral_indice1 = 0, bilateral_indice2 = 0;

	getBilateralIndices(bilateral_indice1, bilateral_indice2);

	Eigen::Vector3f vec_horizon;  //!平行于水平面且与窗户平面共面的向量
	getHorizonVector(vec_horizon);
	Eigen::Vector3f vec_vertical;
	getVerticalVector(vec_vertical);

	PointXYZ &bilateral_pnt1 = m_input->points[bilateral_indice1];
	PointXYZ &bilateral_pnt2 = m_input->points[bilateral_indice2];
	Eigen::Vector3f vec(bilateral_pnt1.x - bilateral_pnt2.x, bilateral_pnt1.y - bilateral_pnt2.y, bilateral_pnt1.z - bilateral_pnt2.z);
	float wall_width = vec.dot(vec_horizon);  //!这里的vec_horizon为单位向量
	size_t iters = (size_t)(fabsf(wall_width) / WindowModel::Z_DELTA);

	PointXYZ pntinline = (wall_width > 0 ? bilateral_pnt2 : bilateral_pnt1);

	VecPointCloud v_lines;
	for (size_t it = 0; it < iters; ++it)
	{
		PointCloud<PointXYZ> pointcloud;
		for (size_t i = 0; i < pntcld_count; ++i)
		{
			//计算点到直线的距离
			PointXYZ &pnt = m_input->points[i];
			float pnt2line_dist;

			pnt2line_dist = getPntToLineDist(pnt, pntinline, vec_vertical);
			if (pnt2line_dist <= WIN_PNT2LINE_DIST_THRSHLD)
			{
				if (m_boundaries.points[i].boundary_point)
				{
					pointcloud.points.push_back(pnt);
				}				
			}			
		}	
		if ( !pointcloud.points.empty()  && pointcloud.points.size() > 4)
		{
			v_lines.push_back(pointcloud);
		}
		pntinline.x += vec_horizon[0] * WindowModel::Z_DELTA;
		pntinline.y += vec_horizon[1] * WindowModel::Z_DELTA;
		pntinline.z += vec_horizon[2] * WindowModel::Z_DELTA;
	}

	//对扫描结果进行处理
	float wid = 0, mar_dist = 0.0f, wins_dist = 0.0f;
	size_t wid_iter_count = 0, mar_iter_count = 0, wins_iter_count = 0;
	for (size_t i = 0; i < v_lines.size(); ++i)
	{
		size_t count_inlines = v_lines[i].points.size();
		bool flag_bilateral = false;
		for (size_t j = 0; j < count_inlines - 1; ++j)
		{
			float dist = comm_oper::getDistBetweenPoints(v_lines[i].points[j], v_lines[i].points[j + 1]);
			if (j < WIN_RELAXATION_FACTOR) //0 == j || count_inlines - 2 == j
			{
				if ( !flag_bilateral)
				{
					if (WIN_PNT2UD_BILATERAL_DIST_MIN < dist && dist < WIN_PNT2UD_BILATERAL_DIST_MAX)
					{
						mar_dist += dist;
						++mar_iter_count;
						flag_bilateral = true;
					}
				}				
			} 
			else
			{
				if (dist < WIN_PNT2PNT_DIST_MIN)
				{
					//扫描线刚好与窗户的某个边重合
					break; 
				}
				else if (dist > WIN_PNT2PNT_DIST_MAX)
				{
					//中间有一块数据采集不够完整
					continue;
				}

				if ( (j & 1) != 0)  //  j % 2 != 0
				{
					wid += dist;
					++wid_iter_count;
				} 
				else
				{
					wins_dist += dist;
					++wins_iter_count;
				}				

			}
		}		
	}
	wid /= wid_iter_count;
	mar_dist /=mar_iter_count; 
	wins_dist /= wins_iter_count;

	height = wid;
	margin_ud_dist = mar_dist;
	vertical_wins_dist = wins_dist;
}

void WindowModel::getHorizonVector(Eigen::Vector3f &vec_horizon)
{
	Vector horizon;
	get_parallel_ground_vector(m_plane, horizon);
	vec_horizon[0] = horizon.x;
	vec_horizon[1] = horizon.y;
	vec_horizon[2] = horizon.z;
}

void WindowModel::getVerticalVector(Eigen::Vector3f &vec_vertical)
{
	Vector vec_horizon;
	get_parallel_ground_vector(m_plane, vec_horizon);

	Eigen::Vector3f v1(m_plane.a, m_plane.b, m_plane.c);
	Eigen::Vector3f v2(vec_horizon.x, vec_horizon.y, vec_horizon.z);
	vec_vertical = v1.cross(v2);
	vec_vertical.normalize();
}

inline float WindowModel::getPntToLineDist(const PointXYZ &pnt, const PointXYZ &pntinline, const Eigen::Vector3f &vec_direction)
{
	float m = vec_direction[0], n = vec_direction[1], L = vec_direction[2];  //! (m, n, L)必须为单位向量
	float k = m * (pnt.x - pntinline.x) + n * (pnt.y - pntinline.y) + L * (pnt.z - pntinline.z);
	float vx = pntinline.x + k * m, vy = pntinline.y + k * n, vz = pntinline.z + k * L;
	float diff_x = pnt.x - vx, diff_y = pnt.y - vy, diff_z = pnt.z - vz;
	float dist = sqrtf(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

	return dist;
}

void WindowModel::checkAllPointsIsBoundary()
{
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
	boundEst.compute(m_boundaries); 
}
