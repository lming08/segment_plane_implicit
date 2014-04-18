// 使用了预编译头文件，因此必须包含以下两个头文件 [4/17/2014 pc]
#include "common.h"
#include "typesdef.h"
#include "gaussfilter.h"

const float GaussFilter::NEAREST_ZERO = 0.01;

bool GaussFilter::filter(PointCloudPtr output)
{
	if (-GaussFilter::NEAREST_ZERO < m_radius && m_radius <GaussFilter::NEAREST_ZERO
		|| -GaussFilter::NEAREST_ZERO < m_sigma && m_sigma < GaussFilter::NEAREST_ZERO)
	{
		PCL_ERROR("必须调用setRadius和setSigma\n");
		return false;
	}

	size_t pntcount = m_input->width * m_input->height;
	output->points.resize(pntcount);
	output->width = pntcount;
	output->height = 1;

	// Set up KDTree
	pcl::KdTreeFLANN<PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<PointXYZ>);
	tree->setInputCloud (m_input);
	
	std::vector<int> indices;
	std::vector<float> distances;
	for (size_t i = 0; i < pntcount; ++i)
	{
		tree->radiusSearch(i, m_radius, indices, distances);

		float w = 0, sum_w = 0;
		size_t count = indices.size();
		float x = 0, y = 0, z = 0;
		for (size_t j = 0; j < count; ++j)
		{
			w = gauss(distances[j]);
			x += w * m_input->points[indices[j]].x;
			y += w * m_input->points[indices[j]].y;
			z += w * m_input->points[indices[j]].z;

			sum_w += w;
		}
		x /= sum_w;
		y /= sum_w;
		z /= sum_w;
		output->points[i].x = x;
		output->points[i].y = y;
		output->points[i].z = z;
	}

	return true;
}
