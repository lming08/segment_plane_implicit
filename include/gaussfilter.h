#ifndef _GAUSSFILTER_H_
#define _GAUSSFILTER_H_

#include "common.h"

class GaussFilter
{
public:
	static const float NEAREST_ZERO;
	typedef PointCloud<PointXYZ>::Ptr PointCloudPtr;

	GaussFilter() : m_radius(0.0f), m_sigma(0.0f){}

	void setInput(const PointCloudPtr pntcld)
	{
		m_input = pntcld;
	}

	void setRadius(const float r)
	{
		m_radius = r;
	}

	void setSigma(const float sigma)
	{
		m_sigma = sigma;
	}

	inline float gauss(float sqr_dist)
	{
		return exp(-0.5 * sqr_dist / (m_sigma * m_sigma));
	}

	bool filter(PointCloudPtr output);

private:
	PointCloudPtr m_input;
	float m_radius;
	float m_sigma;
};

#endif
