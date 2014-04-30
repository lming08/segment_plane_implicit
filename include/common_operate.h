/*
 *	lming_08@hotmail.com
 */
#ifndef _COMMON_OPERATE_H_
#define _COMMON_OPERATE_H_

namespace comm_oper
{
	Vector makeVector(const _Point & pnt1, const _Point & pnt2);

	float getDistBetweenPoints(const PointXYZ &pnt1, const PointXYZ &pnt2);

	void computPointNormals(const PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr normals, const size_t K = 150);
}

#endif
