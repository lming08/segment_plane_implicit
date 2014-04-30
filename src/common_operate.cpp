/*
 *	lming_08@hotmail.com
 */

#include "common.h"
#include "typesdef.h"
#include "common_operate.h"

Vector comm_oper::makeVector(const _Point & pnt1, const _Point & pnt2)
{
	return Vector(pnt1.x - pnt2.x, pnt1.y - pnt2.y, pnt1.z - pnt2.z);
}

float comm_oper::getDistBetweenPoints(const PointXYZ &pnt1, const PointXYZ &pnt2)
{
	float diff_x = pnt1.x - pnt2.x;
	float diff_y = pnt1.y - pnt2.y;
	float diff_z = pnt1.z - pnt2.z;

	return sqrtf(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}

void comm_oper::computPointNormals(const PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr normals, const size_t K)
{
	// Create search tree
	search::KdTree<PointXYZ>::Ptr tree;
	tree.reset (new search::KdTree<PointXYZ> (false));
	tree->setInputCloud (cloud);

	// Normal estimation
	NormalEstimation<PointXYZ, Normal> n;
	n.setInputCloud (cloud);
	//n.setIndices (indices[B);
	n.setSearchMethod (tree);
	n.setKSearch (K);
	n.compute (*normals);
}
