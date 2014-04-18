#ifndef _MEANSHIFT_H_
#define _MEANSHIFT_H_

#include "common.h"
#include "typesdef.h"

class MeanShift
{
public:
	static const float NEAREST_ZERO;
	static const float C;  //!高斯核函数中的常数

	MeanShift() : m_size(0), R(0.0f){}

	 /** \brief 设置输入点云
	   * \param[in]  pPntCloud 输入点云
	   */
	bool setInputCloud(const PointCloud<PointXYZ>::Ptr pPntCloud);


	 /** \brief 获取聚类后的点云  */
	VecVecPoint & getOutputCloud()
	{
		return vv_pnt;
	}

	/** \brief 设置K近邻算法的搜索半径
	   * \param[in]  radius 半径
	   */
	bool setKNNRadius(const float radius)
	{
		R = radius;
        return true;
	}

	 /** \brief 执行MeanShift聚类算法	   */
	bool process();

	 /** \brief 保存聚类后的点云文件
	   * \param[in] dir_name 保存的目录
	   * \param[in] prex_name 文件名前缀
	   */
	bool SaveFile(const char *dir_name, const char *prex_name);

private:
	size_t m_size;  //!要处理点的个数
	PointCloud<PointXYZ>::Ptr mp_pointcloud;  //!PCL形式的点云，主要是为了使用FLANN的KD树
	VecPoint mv_pntcld;  //!点云
	VecPoint mv_local_mode;  //!每个点的局部模式
	VecVecPoint vv_pnt;  //!聚类后的点云

	float R;  //!K近邻算法时对应的搜索半径

	 /** \brief 对每个点执行MeanShift
	   * \param[in]  in_pnt 输入的点
	   * \param[out] out_pnt 输出的点
	   */
	inline bool execMeanShiftEachPoint(const PointXYZ &in_pnt, Point &out_pnt);

	 /** \brief 将具有相同局部模式的点归为一类
	   * \param[in] v_localmode 各个点对应的局部模式
	   * \param[out] vv_pnt 归并后的点
	   */
	bool mergeSameLocalModePoint(const VecPoint &v_localmode, VecVecPoint &vv_pnt);

	inline float gauss(float x)
	{
		return C * sqrt(x) * exp(-0.5 * x);
	}

};

#endif
