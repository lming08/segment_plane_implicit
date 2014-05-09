/*
 *	序列化
 *	lming_08@hotmail.com
 */
#include "common.h"
#include "typesdef.h"
#include <fstream> 
#include <vector>
#include <boost/archive/xml_oarchive.hpp> 
#include <boost/archive/xml_iarchive.hpp> 
#include <boost/serialization/vector.hpp>

namespace boost 
{
	namespace serialization 
	{
		template<class Archive>
		void serialize(Archive& archive, PointCloudPlane &pntcldpln, const unsigned int version)
		{
			archive & BOOST_SERIALIZATION_NVP(pntcldpln.m_coeff);
			archive & BOOST_SERIALIZATION_NVP(pntcldpln.m_indice);
			archive & BOOST_SERIALIZATION_NVP(pntcldpln.mv_pointcloud);
		}

		template<class Archive>
		void serialize(Archive& archive, PlaneCoeff &plncoeff, const unsigned int version)
		{
			archive & BOOST_SERIALIZATION_NVP(plncoeff.a);
			archive & BOOST_SERIALIZATION_NVP(plncoeff.b);
			archive & BOOST_SERIALIZATION_NVP(plncoeff.c);
			archive & BOOST_SERIALIZATION_NVP(plncoeff.d);
		}

		template<class Archive>
		void serialize(Archive& archive, size_t &indice, const unsigned int version)
		{
			archive & BOOST_SERIALIZATION_NVP(indice);
		}

		template<class Archive>
		void serialize(Archive& archive, Point &pnt, const unsigned int version)
		{
			archive & BOOST_SERIALIZATION_NVP(pnt.x);
			archive & BOOST_SERIALIZATION_NVP(pnt.y);
			archive & BOOST_SERIALIZATION_NVP(pnt.z);
		}

		//////////////////////////////////////////////////////////////////////////
		//template<class Archive>
		//void serialize(Archive& archive, pcl::PointCloud<pcl::PointXYZ> &cld, const unsigned int version)
		//{
		//	archive & BOOST_SERIALIZATION_NVP(cld.points);
		//}

		//template<class Archive>
		//void serialize(Archive& archive, pcl::PointXYZ &pnt, const unsigned int version)
		//{
		//	archive & BOOST_SERIALIZATION_NVP(pnt.x);
		//	archive & BOOST_SERIALIZATION_NVP(pnt.y);
		//	archive & BOOST_SERIALIZATION_NVP(pnt.z);
		//}
	}
}

void save_vecpntcldpln(const char * filename, const VecPointCloudPlane &v_pntcldplane) 
{ 
	std::ofstream file(filename); 
	boost::archive::xml_oarchive oa(file); 

	oa & BOOST_SERIALIZATION_NVP(v_pntcldplane); 
}

void load_vecpntcldpln(const char * filename, VecPointCloudPlane &v_pntcldplane) 
{
	std::ifstream file(filename); 
	boost::archive::xml_iarchive ia(file); 

	ia >> BOOST_SERIALIZATION_NVP(v_pntcldplane);
}

//void save_pntcld(const char * filename, const pcl::PointCloud<pcl::PointXYZ> &v_pntcld) 
//{ 
//	std::ofstream file(filename); 
//	boost::archive::xml_oarchive oa(file); 
//
//	oa & BOOST_SERIALIZATION_NVP(v_pntcld); 
//}
//
//void load_pntcld(const char * filename, pcl::PointCloud<pcl::PointXYZ> &v_pntcld) 
//{
//	std::ifstream file(filename); 
//	boost::archive::xml_iarchive ia(file); 
//
//	ia >> BOOST_SERIALIZATION_NVP(v_pntcld);
//}
