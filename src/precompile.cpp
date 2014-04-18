// 该文件仅仅用于创建预编译头 [4/17/2014 pc]
#include "common.h"
#include "typesdef.h"

//顺便把这个函数给定义了，如果在头文件typesdef.h中且多个源文件包含typesdef.h会报函数重定义错误
Vector makeVector(const _Point & pnt1, const _Point & pnt2)
{
	return Vector(pnt1.x - pnt2.x, pnt1.y - pnt2.y, pnt1.z - pnt2.z);
}
