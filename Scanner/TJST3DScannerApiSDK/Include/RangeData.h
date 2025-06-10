#ifndef _SN3D_RANGE_DATA_
#define _SN3D_RANGE_DATA_

/*   RangeData 类

该类封装了深度图形式的扫描几何数据的表示及处理方法,主要包括:

1.深度图几何数据表示、处理方法

2.深度图几何数据存储方法                                   */
#include "MatrixData.h"
#include "box3.h"
#include "point3.h"

using namespace std; 

namespace sn3DCore
{
	class Triangle
	{
	public:
		int& operator[](int j){ return t[j]; }
		int t[3];
		
	};
	//typedef int Triangle[3];
	//////////////////////////////////////////////////////////////////////////
	//
	//	类名： sn3DRangeData
	//
	//	功能： 保存深度图数据。
	//			深度图数据组织方式与图像相类似，只不过每个像素存储的是一个顶点
	//		数组索引。该顶点即是该像素位置的顶点的三维坐标。
	//			常驻内存数据： 顶点索引和顶点数组
	//			可释放内存数据：顶点法向，三角面顶点索引，顶点距离，顶点颜色
	//
	//////////////////////////////////////////////////////////////////////////
#ifdef _LINUX_OS_
	class __attribute__((visibility("default"))) sn3DRangeData
#else
	class __declspec(dllimport) sn3DRangeData
#endif
	{
	public:

		sn3DRangeData();
		virtual ~sn3DRangeData();

		////////////////////////  数据处理方法  //////////////////////////////
		// 设置数据
		bool SetPoints(int w, int h, int* ids, int npt, double *x, double *y, double *z);
		bool SetMap(int w, int h, int *ids=NULL);
		bool Resample( int rate ); 
		bool ResampleNorm(int rate, bool face);
		sn3DRangeData *Clone();
		sn3DRangeData *SimpleClone();		
		void Compact();
		void SetPointS(int index);
		void ClearPointS(int index);
		// 获取包围盒
		bool GetBox(Point3f &cnt, Point3f &max,Point3f &min);
		void UpdateBox();

		///////////////////////   数据访问    ////////////////////////////////
		void SetVN(int n){vn=n;};
		int GetVN()		{ return vn; };
		int GetFN()		{ return fn; };
		int GetW()		{ return w; };
		int GetH()		{ return h;};

		bool      IsInvalid(int i,int j){ return m_maps[i][j]<0;};
		Point3f&  GetPoint(int i, int j){ return m_pnts[m_maps[i][j]];};
		Point3f&  GetNorm( int i, int j){ return m_pntNorm[m_maps[i][j]];};
		Triangle& GetFace(int i)		{ return m_faces[i];};

		bool HasPoints(){ return m_pnts.size()>0;   };
		bool HasNormal(){ return m_pntNorm.size()>0;};
		bool HasFace()	{ return m_faces.size()>0;  };

		MatrixData<int>	&GetMap()		{ return m_maps;};
		vector<Point3f> &GetPoints()	{ return m_pnts;};
		vector<Point3f> &GetNorms()		{ return m_pntNorm;};		//可释放 BuildNormal()分配
		vector<float>	&GetDist()		{ return m_pntDist;}		//可释放 外部分配
		vector<Point3f> &GetColor()		{ return m_pntColor;}		//可释放 外部分配
		vector<Triangle>&GetFaces()		{ return m_faces;};			//可释放 BuildMesh()分配

		////////////////////////   重建网格方法   /////////////////////
		

		// 重建点云法向
		bool FixPoint();
		bool BuildNormal();
		bool BuildNormal(int size);

		// 查找边界 
		void FindBoundary();

		// 设置网格重建参数
		void SetMeshParameter(float maxDepth, float maxDis, float minAngle);
		void BuildMesh();						// 三角网格重建

		///////////////////////  内存操作方法   ///////////////////////////////
		void  ReleaseData(bool all);              // 释放主要内存数据
		void  ReleaseNorm();
		void  ReleaseDist();
		void  ReleaseColor();
		void  ReleaseFace();
	protected:
		// 计算模型包围盒和中心
		bool GetCenter(  Point3f &cnt, Point3f &max,Point3f &min );

		// 计算两个顶点的距离平方
		float distance2(int i0, int i1);

		//--------------------------------------------------------------------
		// 下面四个函数，是用来重建三角面片，供BuildMesh调用
		// 四个点分割成两个三角形
		int   DivideQuad(Triangle &tri0,Triangle &tri1, int id0, int id1, int id2, int id3);
		// 得到三个顶点，判断是否能够形成三角形，如果可以则设置id，否则设置顶点id为-1
		// 调用函数 BuildTriangle()和SetTriangle（）
		int   BuildTriangle(Triangle &tri,int id0, int id1,int id2);

		// 判断是否能连接成三角形
		//		1 深度阈值
		//		2 三个边长
		//		3 三角形最小内角
		bool  TriangleRules( int id0, int id1,int id2 );
		// 设置三角形顶点id
		void  SetTriangle(Triangle &tri,int id0, int id1,int id2);
		//--------------------------------------------------------------------------

		//拟合平面
		bool GetPlan(vector<Point3f> point, Point3f &normal);

		void GuassEliminate(double A[][3], double *B, double *Answer);
	public:
		int vn;			// 点数，  有效点个数
		int fn;			// 面个数，有效面个数
		int w;			// 深度图宽
		int h;			// 深度图高
		MatrixData<int>			m_maps;		// 存放点数组的id
		std::vector<Point3f>	m_pnts;		// 点数组列表
		std::vector<int>        m_flags;
		std::vector<Point3f>	m_pntNorm;	// 顶点法向		通常为空
		std::vector<Point3f>	m_pntColor;	// 顶点颜色
		std::vector<float>		m_pntDist;	// 顶点属性，距离
		std::vector<Triangle>	m_faces;	// 存放面列表

	protected:
		Box3f	m_box;						// 几何模型包围盒
		Point3f m_center;					// 模型中心

	protected:
		float m_maxDepth2;					// 最大深度 平方
		float m_maxDis2;					// 顶点间的最大距离
		float m_minAngleTri2;				// 三角形最小角的余弦平方
		
		friend class sn3DRangeModel;
	};
	//========================================================================
	inline float sn3DRangeData::distance2(int i0, int i1)
	{
		return   (m_pnts[i1][0]-m_pnts[i0][0])*(m_pnts[i1][0]-m_pnts[i0][0])
				+(m_pnts[i1][1]-m_pnts[i0][1])*(m_pnts[i1][1]-m_pnts[i0][1])
				+(m_pnts[i1][2]-m_pnts[i0][2])*(m_pnts[i1][2]-m_pnts[i0][2]);
	}
	inline void sn3DRangeData::SetTriangle(Triangle &tri,int id0, int id1,int id2 )
	{
		tri[0] = id0;
		tri[1] = id1;
		tri[2] = id2;
	}
}
#endif
