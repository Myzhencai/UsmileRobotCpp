#ifndef _SN3D_RANGE_DATA_
#define _SN3D_RANGE_DATA_

/*   RangeData ��

�����װ�����ͼ��ʽ��ɨ�輸�����ݵı�ʾ��������,��Ҫ����:

1.���ͼ�������ݱ�ʾ��������

2.���ͼ�������ݴ洢����                                   */
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
	//	������ sn3DRangeData
	//
	//	���ܣ� �������ͼ���ݡ�
	//			���ͼ������֯��ʽ��ͼ�������ƣ�ֻ����ÿ�����ش洢����һ������
	//		�����������ö��㼴�Ǹ�����λ�õĶ������ά���ꡣ
	//			��פ�ڴ����ݣ� ���������Ͷ�������
	//			���ͷ��ڴ����ݣ����㷨�������涥��������������룬������ɫ
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

		////////////////////////  ���ݴ�����  //////////////////////////////
		// ��������
		bool SetPoints(int w, int h, int* ids, int npt, double *x, double *y, double *z);
		bool SetMap(int w, int h, int *ids=NULL);
		bool Resample( int rate ); 
		bool ResampleNorm(int rate, bool face);
		sn3DRangeData *Clone();
		sn3DRangeData *SimpleClone();		
		void Compact();
		void SetPointS(int index);
		void ClearPointS(int index);
		// ��ȡ��Χ��
		bool GetBox(Point3f &cnt, Point3f &max,Point3f &min);
		void UpdateBox();

		///////////////////////   ���ݷ���    ////////////////////////////////
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
		vector<Point3f> &GetNorms()		{ return m_pntNorm;};		//���ͷ� BuildNormal()����
		vector<float>	&GetDist()		{ return m_pntDist;}		//���ͷ� �ⲿ����
		vector<Point3f> &GetColor()		{ return m_pntColor;}		//���ͷ� �ⲿ����
		vector<Triangle>&GetFaces()		{ return m_faces;};			//���ͷ� BuildMesh()����

		////////////////////////   �ؽ����񷽷�   /////////////////////
		

		// �ؽ����Ʒ���
		bool FixPoint();
		bool BuildNormal();
		bool BuildNormal(int size);

		// ���ұ߽� 
		void FindBoundary();

		// ���������ؽ�����
		void SetMeshParameter(float maxDepth, float maxDis, float minAngle);
		void BuildMesh();						// ���������ؽ�

		///////////////////////  �ڴ��������   ///////////////////////////////
		void  ReleaseData(bool all);              // �ͷ���Ҫ�ڴ�����
		void  ReleaseNorm();
		void  ReleaseDist();
		void  ReleaseColor();
		void  ReleaseFace();
	protected:
		// ����ģ�Ͱ�Χ�к�����
		bool GetCenter(  Point3f &cnt, Point3f &max,Point3f &min );

		// ������������ľ���ƽ��
		float distance2(int i0, int i1);

		//--------------------------------------------------------------------
		// �����ĸ��������������ؽ�������Ƭ����BuildMesh����
		// �ĸ���ָ������������
		int   DivideQuad(Triangle &tri0,Triangle &tri1, int id0, int id1, int id2, int id3);
		// �õ��������㣬�ж��Ƿ��ܹ��γ������Σ��������������id���������ö���idΪ-1
		// ���ú��� BuildTriangle()��SetTriangle����
		int   BuildTriangle(Triangle &tri,int id0, int id1,int id2);

		// �ж��Ƿ������ӳ�������
		//		1 �����ֵ
		//		2 �����߳�
		//		3 ��������С�ڽ�
		bool  TriangleRules( int id0, int id1,int id2 );
		// ���������ζ���id
		void  SetTriangle(Triangle &tri,int id0, int id1,int id2);
		//--------------------------------------------------------------------------

		//���ƽ��
		bool GetPlan(vector<Point3f> point, Point3f &normal);

		void GuassEliminate(double A[][3], double *B, double *Answer);
	public:
		int vn;			// ������  ��Ч�����
		int fn;			// ���������Ч�����
		int w;			// ���ͼ��
		int h;			// ���ͼ��
		MatrixData<int>			m_maps;		// ��ŵ������id
		std::vector<Point3f>	m_pnts;		// �������б�
		std::vector<int>        m_flags;
		std::vector<Point3f>	m_pntNorm;	// ���㷨��		ͨ��Ϊ��
		std::vector<Point3f>	m_pntColor;	// ������ɫ
		std::vector<float>		m_pntDist;	// �������ԣ�����
		std::vector<Triangle>	m_faces;	// ������б�

	protected:
		Box3f	m_box;						// ����ģ�Ͱ�Χ��
		Point3f m_center;					// ģ������

	protected:
		float m_maxDepth2;					// ������ ƽ��
		float m_maxDis2;					// ������������
		float m_minAngleTri2;				// ��������С�ǵ�����ƽ��
		
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
