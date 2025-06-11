#ifndef _BOX3_
#define _BOX3_

#include "point3.h"
#include "matrix44.h"

using namespace sn3DCore;

template <class BoxScalarType>
class Box3
{
public:

	typedef BoxScalarType ScalarType;

//////////////////////////////////// 构造函数  /////////////////////////////////////////////

	inline  Box3() { min.X()= 1;max.X()= -1;min.Y()= 1;max.Y()= -1;min.Z()= 1;max.Z()= -1;}
	inline  Box3( const Box3 & b ) { min=b.min; max=b.max; }
	inline  Box3( const Point3<BoxScalarType> & mi, const Point3<BoxScalarType> & ma ) { min = mi; max = ma; }
    inline  Box3(const Point3<BoxScalarType> & center, const BoxScalarType & radius) 
	{
     min = center-Point3<BoxScalarType>(radius,radius,radius);
     max = center+Point3<BoxScalarType>(radius,radius,radius);
    }

	template <class Q>
	inline void Import( const Box3<Q> & b )
	{
		min.Import(b.min);
		max.Import(b.max);
	}

	template <class Q>
	static inline Box3 Construct( const Box3<Q> & b )
	{
    return Box3(Point3<BoxScalarType>::Construct(b.min),Point3<BoxScalarType>::Construct(b.max));
	}
		
	inline ~Box3() {}
	
/////////////////////////////////  数据操作  //////////////////////////////////////////////

	inline bool operator == ( Box3<BoxScalarType> const & p ) const
	{return min==p.min && max==p.max;}
	
	inline bool operator != ( Box3<BoxScalarType> const & p ) const
	{return min!=p.min || max!=p.max;}

	void Offset( const BoxScalarType s )
	{Offset( Point3<BoxScalarType> (s,s,s));}
		
	void Offset( const Point3<BoxScalarType> & delta )
	{min -= delta;max += delta;}

	void Set( const Point3<BoxScalarType> & p )
	{min = max = p;}

	void SetNull()
	{
	  min.X()= 1; max.X()= -1;
	  min.Y()= 1; max.Y()= -1;
	  min.Z()= 1; max.Z()= -1;
	}

	void Add( Box3<BoxScalarType> const & b )
	{
	  if(IsNull()) *this=b;
	  else
	  {
	    if(min.X() > b.min.X()) min.X() = b.min.X();
		if(min.Y() > b.min.Y()) min.Y() = b.min.Y();
		if(min.Z() > b.min.Z()) min.Z() = b.min.Z();
		if(max.X() < b.max.X()) max.X() = b.max.X();
		if(max.Y() < b.max.Y()) max.Y() = b.max.Y();
		if(max.Z() < b.max.Z()) max.Z() = b.max.Z();
	  }
	}

	void Add( const Point3<BoxScalarType> & p )
	{
	  if(IsNull()) Set(p);
	  else 
	  {
	    if(min.X() > p.X()) min.X() = p.X();
		if(min.Y() > p.Y()) min.Y() = p.Y();
		if(min.Z() > p.Z()) min.Z() = p.Z();
		if(max.X() < p.X()) max.X() = p.X();
		if(max.Y() < p.Y()) max.Y() = p.Y();
		if(max.Z() < p.Z()) max.Z() = p.Z();
	  }
	}
	
	void Add( const Matrix44<BoxScalarType> &m, const Box3<BoxScalarType> & b )
	{
	   const Point3<BoxScalarType> &mn= b.min;
	   const Point3<BoxScalarType> &mx= b.max;
       Add(m*(Point3<BoxScalarType>(mn[0],mn[1],mn[2])));
	   Add(m*(Point3<BoxScalarType>(mx[0],mn[1],mn[2])));
	   Add(m*(Point3<BoxScalarType>(mn[0],mx[1],mn[2])));
	   Add(m*(Point3<BoxScalarType>(mx[0],mx[1],mn[2])));
	   Add(m*(Point3<BoxScalarType>(mn[0],mn[1],mx[2])));
	   Add(m*(Point3<BoxScalarType>(mx[0],mn[1],mx[2])));
	   Add(m*(Point3<BoxScalarType>(mn[0],mx[1],mx[2])));
	   Add(m*(Point3<BoxScalarType>(mx[0],mx[1],mx[2])));
	}

	void Translate( const Point3<BoxScalarType> & p )
	{
		min += p;
		max += p;
	}

	bool IsIn( Point3<BoxScalarType> const & p ) const
	{
		return (
			min.X() <= p.X() && p.X() <= max.X() &&
			min.Y() <= p.Y() && p.Y() <= max.Y() &&
			min.Z() <= p.Z() && p.Z() <= max.Z()
		);
	}

	bool IsInEx( Point3<BoxScalarType> const & p ) const
	{
		return (
			min.X() <= p.X() && p.X() < max.X() &&
			min.Y() <= p.Y() && p.Y() < max.Y() &&
			min.Z() <= p.Z() && p.Z() < max.Z()
		);
	}

	bool Collide(Box3<BoxScalarType> const &b) const
	{
		return b.min.X()<max.X() && b.max.X()>min.X() &&
			   b.min.Y()<max.Y() && b.max.Y()>min.Y() &&
			   b.min.Z()<max.Z() && b.max.Z()>min.Z() ;
	}

	bool IsNull() const { return min.X()>max.X() || min.Y()>max.Y() || min.Z()>max.Z(); }

	bool IsEmpty() const { return min==max; }

///////////////////////////////////////  几何操作    ///////////////////////////////////////////

	BoxScalarType Diag() const
	{
		return Distance(min,max);
	}

	BoxScalarType SquaredDiag() const
	{
		return SquaredDistance(min,max);
	}

	Point3<BoxScalarType> Center() const
	{
		return (min+max)/2;
	}

	Point3<BoxScalarType> Dim() const
	{
		return (max-min);
	}

	BoxScalarType Volume() const
	{ return (max.X()-min.X())*(max.Y()-min.Y())*(max.Z()-min.Z());}

	inline BoxScalarType DimX() const { return max.X()-min.X();}

	inline BoxScalarType DimY() const { return max.Y()-min.Y();}

	inline BoxScalarType DimZ() const { return max.Z()-min.Z();}

	inline unsigned char MaxDim() const 
	{ 
		int i;
		Point3<BoxScalarType> diag = max-min;
		if(diag[0]>diag[1]) i=0; else i=1;
		return (diag[i]>diag[2])? i: 2;
	}

	inline unsigned char MinDim() const 
	{ 
		int i;
		Point3<BoxScalarType> diag =  max-min;
		if(diag[0]<diag[1]) i=0; else i=1;
		return (diag[i]<diag[2])? i: 2;
	}

	void Intersect( const Box3<BoxScalarType> & b )
	{
		if(min.X() < b.min.X()) min.X() = b.min.X();
		if(min.Y() < b.min.Y()) min.Y() = b.min.Y();
		if(min.Z() < b.min.Z()) min.Z() = b.min.Z();

		if(max.X() > b.max.X()) max.X() = b.max.X();
		if(max.Y() > b.max.Y()) max.Y() = b.max.Y();
		if(max.Z() > b.max.Z()) max.Z() = b.max.Z();

		if(min.X()>max.X() || min.Y()>max.Y() || min.Z()>max.Z()) SetNull();
	}
		

public:


	Point3<BoxScalarType> min;
	
	Point3<BoxScalarType> max;
		
	Point3<BoxScalarType> corner[8];

};

////////////////////////////////  类型定义   /////////////////////////////////////////////////////////////////

typedef Box3<short>  Box3s;
typedef Box3<int>	 Box3i;
typedef Box3<float>  Box3f;
typedef Box3<double> Box3d;

#endif

