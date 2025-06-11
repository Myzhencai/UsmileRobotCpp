#ifndef _POINT4_
#define _POINT4_
#include <assert.h>

#include "base.h"
/*该模板类代表4D空间中的点*/
namespace sn3DCore
{
template <class T> class Point4
{
public:
	T _v[4];//唯一的数据成员对用户隐藏

public:
	typedef T ScalarType;
	enum {Dimension = 4};
/*构造器*/
	inline Point4 () { }
	inline Point4 ( const T nx, const T ny, const T nz , const T nw )
	{
		_v[0] = nx; _v[1] = ny; _v[2] = nz; _v[3] = nw;
	}
	inline Point4 ( const T  p[4] )
	{   
		_v[0] = p[0]; _v[1]= p[1]; _v[2] = p[2]; _v[3]= p[3];
	}
	inline Point4 ( const Point4 & p )
	{   
		_v[0]= p._v[0]; _v[1]= p._v[1]; _v[2]= p._v[2]; _v[3]= p._v[3];
	}
	
	
	inline void Zero()
	{   
		_v[0] = _v[1] = _v[2] = _v[3]= 0;
	}
	template <class Q>
	
	inline void Import( const Point4<Q> & b )
	{
		_v[0] = T(b[0]);		_v[1] = T(b[1]);
		_v[2] = T(b[2]);
		_v[3] = T(b[3]);
	}
	
  template <class Q> 
  static inline Point4 Construct( const Point4<Q> & b )
  {
    return Point4(T(b[0]),T(b[1]),T(b[2]),T(b[3]));
  }

/*数据访问*/
	inline const T & operator [] ( const int i ) const
	{
		assert(i>=0 && i<4);
		return _v[i];
	}
	inline T & operator [] ( const int i )
	{
		assert(i>=0 && i<4);
		return _v[i];
	}
	inline T &X() {return _v[0];}
	inline T &Y() {return _v[1];}
	inline T &Z() {return _v[2];}
	inline T &W() {return _v[3];}
	inline T const * V() const
	{
		return _v;
	}
	inline const T & V ( const int i ) const
	{
		assert(i>=0 && i<4);
		return _v[i];
	}
	inline T & V ( const int i )
	{
		assert(i>=0 && i<4);
		return _v[i];
	}
	
	inline T Ext( const int i ) const
	{
		if(i>=0 && i<=3) return _v[i];
		else             return 0;
	}

	inline Point4 operator + ( const Point4 & p) const
	{ 
		return Point4( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2], _v[3]+p._v[3] );
	}
	inline Point4 operator - ( const Point4 & p) const
	{
		return Point4( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2], _v[3]-p._v[3] );
	}
	inline Point4 operator * ( const T s ) const
	{
		return Point4( _v[0]*s, _v[1]*s, _v[2]*s, _v[3]*s );
	}
	inline Point4 operator / ( const T s ) const
	{
		return Point4( _v[0]/s, _v[1]/s, _v[2]/s, _v[3]/s );
	}
	inline Point4 & operator += ( const Point4 & p)
	{
		_v[0] += p._v[0]; _v[1] += p._v[1]; _v[2] += p._v[2]; _v[3] += p._v[3];
		return *this;
	}
	inline Point4 & operator -= ( const Point4 & p )
	{
		_v[0] -= p._v[0]; _v[1] -= p._v[1]; _v[2] -= p._v[2]; _v[3] -= p._v[3];
		return *this;
	}
	inline Point4 & operator *= ( const T s )
	{
		_v[0] *= s; _v[1] *= s; _v[2] *= s; _v[3] *= s;
		return *this;
	}
	inline Point4 & operator /= ( const T s )
	{
		_v[0] /= s; _v[1] /= s; _v[2] /= s; _v[3] /= s;
		return *this;
	}
	inline Point4 operator - () const
	{
		return Point4( -_v[0], -_v[1], -_v[2], -_v[3] );
	}
	inline Point4 VectProd ( const Point4 &x, const Point4 &z ) const
	{	
		Point4 res;
		const Point4 &y = *this;

		res[0] = y[1]*x[2]*z[3]-y[1]*x[3]*z[2]-x[1]*y[2]*z[3]+
		         x[1]*y[3]*z[2]+z[1]*y[2]*x[3]-z[1]*y[3]*x[2];
		res[1] = y[0]*x[3]*z[2]-z[0]*y[2]*x[3]-y[0]*x[2]*
		         z[3]+z[0]*y[3]*x[2]+x[0]*y[2]*z[3]-x[0]*y[3]*z[2];
		res[2] = -y[0]*z[1]*x[3]+x[0]*z[1]*y[3]+y[0]*x[1]*
  		         z[3]-x[0]*y[1]*z[3]-z[0]*x[1]*y[3]+z[0]*y[1]*x[3];
		res[3] = -z[0]*y[1]*x[2]-y[0]*x[1]*z[2]+x[0]*y[1]*
		         z[2]+y[0]*z[1]*x[2]-x[0]*z[1]*y[2]+z[0]*x[1]*y[2];
		return res;
	}

	inline T Norm() const
	{
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );
	}
	
	inline T SquaredNorm() const
	{
		return _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3];
	}
	
  inline Point4 & Normalize()
	{
		T n = sqrt(_v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );
		if(n>0.0) {	_v[0] /= n;	_v[1] /= n;	_v[2] /= n; _v[3] /= n; }
		return *this;
	}
	
	inline Point4 & HomoNormalize(){
		if (_v[3]!=0.0) {	_v[0] /= _v[3];	_v[1] /= _v[3];	_v[2] /= _v[3]; _v[3]=1.0; }
		return *this;
	};

/*比较操作*/
	inline bool operator == (  const Point4& p ) const
	{
		return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2] && _v[3]==p._v[3];
	} 
	inline bool operator != ( const Point4 & p ) const
	{
		return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2] || _v[3]!=p._v[3];
	}
	inline bool operator <  ( Point4 const & p ) const
	{
		return	(_v[3]!=p._v[3])?(_v[3]<p._v[3]):
				(_v[2]!=p._v[2])?(_v[2]<p._v[2]):
				(_v[1]!=p._v[1])?(_v[1]<p._v[1]):
				(_v[0]<p._v[0]);
	}
	inline bool operator >  ( const Point4 & p ) const
	{
		return	(_v[3]!=p._v[3])?(_v[3]>p._v[3]):
				(_v[2]!=p._v[2])?(_v[2]>p._v[2]):
				(_v[1]!=p._v[1])?(_v[1]>p._v[1]):
				(_v[0]>p._v[0]);
	}
	inline bool operator <= ( const Point4 & p ) const
	{
		return	(_v[3]!=p._v[3])?(_v[3]< p._v[3]):
				(_v[2]!=p._v[2])?(_v[2]< p._v[2]):
				(_v[1]!=p._v[1])?(_v[1]< p._v[1]):
				(_v[0]<=p._v[0]);
	}
	inline bool operator >= ( const Point4 & p ) const
	{
		return	(_v[3]!=p._v[3])?(_v[3]> p._v[3]):
				(_v[2]!=p._v[2])?(_v[2]> p._v[2]):
				(_v[1]!=p._v[1])?(_v[1]> p._v[1]):
				(_v[0]>=p._v[0]);
	}

	inline T operator * ( const Point4 & p ) const
	{
		return _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2] + _v[3]*p._v[3];
	} 
	inline Point4 operator ^ (  const Point4& p ) const
	{
		assert(0);
		return Point4();
	}


	T StableDot ( const Point4<T> & p ) const
	{

		T k0=_v[0]*p._v[0],	k1=_v[1]*p._v[1], k2=_v[2]*p._v[2], k3=_v[3]*p._v[3];
		int exp0,exp1,exp2,exp3;

		frexp( double(k0), &exp0 );frexp( double(k1), &exp1 );
		frexp( double(k2), &exp2 );frexp( double(k3), &exp3 );

		if (exp0>exp1) { math::Swap(k0,k1); math::Swap(exp0,exp1); }
		if (exp2>exp3) { math::Swap(k2,k3); math::Swap(exp2,exp3); }
		if (exp0>exp2) { math::Swap(k0,k2); math::Swap(exp0,exp2); }
		if (exp1>exp3) { math::Swap(k1,k3); math::Swap(exp1,exp3); }
		if (exp2>exp3) { math::Swap(k2,k3); math::Swap(exp2,exp3); }

		return ( (k0 + k1) + k2 ) +k3;
	}  
}; 



template <class T>
T Angle( const Point4<T>& p1, const Point4<T>  & p2 )
{
	T w = p1.Norm()*p2.Norm();
	if(w==0) return -1;
	T t = (p1*p2)/w;
	if(t>1) t=1;
	return T( math::Acos(t) );
}

template <class T>
inline T Norm( const Point4<T> & p )
{
	return p.Norm();
}

template <class T>
inline T SquaredNorm( const Point4<T> & p )
{
    return p.SquaredNorm();
}

template <class T>
inline T Distance( const Point4<T> & p1, const Point4<T> & p2 )
{
    return Norm(p1-p2);
}

template <class T>
inline T SquaredDistance( const Point4<T> & p1, const Point4<T> & p2 )
{
    return SquaredNorm(p1-p2);
}

	
template<class T>
double StableDot ( Point4<T> const & p0, Point4<T> const & p1 )
{
	return p0.StableDot(p1);
}  

typedef Point4<short>  Point4s;
typedef Point4<int>	   Point4i;
typedef Point4<float>  Point4f;
typedef Point4<double> Point4d;
}
#endif
