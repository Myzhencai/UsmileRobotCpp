#ifndef _POINT3_
#define _POINT3_
#include<assert.h>
#include<math.h>



	template <class P3ScalarType> 
	class Point3
	{
	public:

		typedef P3ScalarType ScalarType;


		//////////////////////////////////// 构造函数  /////////////////////////////////////////////

		inline Point3 () {}
		inline Point3 ( const P3ScalarType nx, const P3ScalarType ny, const P3ScalarType nz )
		{
			_v[0] = nx;
			_v[1] = ny;
			_v[2] = nz;
		}
		inline Point3 ( Point3 const & p )
		{   
			_v[0]= p._v[0];
			_v[1]= p._v[1];
			_v[2]= p._v[2];
		}
		inline Point3 ( const P3ScalarType nv[3] )
		{
			_v[0] = nv[0];
			_v[1] = nv[1];
			_v[2] = nv[2];
		}
		inline Point3 & operator =( Point3 const & p )
		{
			_v[0]= p._v[0]; _v[1]= p._v[1]; _v[2]= p._v[2];
			return *this;
		}
		inline void Zero()
		{
			_v[0] = 0;
			_v[1] = 0;
			_v[2] = 0;
		}

		template <class Q>
		inline void Import( const Point3<Q> & b )
		{
			_v[0] = P3ScalarType(b[0]);
			_v[1] = P3ScalarType(b[1]);
			_v[2] = P3ScalarType(b[2]);
		}


		template <class Q> 

		static inline Point3 Construct( const Point3<Q> & b )
		{
			return Point3(P3ScalarType(b[0]),P3ScalarType(b[1]),P3ScalarType(b[2]));
		}


		template <class Q> 

		static inline Point3 Construct( const Q & P0, const Q & P1, const Q & P2)
		{
			return Point3(P3ScalarType(P0),P3ScalarType(P1),P3ScalarType(P2));
		}


		static inline Point3 Construct( const Point3<ScalarType> & b )
		{
			return b;
		}

		///////////////////////////////////    数据操作   ////////////////////////////////////////////////

		inline P3ScalarType & operator [] ( const int i )
		{assert(i>=0 && i<3);return _v[i];}

		inline const P3ScalarType & operator [] ( const int i ) const
		{assert(i>=0 && i<3);return _v[i];}

		inline const P3ScalarType &X() const { return _v[0]; } 

		inline const P3ScalarType &Y() const { return _v[1]; }

		inline const P3ScalarType &Z() const { return _v[2]; }

		inline P3ScalarType &X() { return _v[0]; }

		inline P3ScalarType &Y() { return _v[1]; }

		inline P3ScalarType &Z() { return _v[2]; }

		inline const P3ScalarType * V() const
		{return _v;}

		inline P3ScalarType * V()
		{return _v;}


		inline P3ScalarType & V( const int i )
		{assert(i>=0 && i<3);return _v[i];}


		inline const P3ScalarType & V( const int i ) const
		{assert(i>=0 && i<3);return _v[i];}

		//////////////////////////////////  数学操作   ///////////////////////////////////////////

		inline Point3 operator + ( Point3 const & p) const
		{return Point3<P3ScalarType>( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2] );}

		inline Point3 operator - ( Point3 const & p) const
		{return Point3<P3ScalarType>( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2] );}

		inline Point3 operator * ( const P3ScalarType s ) const
		{return Point3<P3ScalarType>( _v[0]*s, _v[1]*s, _v[2]*s );}

		inline Point3 operator / ( const P3ScalarType s ) const
		{return Point3<P3ScalarType>( _v[0]/s, _v[1]/s, _v[2]/s );}

		inline P3ScalarType operator * ( Point3 const & p ) const
		{return ( _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2] );}

		void Set(const ScalarType x,const ScalarType y,const ScalarType z)
		{_v[0]=x;_v[1]=y;_v[2]=z;}

		Point3 Cross(const Point3& v) const
		{
			return Point3(Y() * v.Z() - Z() * v.Y(),
				Z() * v.X() - X() * v.Z(),
				X() * v.Y() - Y() * v.X());
		}

		inline Point3 operator ^ ( Point3 const & p ) const
		{
			return Point3 <P3ScalarType>
				(
				_v[1]*p._v[2] - _v[2]*p._v[1],
				_v[2]*p._v[0] - _v[0]*p._v[2],
				_v[0]*p._v[1] - _v[1]*p._v[0]
			);
		}

		inline Point3 & operator += ( Point3 const & p)
		{
			_v[0] += p._v[0];
			_v[1] += p._v[1];
			_v[2] += p._v[2];
			return *this;
		}
		inline Point3 & operator -= ( Point3 const & p)
		{
			_v[0] -= p._v[0];
			_v[1] -= p._v[1];
			_v[2] -= p._v[2];
			return *this;
		}
		inline Point3 & operator *= ( const P3ScalarType s )
		{
			_v[0] *= s;
			_v[1] *= s;
			_v[2] *= s;
			return *this;
		}
		inline Point3 & operator /= ( const P3ScalarType s )
		{
			_v[0] /= s;
			_v[1] /= s;
			_v[2] /= s;
			return *this;
		}

		inline P3ScalarType Norm() const
		{return sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] );}

		inline P3ScalarType SquaredNorm() const
		{return ( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] );}

		inline Point3 & Scale( const P3ScalarType sx, const P3ScalarType sy, const P3ScalarType sz )
		{
			_v[0] *= sx;
			_v[1] *= sy;
			_v[2] *= sz;
			return *this;
		}

		inline Point3 & Scale( const Point3 & p )
		{
			_v[0] *= p._v[0];
			_v[1] *= p._v[1];
			_v[2] *= p._v[2];
			return *this;
		}

		inline Point3 & Normalize()
		{
			P3ScalarType n = sqrt(_v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]);
			if(n>0) 
			{	
				n = 1/n;
				_v[0] *= n;	_v[1] *= n;	_v[2] *= n;  
			}
			return *this;
		}

		void ToPolar(P3ScalarType &ro, P3ScalarType &theta, P3ScalarType &phi) const
		{
			ro = Norm();
			theta = (P3ScalarType)atan2(_v[2], _v[0]);
			phi   = (P3ScalarType)asin(_v[1]/ro);
		}

		void FromPolar(const P3ScalarType &ro, const P3ScalarType &theta, const P3ScalarType &phi)
		{
			_v[0]= ro*cos(theta)*cos(phi);
			_v[1]= ro*sin(phi);
			_v[2]= ro*sin(theta)*cos(phi);
		}

		inline bool operator == ( Point3 const & p ) const
		{return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2];}

		inline bool operator != ( Point3 const & p ) const
		{return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2];}

		inline bool operator <  ( Point3 const & p ) const
		{ return	(_v[2]!=p._v[2])?(_v[2]<p._v[2]):
		(_v[1]!=p._v[1])?(_v[1]<p._v[1]):
		(_v[0]<p._v[0]);}

		inline bool operator >  ( Point3 const & p ) const
		{ return	(_v[2]!=p._v[2])?(_v[2]>p._v[2]):
		(_v[1]!=p._v[1])?(_v[1]>p._v[1]):
		(_v[0]>p._v[0]);}

		inline bool operator <= ( Point3 const & p ) const
		{ return	(_v[2]!=p._v[2])?(_v[2]< p._v[2]):
		(_v[1]!=p._v[1])?(_v[1]< p._v[1]):
		(_v[0]<=p._v[0]);}

		inline bool operator >= ( Point3 const & p ) const
		{ return	(_v[2]!=p._v[2])?(_v[2]> p._v[2]):
		(_v[1]!=p._v[1])?(_v[1]> p._v[1]):
		(_v[0]>=p._v[0]);}

		inline Point3 operator - () const
		{return Point3<P3ScalarType> ( -_v[0], -_v[1], -_v[2] );}


	public:
		union {
			struct { P3ScalarType x, y, z; };
			struct { P3ScalarType _v[3]; };
		};
	protected:
		enum {Dimension = 3};

	};


	///////////////////////////////////////  常用函数   //////////////////////////////////////////////////////////


	template <class P3ScalarType>
	inline P3ScalarType Angle( Point3<P3ScalarType> const & p1, Point3<P3ScalarType> const & p2 )
	{
		P3ScalarType w = p1.Norm()*p2.Norm();
		if(w==0) return -1;
		P3ScalarType t = (p1*p2)/w;
		if(t>1) t = 1;
		else if(t<-1) t = -1;
		return (P3ScalarType) acos(t);
	}

	template <class P3ScalarType>
	inline P3ScalarType AngleN( Point3<P3ScalarType> const & p1, Point3<P3ScalarType> const & p2 )
	{
		P3ScalarType w = p1*p2;
		if(w>1) 
			w = 1;
		else if(w<-1) 
			w=-1;
		return (P3ScalarType) acos(w);
	}

	template <class P3ScalarType>
	inline P3ScalarType Norm( Point3<P3ScalarType> const & p )
	{
		return p.Norm();
	}

	template <class P3ScalarType>
	inline P3ScalarType SquaredNorm( Point3<P3ScalarType> const & p )
	{
		return p.SquaredNorm();
	}

	template <class P3ScalarType>
	inline Point3<P3ScalarType> & Normalize( Point3<P3ScalarType> & p )
	{
		p.Normalize();
		return p;
	}

	template <class P3ScalarType>
	inline P3ScalarType Distance( Point3<P3ScalarType> const & p1,Point3<P3ScalarType> const & p2 )
	{
		return (p1-p2).Norm();
	}

	template <class P3ScalarType>
	inline P3ScalarType SquaredDistance( Point3<P3ScalarType> const & p1,Point3<P3ScalarType> const & p2 )
	{
		return (p1-p2).SquaredNorm();
	}

	typedef Point3<short>  Point3s;
	typedef Point3<int>	   Point3i;
	typedef Point3<float>  Point3f;
	typedef Point3<double> Point3d;

#endif

