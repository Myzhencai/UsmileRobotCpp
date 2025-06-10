

#ifndef _MATH_BASE_
#define _MATH_BASE_
#include <math.h>
#include <assert.h>
#include <limits.h>
#include<float.h>

/// static_assert: implemented as a macro for "assert", but it is separated for clarity.
/// Should be used for checking integrity constraints that can be tested at complile time,
/// as the ones involving templated constants in templated classes.
#define static_assert assert

  #ifdef __BORLANDC__
    float sqrtf (float v) {return sqrt(v);}
    float fabsf (float v) {return fabs(v);}
    float cosf  (float v) {return cos(v);}
    float sinf  (float v) {return sin(v);}
    float acosf  (float v) {return acos(v);}
    float asinf  (float v) {return asin(v);}
    float atan2f (float v0, float v1) {return atan2(v0,v1);}
  #endif



namespace math {

 template <class SCALAR> 
 class MagnitudoComparer
    {
      public:
	    inline bool operator() ( const SCALAR a, const SCALAR b ) { return fabs(a)>fabs(b);  }
    };
  
  inline float Sqrt(const short v)   { return sqrtf(v); }
  inline float Sqrt(const int v)   { return sqrtf((float)v); }
  
  inline float Sqrt(const float v)   { return sqrtf(v); }
  inline float Abs(const float v)   { return fabsf(v); }
  inline float Cos(const float v)   { return cosf(v); }
  inline float Sin(const float v)   { return sinf(v); }
  inline float Acos(const float v)   { return acosf(v); }
  inline float Asin(const float v)   { return asinf(v); }
  inline float Atan2(const float v0,const float v1)   { return atan2f(v0,v1); }

  inline double Sqrt(const double v)   { return sqrt(v); }
  inline double Abs(const double v)   { return fabs(v); }
  inline double Cos(const double v)   { return cos(v); }
  inline double Sin(const double v)   { return sin(v); }
  inline double Acos(const double v)   { return acos(v); }
  inline double Asin(const double v)   { return asin(v); }
  inline double Atan2(const double v0,const double v1)   { return atan2(v0,v1); }
  
	template<class T> inline const T & Min(const T &a, const T &b){
		if (a<b) return a; else return b;
	}
	template<class T> inline const T & Max(const T &a, const T &b){
		if (a<b) return b; else return a;
	}

	template<class T> inline void Swap(T &a, T &b){
		T tmp=a; a=b; b=tmp;
	}
	template<class T> inline void Sort(T &a, T &b){
		if (a>b) Swap(a,b);
	}
	template<class T> inline void Sort(T &a, T &b, T &c){
		if (a>b) Swap(a,b);
		if (b>c) {Swap(b,c); if (a>b) Swap(a,b);}
	} 

/* Some <math.h> files do not define M_PI... */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template <class SCALAR> 
inline SCALAR  Clamp( const SCALAR & val, const SCALAR& minval, const SCALAR& maxval)
{
	if(val < minval) return minval;
	if(val > maxval) return maxval;
	return val;
}



inline float   ToDeg(const float &a){return a*180.0f/float(M_PI);}
inline float   ToRad(const float &a){return float(M_PI)*a/180.0f;}
inline double  ToDeg(const double &a){return a*180.0/M_PI;}
inline double  ToRad(const double &a){return M_PI*a/180.0;}

#if defined(_MSC_VER) // Microsoft Visual C++
	template<class T> int IsNAN(T t) {    return _isnan(t); }
#elif defined(__GNUC__) // GCC
	template<class T> int IsNAN(T t) {    return isnan(t); }
#else // generic

template<class T> int IsNAN(T t)
{
     if(std::numeric_limits<T>::has_infinity)
         return !(t <= std::numeric_limits<T>::infinity());
     else
         return t != t;
}

#endif 
}	// End math namespace

/// a type that stands for "void". Useful for Parameter type of a point.
class VoidType{ public:
	VoidType(){};
};




#endif
