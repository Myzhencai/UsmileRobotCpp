#ifndef _SINGLETON_CLASS_HEADER_
#define _SINGLETON_CLASS_HEADER_

class SINGLETON {
 public:
  /// for singleton implementation
  static SINGLETON* Instance();

 protected:
  /// for singleton implementation
  static SINGLETON* pinstance;
  /// for singleton implementation
  SINGLETON();
  /// for singleton implementation
  SINGLETON(const SINGLETON&);
  /// for singleton implementation
  SINGLETON& operator=(const SINGLETON&);
  ~SINGLETON();
};

#endif