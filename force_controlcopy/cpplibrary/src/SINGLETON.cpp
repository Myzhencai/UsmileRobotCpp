#include "SINGLETON.h"

SINGLETON* SINGLETON::pinstance = 0;

SINGLETON* SINGLETON::Instance() {
  if (pinstance == 0)  // first time call
  {
    pinstance = new SINGLETON;
  }
  return pinstance;
}

SINGLETON::SINGLETON() {}

SINGLETON& SINGLETON::operator=(const SINGLETON& olc) {
  // shall never get called
  SINGLETON* lc = new SINGLETON();
  return *lc;
}

SINGLETON::~SINGLETON() { delete pinstance; }