/*
 * LibraryHacks.cpp
 *
 *  Created on: 2014¦~8¤ë4¤é
 *      Author: YunKei
 */

/*
 * LibraryHacks.cpp
 *
 *  Created on: 23 Jan 2011
 *      Author: Andy
 */

#include <cstdlib>
#include <sys/types.h>
/*
 * The default pulls in 70K of garbage
 */

namespace __gnu_cxx {

  void __verbose_terminate_handler() {
    for(;;);
  }
}


/*
 * The default pulls in about 12K of garbage
 */

extern "C" void __cxa_pure_virtual() {
  for(;;);
}


/*
 * Implement C++ new/delete operators using the heap
 */

void *operator new(size_t size) {
  return malloc(size);
}

void *operator new[](size_t size) {
  return malloc(size);
}

void operator delete(void *p) {
  free(p);
}

void operator delete[](void *p) {
  free(p);
}



