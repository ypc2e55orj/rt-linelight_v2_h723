#include "Wrapper/New.h"

/* FreeRTOS */
#include <FreeRTOS.h>

/* C++ */
#include <exception>

static void *operator_new_impl(std::size_t size) {
  void *p = nullptr;

  if (size == 0) {
    size = 1;
  }
  while ((p = pvPortMalloc(size)) == nullptr) {
    std::new_handler handler = std::get_new_handler();
    if (handler) {
      handler();
    } else {
      break;
    }
  }
  return p;
}
static void operator_delete_impl(void *ptr) {
  if (ptr != nullptr) {
    vPortFree(ptr);
  }
}

void *operator new(std::size_t size) {
  void *p = operator_new_impl(size);
  // if (p == nullptr) {
  //   throw std::bad_alloc();
  // }
  return p;
}
void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
  // try {
    return ::operator new(size);
  // } catch (...) {
  //   return nullptr;
  // }
}
void *operator new[](std::size_t size) { return ::operator new(size); }
void *operator new[](std::size_t size, const std::nothrow_t &) noexcept {
  // try {
    return ::operator new[](size);
  // } catch (...) {
  //   return nullptr;
  // }
}

void operator delete(void *ptr) { operator_delete_impl(ptr); }
void operator delete(void *ptr, const std::nothrow_t &) noexcept { ::operator delete(ptr); }
void operator delete[](void *ptr) { ::operator delete(ptr); }
void operator delete[](void *ptr, const std::nothrow_t &) noexcept { ::operator delete[](ptr); }
void operator delete(void *ptr, std::size_t) { ::operator delete(ptr); }
void operator delete(void *ptr, std::size_t, const std::nothrow_t &) noexcept { ::operator delete(ptr); }
void operator delete[](void *ptr, std::size_t) { ::operator delete[](ptr); }
void operator delete[](void *ptr, std::size_t, const std::nothrow_t &) noexcept { ::operator delete[](ptr); }
