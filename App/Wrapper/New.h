#ifndef WRAPPER_NEW_H_
#define WRAPPER_NEW_H_

/* C++ */
#include <new>

void *operator new(std::size_t);
void *operator new(std::size_t, const std::nothrow_t &) noexcept;
void *operator new[](std::size_t);
void *operator new[](std::size_t, const std::nothrow_t &) noexcept;

void operator delete(void *ptr);
void operator delete(void *ptr, const std::nothrow_t &) noexcept;
void operator delete[](void *ptr);
void operator delete[](void *ptr, const std::nothrow_t &) noexcept;
void operator delete(void *, std::size_t);
void operator delete(void *, std::size_t, const std::nothrow_t &) noexcept;
void operator delete[](void *, std::size_t);
void operator delete[](void *, std::size_t, const std::nothrow_t &) noexcept;

#endif  // WRAPPER_NEW_H_
