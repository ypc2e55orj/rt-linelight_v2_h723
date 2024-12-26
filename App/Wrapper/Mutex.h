#ifndef WRAPPER_MUTEX_H_
#define WRAPPER_MUTEX_H_

/* FreeRTOS */
#include <FreeRTOS.h>
#include <semphr.h>

class Mutex {
 public:
  /* コンストラクタ・デストラクタ */
  Mutex() noexcept;
  ~Mutex();

  Mutex(const Mutex &) = delete;
  Mutex &operator=(const Mutex &) = delete;

  void lock();
  bool try_lock();
  void unlock();

  typedef SemaphoreHandle_t native_handle_type;
  native_handle_type native_handle();

 private:
  StaticSemaphore_t mutexBuf_;
  SemaphoreHandle_t mutex_;
};

#endif  // WRAPPER_MUTEX_H_
