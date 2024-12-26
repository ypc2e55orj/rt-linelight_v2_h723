#include "Wrapper/Mutex.h"

Mutex::Mutex() noexcept : mutex_(xSemaphoreCreateMutexStatic(&mutexBuf_)) {}

Mutex::~Mutex() {}

void Mutex::lock() { xSemaphoreTake(mutex_, portMAX_DELAY); }

bool Mutex::try_lock() { return xSemaphoreTake(mutex_, 0) == pdTRUE; }

void Mutex::unlock() { xSemaphoreGive(mutex_); }

Mutex::native_handle_type Mutex::native_handle() { return mutex_; }
