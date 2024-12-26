#ifndef DATA_SINGLETON_H_
#define DATA_SINGLETON_H_

template <typename T>
class Singleton {
 public:
  template <typename... Args>
  static T &Instance(Args... args) {
    static T this_(args...);
    return this_;
  }

 protected:
  Singleton() = default;
  virtual ~Singleton() = default;

 private:
  /* コピー禁止 */
  Singleton(const Singleton &) = delete;
  /* コピー代入禁止 */
  Singleton &operator=(const Singleton &) = delete;

  /* ムーブ禁止 */
  Singleton(Singleton &&) = delete;
  /* ムーブ代入禁止 */
  Singleton &operator=(Singleton &&) = delete;
};

#endif  // DATA_SINGLETON_H_
