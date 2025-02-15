#include <SDL3/SDL.h>
#include <vector>

template <typename T> class Queue {
private:
  std::vector<T> elements;

public:
  void enqueue(const T &value);

  bool empty();

  void dequeue();
  const T &front() const;
  void printQueue() const;
};