#include "queue.h"
#include <iostream>


template <typename T> void Queue<T>::enqueue(const T &value) {
  elements.push_back(value);
}

template <typename T> bool Queue<T>::empty() { return elements.empty(); }

template <typename T> void Queue<T>::dequeue() {
  if (!empty()) {
    elements.erase(elements.begin());
  }
}

template <typename T> const T& Queue<T>::front() const {
  if (!empty()) {
    return elements.front();
  }
}

template<typename  T> void Queue<T>::printQueue() const {
  for (const T& element: elements) {
    std::cout << element << "";
  }
  std::cout << "\n";
}