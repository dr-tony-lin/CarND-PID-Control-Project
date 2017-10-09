#ifndef _UTILS_CIRCULARBUFFER_H_
#define _UTILS_CIRCULARBUFFER_H_
#include <deque>

using namespace std;

/**
 * Reducer reduces a sequence of numbers (short, int, float, double). 
 * Reduce functions include min, max, sum, weighted,and unweighted means
 */ 
template<typename T> class Reducer {
  int limit;
  deque<T> queue; 
public:
  Reducer(int size_limit): limit(size_limit) {};

  void push(const T &v) {
    queue.push_back(v);
    if (queue.size() > limit) {
      queue.pop_front();
    }
  };

  int size() { return queue.size();}

  T &operator[](int index) {
    if (index >= 0 && index < queue.size()) {
      return queue[index];
    }
    throw "Indxex out of bound";
  }

  /**
   * Return max
   */
  T &max() {
    if (queue.size() > 0) {
      T max = queue[0];
      for (int i = 1; i < queue.size(); i++) {
        T &a = queue[i];
        if (max < a) {
          max = a;
        }
      }
      return max;
    }
    else {
      return 0;
    }
  }

  /**
   * Return min
   */
  T &min() {
    if (queue.size() > 0) {
      T min = queue[0];
      for (int i = 1; i < queue.size(); i++) {
        T &a = queue[i];
        if (min > a) {
          min = a;
        }
      }
      return min;
    }
    else {
      return 0;
    }
  }

  /**
   * Return weighted sum
   * @param weights the weights
   */
  template<typename V> V mean(T weights[]) {
    V total = 0;
    V total_w = 0;
    if (queue.size()) {
      for (int i = 0; i < queue.size(); i++) {
        total += V(weights[i] * queue[i]);
        total_w += V(weights[i]);
      }
      return total/total_w;
    }
    return 0;
  }

  /**
   * Return weighted sum
   * @param weights the weights iterator
   */
  template<template <typename> class C, typename V> V mean(typename C<T>::iterator weights) {
    V total = 0;
    V total_w = 0;
    if (queue.size()) {
      for (int i = 0; i < queue.size(); i++, weights++) {
        total += V(*weights * queue[i]);
        total_w += V(*weights);
      }
      return total/total_w;
    }
    return 0;
  }

  /**
   * Return unweighted mean
   */
  template<typename V> V  mean() {
    V total = 0;
    if (queue.size()) {
      for (int i = 0; i < queue.size(); i++) {
        total += queue[i];
      }
      return total/queue.size();
    }
    return 0;
  }

  /**
   * Return sum
   */ 
  T sum() {
    T total = 0;
    if (queue.size()) {
      for (int i = 0; i < queue.size(); i++) {
          total += queue[i];
      }
      return total;
    }
    return 0;
  }
};
#endif