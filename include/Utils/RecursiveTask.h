#pragma once

#include "Utils/Naming.h"

#include <functional>
#include <omp.h>
#include <vector>

// Recursive function to spawn tasks and calculate results using a binary tree pattern
template<typename T, typename Obj, typename MemFn>
void
recursiveTask(std::vector<T>& res, size_t start, size_t end, Obj* obj, MemFn func, int depth)
{
  if (depth == 0) {
    // Base case: compute directly if there is only one element or no elements
    (obj->*func)(res, start, end);
  } else {
    std::vector<T> left_result;
    std::vector<T> right_result;

#pragma omp task shared(left_result)
    recursiveTask(left_result, start, (start + end) / 2, obj, func, depth - 1);

#pragma omp task shared(right_result)
    recursiveTask(right_result, (start + end) / 2, end, obj, func, depth - 1);

#pragma omp taskwait
    res.reserve(left_result.size() + right_result.size());
    res.insert(res.end(), left_result.begin(), left_result.end());
    res.insert(res.end(), right_result.begin(), right_result.end());
  }
}