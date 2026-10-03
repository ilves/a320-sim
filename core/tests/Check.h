#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

// Minimal test harness: no third-party dependency for a handful of checks.
namespace check {

struct Test {
  const char* name;
  std::function<void()> fn;
};

std::vector<Test>& registry();
int& failures();

struct Register {
  Register(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

}  // namespace check

#define CHECK_CAT2(a, b) a##b
#define CHECK_CAT(a, b) CHECK_CAT2(a, b)
#define TEST(name)                                                              \
  static void name();                                                           \
  static check::Register CHECK_CAT(reg_, name)(#name, name);                    \
  static void name()

#define CHECK(cond)                                                             \
  do {                                                                          \
    if (!(cond)) {                                                              \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
      ++check::failures();                                                      \
    }                                                                           \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                   \
  do {                                                                          \
    const double va_ = (a), vb_ = (b);                                          \
    if (!(std::fabs(va_ - vb_) <= (tol))) {                                     \
      std::printf("  FAIL %s:%d: %s = %.4f, expected %.4f +- %.4f\n", __FILE__, \
                  __LINE__, #a, va_, vb_, static_cast<double>(tol));            \
      ++check::failures();                                                      \
    }                                                                           \
  } while (0)
