#include <cstring>

#include "Check.h"

namespace check {
std::vector<Test>& registry() {
  static std::vector<Test> tests;
  return tests;
}
int& failures() {
  static int count = 0;
  return count;
}
}  // namespace check

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int run = 0;
  for (const auto& t : check::registry()) {
    if (filter && !std::strstr(t.name, filter)) continue;
    const int before = check::failures();
    std::printf("[ RUN  ] %s\n", t.name);
    t.fn();
    std::printf("[ %s ] %s\n", check::failures() == before ? " OK " : "FAIL", t.name);
    ++run;
  }
  std::printf("%d tests, %d failed checks\n", run, check::failures());
  return check::failures() == 0 ? 0 : 1;
}
