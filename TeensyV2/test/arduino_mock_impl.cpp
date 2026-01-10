// Intentionally include the mock implementation as part of the test build.
// PlatformIO's native test runner compiles test/*.cpp but may not recurse
// into test/mocks/*.cpp; this guarantees the symbols are linked.

#include "mocks/Arduino.cpp"
