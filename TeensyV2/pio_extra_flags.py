Import("env")

# Suppress upstream Teensy core C++ warnings without affecting C compilation.
env.Append(CXXFLAGS=["-Wno-deprecated-copy"])
