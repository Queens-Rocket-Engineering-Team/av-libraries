import os
Import("env")

# Force PlatformIO to include the cross-compiler toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# Override and output the compilation database directly to the project root
env.Replace(COMPILATIONDB_PATH=os.path.join("$PROJECT_DIR", "compile_commands.json"))

