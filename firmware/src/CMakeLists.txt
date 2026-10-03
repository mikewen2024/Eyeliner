# Main component. Sources are listed explicitly so src/tests/ (Arduino-based
# hardware tests with their own PlatformIO envs) never ends up in this build.
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "../include")
