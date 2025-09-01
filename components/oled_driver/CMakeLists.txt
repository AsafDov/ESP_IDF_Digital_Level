# CMakeLists.txt for the oled_driver component

idf_component_register(SRCS "oled_driver.c"               # List of source files to compile
                       INCLUDE_DIRS "include"              # The directory for public header files
                       REQUIRES driver freertos)           # List of other components this driver depends on