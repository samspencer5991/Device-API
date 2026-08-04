# ESP-IDF component wrapper for the Device-API library.
# PlatformIO/Arduino and STM32Cube builds ignore this file (they use library.json /
# their own build systems); ESP-IDF uses it to register Device-API as a component.
#
# Only the portable C core and the utility (SysEx/CDC serialisation) are built here.
# device_api.cpp is the Arduino-framework variant and is intentionally excluded — the
# ESP-IDF implementation lives in the ESP_PLATFORM branch of device_api.c.
idf_component_register(
    SRCS "src/device_api.c"
         "src/device_api_utility.cpp"
    INCLUDE_DIRS "src"
)
