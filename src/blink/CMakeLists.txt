add_executable(blink blink.c)
target_link_libraries(blink PRIVATE common)
pico_add_extra_outputs(blink)
pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 0)