if(CONFIG_BOARD_ZITAOTECH_SOFLE_LEFT)
zephyr_library()
zephyr_library_sources(custom_driver_left/keyboard_backlight.c)
zephyr_library_sources(../../../behaviors/scroll_accelerator.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_library_include_directories(../../../include)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
endif()


if(CONFIG_BOARD_ZITAOTECH_SOFLE_RIGHT)
  zephyr_library()
  zephyr_library_sources(custom_driver_right/keyboard_backlight.c)
  zephyr_library_sources(../../../behaviors/scroll_accelerator.c)
  zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
  zephyr_library_include_directories(../../../include)
  zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
endif()

