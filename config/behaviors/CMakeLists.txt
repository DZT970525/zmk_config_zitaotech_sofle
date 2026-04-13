# Copyright (c) 2025 ZitaoTech
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources(scroll_accelerator.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)
