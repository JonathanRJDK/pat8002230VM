# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pat8002230VM)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/atm90e36aaurReadAndWrite.c)
target_sources(app PRIVATE src/powerMeterFiltering.c)