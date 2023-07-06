# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

load("//rules:opentitan.bzl", "OPENTITAN_CPU", "opentitan_ram_binary")
load("//rules:exclude_files.bzl", "exclude_files")
load("//rules:linker.bzl", "ld_library")
load(
    "//rules:opentitan_test.bzl",
    "manual_test",
    "opentitan_functest",
)
load("//rules:cross_platform.bzl", "dual_cc_library", "dual_inputs")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files")

package(default_visibility = ["//visibility:public"])

opentitan_ram_binary(
    name = "spi_host_test",
    srcs = [
    	"spi_host_test.c",
    ],
    hdrs = [
        "spi_host_test.h",
    ],
    archive_symbol_prefix = "spi_host_test",
    testonly = True,
    deps = [
        "//sw/tests/opentitan/common:common",
        "//sw/tests/opentitan/common:linker",
        "//sw/tests/opentitan/common:crt",
	"//sw/device/silicon_creator/rom:rom_common",
        "//hw/top_earlgrey/sw/autogen:top_earlgrey",
        "//sw/device/lib/arch:device",
        "//sw/device/lib/base:memory",
        "//sw/device/lib/base:mmio",
        "//sw/device/lib/dif:spi_host",
        "//sw/device/lib/runtime:hart",
        "//sw/device/lib/runtime:log",
        "//sw/device/lib/runtime:print",
        "//sw/device/lib/testing/test_framework:ottf_main",
     ],
)