# OpenTitan

![OpenTitan logo](https://docs.opentitan.org/doc/opentitan-logo.png)

## About the project

[OpenTitan](https://opentitan.org) is an open source silicon Root of Trust
(RoT) project.  OpenTitan will make the silicon RoT design and implementation
more transparent, trustworthy, and secure for enterprises, platform providers,
and chip manufacturers.  OpenTitan is administered by [lowRISC
CIC](https://www.lowrisc.org) as a collaborative project to produce high
quality, open IP for instantiation as a full-featured product. See the
[OpenTitan site](https://opentitan.org/) and [OpenTitan
docs](https://docs.opentitan.org) for more information about the project.

## About this repository

This repository contains hardware, software and utilities written as part of the
OpenTitan project. It is structured as monolithic repository, or "monorepo",
where all components live in one repository. It exists to enable collaboration
across partners participating in the OpenTitan project.

## Documentation

The project contains comprehensive documentation of all IPs and tools. You can
access it [online at docs.opentitan.org](https://docs.opentitan.org/).

## Using the repository
The repository is provided with a top-level wrapper of the Earlgrey architecture, whichc an be found at "opentitan/hw/top_earlgrey/top/secure_subsystem_asynch_synth_wrap.sv". A testbench can be found at "opentitan/hw/tb/testbench_asynch.sv".

The repository is automatically configured when running a test, so to configure and run the repo:
```
git clone -b workshop git@github.com/pulp-platform/opentitan.git
cd opentitan/
make clean sim SRAM=sw/tests/opentitan/idma_test/idma_test.elf
```
After cloning the repo it is possible to just run the test, the scripts will donwload and configure the Bender deps and install all the verification IPs, before running the test.

In general, to execute an arbitrary test, provide the path to the binary with the "SRAM=" flag.

It is possible to run the secure boot (both preloading via JTAG or via SPI the embedded emulated flash) with:

```
make secure_boot_spi

make secure_boot_jtag
```

These tests are very slow, requiring about one hour to run in the stand alone repository.