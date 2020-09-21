
# Gateway to DDS Networks
A gateway for bridging between iceoryx systems and eth networks.
The gateway enables iceoryx systems running on separate network connected nodes to communicate with each other.

i.e. Data published by a publisher on `Node A` can be received by a matching subscriber on `Node B`.

# Organization
This module exports the following executables:
* `iox-gw-iceoryx2eth`
* `iox-gw-eth2iceoryx`

## Pre-requisites
* CMake is installed

## Scripted Build
The easiest way to build the gateway is via the script `iceoryx/tools/iceoryx_build_test.sh`.

To build, simply run:
```bash
iceoryx/tools/iceoryx_build_test.sh release with-eth-gateway
```

You may want to specify the build directory, this can be done via a flag. e.g.
```bash
iceoryx/tools/iceoryx_build_test.sh --builddir ./my-build release with-dds-gateway
```

Once complete, the gateway binaries can be found in `./my-build/install/prefix/bin`.

## CMake Build
Alternatively, you may like to manually run the build via CMake. This option is useful especially during development.

First, all of the gateway dependencies must be fetched, built, and installed into a common location.  The install location shall be referred to as `$INSTALL_DIR` from hereon.

This can be done manually, in which case the following dependencies must be installed to `$INSTALL_DIR`:
* cpptoml
* gtest (if testing)
* cyclonedds
* iceoryx_utils
* iceoryx_posh

A potentially easier method is, again, to take advantage of the script `iceoryx/tools/iceoryx_build_test.sh`.

```
# Running
Before running, you may need to add the install directory to the library load path if it is not standard (so that the runtime dependencies can be found).
i.e.
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$INSTALL_DIR/lib
```

Then, simply run the gateway executables as desired.

e.g.
```bash
$INSTALL_DIR/bin/iox-gw-iceoryx2eth
$INSTALL_DIR/bin/iox-gw-eth2iceoryx
```
