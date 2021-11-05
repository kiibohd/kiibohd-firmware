# Kiibohd Firmware

Rust successor to the [kiibohd-controller](https://github.com/kiibohd/controller) firmware.

Each keyboard has it's own directory.

* [common](common) - Common files used in all projects.
* [hexgears](hexgears) - Hexgears device firmware.
* [inputclub](inputclub) - Input Club device firmware.


## Environment Setup

To build you'll need the following tools installed:
```bash
rustup install nightly
rustup default nightly
rustup target add thumbv7em-none-eabi
rustup component add llvm-tools-preview
cargo install cargo-binutils cargo-make flip-link
```

It is recommended to default to the nightly compile as there are many experimental features used in dependencies.
You can also add +nightly instead (e.g. `cargo +nightly build`).


## Top-Level Build Commands

These commands are useful when testing changes across multiple devices.

**NOTE**: These devices generally require +nightly to build correctly if not enabled by default.

```bash
cargo make check
cargo make fmt
cargo make fmt-check
cargo make clippy
cargo make doc

# Runs check, fmt-check, doc and clippy
cargo make sanity

# Builds debug binaries for all crates
cargo make build

# Builds release .dfu.bin for all crates
cargo make bin-dfu

# Flashes binary using dfu-util (if supported by keyboard)
cargo make flash
```

The commands are used by GitHub Actions to verify changes.
Don't try to use `cargo build` at the top-level as each device (keyboard) is compiled for a specific arch target.


## Debugging

You can run binaries directly from cargo (provided you have the necessary debugging cable: TODO Link).

```bash
cd hexgears/gemini # Or any other device
cargo make run
```

### GDB

To use gdb you'll need a few things:

- `cargo install defmt-print`
- `cargo install cargo-binutils`
- openocd >= 0.11.0 (needed for RTT support)
- nc (netcat)
- Comment out this section in `Cargo.toml` (otherwise gdb isn't as useful).
```toml
[profile.dev]
codegen-units = 1
debug = 2
debug-assertions = true # <-
incremental = false
opt-level = 3 # <-
overflow-checks = true # <-
```

To start openocd GDB server (this will also rebuild firmware).
```bash
cd hexgears/gemini # Or any other device
cargo make gdb-server
```

Then to load up a gdb client (from the cli):
```bash
cd hexgears/gemini # Same device as you are trying to debug
cargo make gdb-client
```
You can also connect using another gdb client using localhost:3333

A useful command to run in gdb is:
```
monitor soft_reset_halt
```
which soft reset the processor and halt at the beginning so you can load any breakpoints you want before continuing.

### Debugging in CLion
As an alternative to running GDB/OpenOCD directly you can use the following steps to configure debug within CLion.

1. Run through the previous steps to get the gdb-server and gdb-client working including installing all dependencies. 
2. Create an OpenOCD configuration file containing the following: 
```yaml
source [find interface/cmsis-dap.cfg]
transport select swd
source [find target/at91sam4sXX.cfg]
adapter speed 30000
```
3. Within CLion open the Run/Debug Configurations dialog and add a new ```OpenOCD Download & Run``` configuration.
4. For ```Target``` create a dummy build target. 
5. Set ```Executable``` to the debug gemini executable found in ```kiibohd-firmware/target/thumbv7em-none-eabi/debug/gemini```.
6. Leave GDB set to use ```Bundled GDB multiarch```.
7. Set the Board config file to point to the OpenOCD config file you previously created.
8. Leave the fields GDB Port (3333), Telnet Port (4444), Download (Updated Only), and Reset (Init), values as they are.
9. In the Before launch field add a new entry for Run External tool, setting Program to ```cargo```,  Arguments to ```build --target thumbv7em-none-eabi``` and Working directory to the full path to the ```kiibohd-firmware/hexgears/gemini``` folder.
10. Once created use this and remove the existing Build target from the Before launch field as it is no longer needed.
11. Save your changes.
12. You should now have a Run/Debug Configuration that when you run Debug will allow you to debug within CLion. 

The above steps have been adapted from the following discussion:
* https://users.rust-lang.org/t/embedded-rust-how-can-one-debug-stm32-openocd-from-clion-intellij-rust/19062/4
 

## License

Licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.


### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
