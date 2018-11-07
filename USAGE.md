# Usage Cheat Sheet

## Prerequirements

This describes how to get things working on Linux.

 - Rust beta 1.31
 - The thumbv6m-none-eabi target
 - GDB for ARM
 - OpenOCD

### Install Rust

Install rust through https://rustup.rs/

Then install the beta channel and thumbv6m-none-eabi

```
$ rustup install beta
$ rustup default beta
$ rustup component add rust-std --target thumbv6m-none-eabi
$ rustup default stable
```

### Install OpenOCD

#### Ubuntu

```
# apt install openocd
```

#### Fedora

```
# dnf install openocd
```

### Install GDB

#### Ubuntu

```
# apt install gdb-multiarch
```

#### Fedora

```
# dnf install arm-none-eabi-gdb
```

## Build

```
$ cargo +beta build --examples
```

## Run and Debug

Connect the board to you computer.

Start OpenOCD in the checked out directory.

```
$ openocd
```

Start a new terminal and run the program,

```
$ cargo +beta run --example ble-hal
```
