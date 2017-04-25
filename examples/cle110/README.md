examples/default
================
This application is a showcase for RIOT's hardware support. Using it
for your board, you should be able to interactively use any hardware
that is supported.

To do this, the application uses the `shell` and `shell_commands`
modules and all the driver modules each board supports.

`shell` is a very simple interactive command interpreter that can be
used to call functions.  Many of RIOT's modules define some generic
shell commands. These are included via the `shell_commands` module.

Additionally, the `ps` module which provides the `ps` shell command is
included.

Usage
=====

Build, flash and start the application:
```
export BOARD=axio-builder-ms500
make
make sig
make flash 
make term
```

The `term` make target starts a terminal emulator for your board. It
connects to a default port so you can interact with the shell, usually
that is `/dev/ttyUSB0`. If your port is named differently, the
`PORT=/dev/yourport` variable can be used to override this.


