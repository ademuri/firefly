# Radio Test

Tests basic functionality of a single Firefly node. Uses a sketch that translates
between ASCII-over-serial and packets (in both directions) to emulate a node.

## How to use

Load the serial radio .hex file onto a Firefly board. Or, build SerialRadio.ino
yourself and load it. It uses the same cc1101 as the main Firefly project.

Then, load the Firefly sketch to be tested onto another Firefly board, and
power it. You won't need to do anything else with it for the duration of the test.

Then, run `python radio_test.py <serial_port>` to run the tests.

Serial port examples:
- Windows: COM9
- Linux: /dev/tty1