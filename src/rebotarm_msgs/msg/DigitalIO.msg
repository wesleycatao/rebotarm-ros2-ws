# A single digital I/O point (Raspberry Pi GPIO).
#
# direction
#   0 = input (opto-isolated), 1 = output (relay)
# id
#   Stable logical identifier, e.g. "IN1" or "OUT3".
# tag
#   User-defined descriptive label.
# pin
#   BCM GPIO pin number backing this I/O point.
# enabled
#   Whether the point is active (disabled points are ignored by the
#   motion planner and report a static low level).
# value
#   Current logic level (input: read level; output: commanded relay state).
# activation_count
#   Number of activations: rising edges for inputs, energize commands for
#   outputs.

uint8 DIR_INPUT=0
uint8 DIR_OUTPUT=1

uint8 direction
string id
string tag
uint32 pin
bool enabled
bool value
uint32 activation_count
