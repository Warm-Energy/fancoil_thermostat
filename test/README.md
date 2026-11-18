Host unit test for the MCU codec (no hardware needed):
    clang -I ../src -Wall ../src/mcu_protocol.c test_proto.c -o /tmp/test_proto && /tmp/test_proto
Covers: set-frame framing/checksum, status-frame decode, reset-bit detection, parser
resync after a false start, 0x0F replies, and real healthy + shifted-reply captures.
