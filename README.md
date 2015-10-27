Builds a Throughput Test Binary
===============================

The throughput test is a IPv6 based test that streams packets
from a sender node to a connected PC. You will need a server software to
properly use the test.

To start the test, press the button on the source node which maps to
`button_sensor`. This project will not currently work on platforms without
any buttons.

Building
========

This project builds just like any other Contiki project. Just use
`make TARGET=<target>`. If, however, the contiki source tree does not reside at
`../../contiki_src/contiki` then you will need to use
`make CONTIKI=path/to/contiki/ TARGET=<target>` or edit the Makefile.
