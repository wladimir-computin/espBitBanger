# EspBitBanger

## Implementation of a flexible speed / high precision bit banger for the ESP8266 / ESP32 family
### based on [EspSoftwareSerial](https://github.com/plerup/espsoftwareserial)

This fork implements interrupt service routine best practice.
In the receive interrupt, instead of blocking for whole bytes
at a time - voiding any near-realtime behavior of the CPU - only level
change and timestamp are recorded. The more time consuming phase
detection and byte assembly are done in the main code.

Except at high bitrates, depending on other ongoing activity,
interrupts in particular, this software serial adapter
supports full duplex receive and send. At high bitrates (115200bps)
send bit timing can be improved at the expense of blocking concurrent
full duplex receives, with the `BitBanger::enableIntTx(false)` function call.

The same functionality is given as the corresponding AVR library but
several instances can be active at the same time. Speed up to 115200 baud
is supported. Besides a constructor compatible to the AVR BitBanger class,
and updated constructor that takes no arguments exists, instead the `begin()`
function can handle the pin assignments and logic inversion.
It also has optional input buffer capacity arguments for byte buffer and ISR bit buffer.
This way, it is a better drop-in replacement for the hardware serial APIs on the ESP MCUs.

Please note that due to the fact that the ESPs always have other activities
ongoing, there will be some inexactness in interrupt timings. This may
lead to inevitable, but few, bit errors when having heavy data traffic
at high baud rates.

This library supports ESP8266, ESP32, ESP32-S2 and ESP32-C3 devices.

## Resource optimization

The memory footprint can be optimized to just fit the amount of expected
incoming asynchronous data.
For this, the `BitBanger` constructor provides two arguments. First, the
octet buffer capacity for assembled received octets can be set. Read calls are
satisfied from this buffer, freeing it in return.
Second, the signal edge detection buffer of 32bit fields can be resized.
One octet may require up to to 10 fields, but fewer may be needed,
depending on the bit pattern. Any read or write calls check this buffer
to assemble received octets, thus promoting completed octets to the octet
buffer, freeing fields in the edge detection buffer.

Look at the swsertest.ino example. There, on reset, ASCII characters ' ' to 'z'
are sent. This happens not as a block write, but in a single write call per
character. As the example uses a local loopback wire, every outgoing bit is
immediately received back. Therefore, any single write call causes up to
10 fields - depending on the exact bit pattern - to be occupied in the signal
edge detection buffer. In turn, as explained before, each single write call
also causes received bit assembly to be performed, promoting these bits from
the signal edge detection buffer to the octet buffer as soon as possible.
Explaining by way of contrast, if during a a single write call, perhaps because
of using block writing, more than a single octet is received, there will be a
need for more than 10 fields in the signal edge detection buffer.
The necessary capacity of the octet buffer only depends on the amount of incoming
data until the next read call.

For the swsertest.ino example, this results in the following optimized
constructor arguments to spend only the minimum RAM on buffers required:

The octet buffer capacity (`bufCapacity`) is 95 (93 characters net plus two tolerance).
The signal edge detection buffer capacity (`isrBufCapacity`) is 11, as each
single octet can have up to 11 bits on the wire,
which are immediately received during the write, and each
write call causes the signal edge detection to promote the previously sent and
received bits to the octet buffer.

In a more generalized scenario, calculate the bits (use message size in octets
times 10) that may be asynchronously received to determine the value for
`isrBufCapacity` in the constructor. Also use the number of received octets
that must be buffered for reading as the value of `bufCapacity`.
The more frequently your code calls write or read functions, the greater the
chances are that you can reduce the `isrBufCapacity` footprint without losing data,
and each time you call read to fetch from the octet buffer, you reduce the
need for space there.

## Checking for correct pin selection / configuration 
In general, most pins on the ESP8266 and ESP32 devices can be used by BitBanger, 
however each device has a number of pins that have special functions or require careful
handling to prevent undesirable situations, for example they are connected to the 
on-board SPI flash memory or they are used to determine boot and programming modes 
after powerup or brownouts. These pins are not able to be configured by this library.

The exact list for each device can be found in the
[ESP32 data sheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
in sections 2.2 (Pin Descriptions) and 2.4 (Strapping pins). There is a discussion
dedicated to the use of GPIO12 in this
[note about GPIO12](https://github.com/espressif/esp-idf/tree/release/v3.2/examples/storage/sd_card#note-about-gpio12).
Refer to the `isValidGPIOpin()`, `isValidRxGPIOpin()` and `isValidTxGPIOpin()`
functions for the GPIO restrictions enforced by this library by default.

The easiest and safest method is to test the object returned at runtime, to see if 
it is valid. For example:

```
#include <EspBitBanger.h>

#define MYPORT_TX 12
#define MYPORT_RX 13

EspBitBanger myPort;

[...]

Serial.begin(115200); // Standard hardware serial port

myPort.begin(38400, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
if (!myPort) { // If the object did not initialize, then its configuration is invalid
  Serial.println("Invalid BitBanger pin configuration, check config"); 
  while (1) { // Don't continue with invalid configuration
    delay (1000);
  }
} 

[...]
```

## Installation (Arduino)

```shell
$ cd libraries/BitBanger
$ git clone https://github.com/wladimir-computin/espBitBanger.git
```
