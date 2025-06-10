# INMP441 Raspberry Pi Pico Library

A C++ library for interfacing with the INMP441 I2S MEMS microphone using Raspberry Pi Pico (RP2040) and other RP2xxx microcontrollers.

## Overview

This library provides a simple and efficient way to capture audio from an INMP441 microphone using the Programmable I/O (PIO) and Direct Memory Access (DMA) capabilities of the Raspberry Pi Pico. It supports both the original RP2040 and newer RP2350+ chips.

Key features:
- Efficient audio capture using PIO and DMA
- Configurable sample rate (default: 48kHz)
- Configurable buffer size
- Support for reading left channel, right channel, or interleaved stereo audio
- Template-based design supporting multiple data types (int32_t, float, double)
- Automatic PIO/state machine allocation

## Hardware Requirements

- Raspberry Pi Pico or other RP2xxx-based board
- INMP441 I2S MEMS microphone
- Appropriate connections (see Wiring section)

## Wiring

Connect your INMP441 microphone to the Raspberry Pi Pico as follows:

| INMP441 Pin  | Function | Raspberry Pi Pico |
|--------------|----------|------------------|
| VDD          | Power    | 3.3V             |
| GND          | Ground   | GND              |
| SCK          | Clock    | GPIO pin_start   |
| L/R (aka WS) | Left/Right Select | GPIO pin_start+1 |
| SD           | Data     | GPIO pin_start+2 |


Note: `pin_start` is the base GPIO pin you specify when initializing the library.

## Installation

1. Copy `inmp441.h` and `inmp441.cpp` to your project directory
2. Add `inmp441.h` and `inmp441.cpp` to your `CMakeLists.txt`
2. Include the header in your code:
```cpp
#include "inmp441.h"
```

## Usage

### Basic Example

```cpp
#include "inmp441.h"
#include <stdio.h>
#include "pico/stdlib.h"

#define PIN_START 10  // Base GPIO pin (SCK)

int main() {
    stdio_init_all();
    
    // Initialize the INMP441 microphone
    INMP441<int32_t> mic(PIN_START);
    
    // Buffer to store audio samples
    int32_t samples[256];
    
    while (true) {
        // Read audio from left channel
        mic.read_audio_left(samples, 256);
        
        // Process audio data...
        for (int i = 0; i < 256; i++) {
            printf("%d ", samples[i]);
        }
        printf("\n");
        
        sleep_ms(100);
    }
    
    return 0;
}
```

### Reading Different Audio Channels

```cpp
// Read from left channel
mic.read_audio_left(buffer, length);

// Read from right channel
mic.read_audio_right(buffer, length);

// Read interleaved stereo (left and right alternating)
mic.read_audio_interleaved(buffer, length);
```

### Using Different Data Types

```cpp
// Using int32_t (raw samples)
INMP441<int32_t> mic_int(PIN_START);

// Using float (normalized samples)
INMP441<float> mic_float(PIN_START);

// Using double (for higher precision if needed)
INMP441<double> mic_double(PIN_START);
```

## API Reference

### Constructor

```cpp
INMP441(
    uint8_t pin_start,
    size_t buffered_sample_count = 4096,
    uint32_t sample_rate = 48000,
    uint8_t target_pio = 0xff
);
```

Parameters:
- `pin_start`: Base GPIO pin number (SCK). The library will use pin_start, pin_start+1, and pin_start+2.
- `buffered_sample_count`: Size of the internal buffer (must be a power of 2). Default is 4096.
- `sample_rate`: Audio sample rate in Hz. Default is 48000.
- `target_pio`: Specific PIO to use, or 0xff for automatic selection. Default is 0xff.

### Methods

```cpp
void read_audio_left(T* buf, size_t len);
```
Reads `len` samples from the left audio channel into the buffer `buf`.

```cpp
void read_audio_right(T* buf, size_t len);
```
Reads `len` samples from the right audio channel into the buffer `buf`.

```cpp
void read_audio_interleaved(T* buf, size_t len);
```
Reads `len` interleaved stereo samples (left and right alternating) into the buffer `buf`.

## Technical Details

- The library uses PIO to implement the I2S protocol for communicating with the INMP441 microphone.
- DMA is used to efficiently transfer audio data from the PIO FIFO to a ring buffer in memory.
- The INMP441 provides 24-bit audio samples.
- The library automatically handles buffer wrapping and channel selection.

## Limitations

- The buffer size must be a power of 2.
- Requires at least one free PIO state machine on a PIO module with space for an 8 instruction program
- Requires one or two free DMA channels (depending on the chip).

## Credits

PIO implementation gleaned from [sfera-lab's repo](https://github.com/sfera-labs/arduino-pico-i2s-audio/blob/master/src/machine_i2s.c)