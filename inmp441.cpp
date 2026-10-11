#include "inmp441.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include <math.h>

// i2s spec is 32 bits, but the INMP411 only sends 24 bits
// chop off the lower 8 bits maintaining signed value
#define INMP441_SIGNAL_DIVISOR (256)

template<typename T>
INMP441<T>::INMP441(
	uint8_t pin_start,
	size_t buffered_sample_count,
	uint32_t sample_rate,
	uint8_t target_pio
) {
	float bsc_power = log2(buffered_sample_count);
	if (floor(bsc_power) != bsc_power)
		panic("MP441: ERROR - buffered_sample_count must be a power of two, current value is %d (2^%f)", buffered_sample_count, bsc_power);
	// ring buffer holds interleaved L/R slots, so double the requested per-channel count
	this->buffered_sample_count = buffered_sample_count<<1;

	this->sample_rate = sample_rate;

	this->pio.pin_start = pin_start;
	this->init_pio();

	this->init_dma();
}

template<typename T>
INMP441<T>::~INMP441() {
	dma_channel_cleanup(this->dma.channel);
	#if PICO_RP2040
	dma_channel_cleanup(this->dma.chain_channel);
	#endif
	free(this->dma.ringbuf);
	pio_remove_program_and_unclaim_sm(
		&pio_i2s_read32,
		this->pio.pio,
		this->pio.sm,
		this->pio.offset
	);
}

// configure one GPIO for use by a state machine: route it to PIO, then set its
// initial output value and direction via the SM's pin control registers
inline static void pio_init_gpio(PIO pio, uint8_t sm, uint8_t pin_num, uint8_t pin_val, uint8_t pin_dir) {
	pio_gpio_init(pio, pin_num);
	uint32_t pinmask = 1 << pin_num;
	pio_sm_set_pins_with_mask(pio, sm, pin_val << pin_num, pinmask);
	pio_sm_set_pindirs_with_mask(pio, sm, pin_dir << pin_num, pinmask);
}

template<typename T>
void INMP441<T>::init_pio(uint8_t install_to) {
	if (install_to == 0xff) {
		// automatically find a usable PIO module and claim a state machine
		for (uint8_t i = 0; i < NUM_PIOS; i += 1) {
			this->pio.pio = pio_get_instance(i);
			if (pio_can_add_program(this->pio.pio, &pio_i2s_read32)) {
				int8_t test_sm = pio_claim_unused_sm(this->pio.pio, false);
				if (test_sm >= 0) {
					this->pio.sm = test_sm;
					goto success;
				}
			}
		}
		panic("INMP441: ERROR - could not automatically add PIO program.");
		success:
		;
	} else {
		// use the explicitly defined PIO module
		this->pio.pio = pio_get_instance(install_to);
		hard_assert(pio_can_add_program(this->pio.pio, &pio_i2s_read32));
		this->pio.sm = pio_claim_unused_sm(this->pio.pio, true);
	}

	// install and initialize the program/state machine
	this->pio.offset = pio_add_program(this->pio.pio, &pio_i2s_read32);
	pio_sm_config c = pio_get_default_sm_config();
	// loop the SM over the full instruction range of the program
	sm_config_set_wrap(&c, this->pio.offset, this->pio.offset+pio_i2s_read32.length-1);
	// 2 side-set bits (SCK + WS), non-optional, no delay bits in the encodings
	sm_config_set_sideset(&c, 2, false, false);
	// the SD data pin sits 2 pins after SCK
	sm_config_set_in_pins(&c, this->pio.pin_start+2);
	// shift left (I2S is MSB-first), auto-push the ISR once all 32 bits are shifted in
	sm_config_set_in_shift(&c, false, true, 32);
	// we only ever read from the mic, so give the SM a single 8-deep RX FIFO
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	sm_config_set_sideset(&c, 2, false, false); // NOTE: duplicate of the call above
	sm_config_set_sideset_pins(&c, this->pio.pin_start);
	// the PIO program needs 64 SM cycles per stereo frame:
	// sample rate * 32 bits per sample * 2 channels * 2 instructions per bit
	float clock_rate = this->sample_rate * 32.0 * 2.0 * 2.0;
	// derive the SM clock divider from clk_sys to hit that rate exactly
	float div = clock_get_hz(clk_sys) / clock_rate;
	sm_config_set_clkdiv(&c, div);
	pio_sm_init(this->pio.pio, this->pio.sm, this->pio.offset, &c);
	// drive SCK and WS as PIO outputs (they are toggled by the side-set field),
	// and route the SD data pin in as the SM's input
	pio_init_gpio(this->pio.pio, this->pio.sm, this->pio.pin_start, 1, 1);
	pio_init_gpio(this->pio.pio, this->pio.sm, this->pio.pin_start+1, 1, 1);
	pio_init_gpio(this->pio.pio, this->pio.sm, this->pio.pin_start+2, 0, 0);
	pio_sm_set_enabled(this->pio.pio, this->pio.sm, true);
}

template<typename T>
void INMP441<T>::init_dma() {
	// ring buffer of interleaved L/R samples; aligned to its own byte size so the
	// DMA ring wrap (a power-of-2 address window) always stays inside the allocation
	const uint32_t dma_ring_buffer_byte_size = this->buffered_sample_count * sizeof(int32_t);
	this->dma.ringbuf = (int32_t*)aligned_alloc(dma_ring_buffer_byte_size, dma_ring_buffer_byte_size);
	std::memset(this->dma.ringbuf, 0, dma_ring_buffer_byte_size);
	this->dma.channel = dma_claim_unused_channel(true);
	dma_channel_config dma_config = dma_channel_get_default_config(this->dma.channel);
	channel_config_set_read_increment(&dma_config, false);
	channel_config_set_write_increment(&dma_config, true);
	// wrap DMA write addresses in a power-of-2 byte window so writes circle the
	// ring buffer; ring_size is log2 of the window size in bytes
	channel_config_set_ring(&dma_config, true, log2(this->buffered_sample_count)+log2(sizeof(int32_t)));
	// pace transfers off the SM's RX FIFO (one 32-bit word per completed slot)
	channel_config_set_dreq(&dma_config, pio_get_dreq(this->pio.pio, this->pio.sm, false));
	#if PICO_RP2040
	// RP2040-specific code (using 2-channel chained dma)
	// RP2040 has no endless-transfer mode, so when the main channel finishes a full
	// pass it chains to a second channel that rewrites its write_addr back to the
	// buffer start; that channel in turn chains back to the main channel, looping forever
	this->dma.chain_channel = dma_claim_unused_channel(true);
	channel_config_set_chain_to(
		&dma_config,
		this->dma.chain_channel
	);

	dma_channel_configure(
		this->dma.channel,
		&dma_config,
		this->dma.ringbuf,
		&this->pio.pio->rxf[this->pio.sm],
		this->buffered_sample_count,
		true
	);

	dma_channel_config chain_config = dma_channel_get_default_config(this->dma.chain_channel);
	channel_config_set_transfer_data_size(&chain_config, DMA_SIZE_32);
	channel_config_set_read_increment(&chain_config, false);
	channel_config_set_write_increment(&chain_config, false);
	// the reload channel reads the stored base pointer (not the buffer contents!)
	// and pokes it into the main channel's write_addr register
	channel_config_set_chain_to(&chain_config, this->dma.channel);
	dma_channel_configure(
		this->dma.chain_channel,
		&chain_config,
		&(dma_channel_hw_addr(this->dma.channel)->write_addr),
		&this->dma.ringbuf,
		1,
		true
	);
	#else
	// rp2350+ code (single-channel dma chaining)
	// TRANS_COUNT bits 31:28 = MODE; 0x1 means "endless": the channel re-triggers
	// itself after every pass and wraps write_addr via the DMA ring config
	dma_channel_configure(
		this->dma.channel,
		&dma_config,
		this->dma.ringbuf,
		&this->pio.pio->rxf[this->pio.sm],
		(0x1 << 28) | this->buffered_sample_count,
		true
	);
	#endif
	this->dma.read_head = 0;
}

template<typename T>
size_t INMP441<T>::get_write_head() {
	// returns the DMA write position as a byte offset from the ring buffer start
	dma_channel_hw_addr(this->dma.channel)->transfer_count;
	return dma_channel_hw_addr(this->dma.channel)->write_addr - (size_t)this->dma.ringbuf;
}

template<typename T>
void INMP441<T>::read_audio_left(T* buf, size_t len) {
	// left-channel slots are the even indices of the interleaved ring buffer;
	// stride 2 de-interleaves, and the head advances by 2*len slots
	size_t buf_pos = this->dma.read_head;
	if ((buf_pos & 0b1) == 1) buf_pos += 1;
	for (size_t i = 0; i < len; i += 1) {
		buf[i] = (T)(this->dma.ringbuf[(buf_pos+i*2) % this->buffered_sample_count] / INMP441_SIGNAL_DIVISOR);
	}
	this->dma.read_head = (this->dma.read_head + (len*2)) % this->buffered_sample_count;
}

template<typename T>
void INMP441<T>::read_audio_right(T* buf, size_t len) {
	// right-channel slots are the odd indices of the interleaved ring buffer
	size_t buf_pos = this->dma.read_head;
	if ((buf_pos & 0b1) == 0) buf_pos += 1;
	for (size_t i = 0; i < len; i += 1) {
		buf[i] = (T)(this->dma.ringbuf[(buf_pos+i*2) % this->buffered_sample_count] / INMP441_SIGNAL_DIVISOR);
	}
	this->dma.read_head = (this->dma.read_head + (len*2)) % this->buffered_sample_count;
}

template<typename T>
void INMP441<T>::read_audio_interleaved(T* buf, size_t len) {
	size_t buf_pos = this->dma.read_head;
	if ((buf_pos & 0b1) == 1) buf_pos += 1;
	for (size_t i = 0; i < len; i += 1) {
		buf[i] = (T)(this->dma.ringbuf[(buf_pos+i) % this->buffered_sample_count] / INMP441_SIGNAL_DIVISOR);
	}
	this->dma.read_head = (this->dma.read_head + len*2) % this->buffered_sample_count;
}

template class INMP441<int32_t>;
template class INMP441<float>;
template class INMP441<double>;