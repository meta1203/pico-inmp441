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
	sm_config_set_wrap(&c, this->pio.offset, this->pio.offset+pio_i2s_read32.length-1);
	sm_config_set_sideset(&c, 2, false, false);
	sm_config_set_in_pins(&c, this->pio.pin_start+2);
	sm_config_set_in_shift(&c, false, true, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	sm_config_set_sideset(&c, 2, false, false);
	sm_config_set_sideset_pins(&c, this->pio.pin_start);
	// sample rate * 32 bits per sample * 2 channels * 2 instructions per bit
	float clock_rate = this->sample_rate * 32.0 * 2.0 * 2.0;
	float div = clock_get_hz(clk_sys) / clock_rate;
	sm_config_set_clkdiv(&c, div);
	pio_sm_init(this->pio.pio, this->pio.sm, this->pio.offset, &c);
	pio_init_gpio(this->pio.pio, this->pio.sm, this->pio.pin_start, 1, 1);
	pio_init_gpio(this->pio.pio, this->pio.sm, this->pio.pin_start+1, 1, 1);
	pio_init_gpio(this->pio.pio, this->pio.sm, this->pio.pin_start+2, 0, 0);
	pio_sm_set_enabled(this->pio.pio, this->pio.sm, true);
}

template<typename T>
void INMP441<T>::init_dma() {
	this->dma.ringbuf = (int32_t*)aligned_alloc(this->buffered_sample_count, this->buffered_sample_count * sizeof(int32_t));
	for (size_t i = 0; i < this->buffered_sample_count; i += 1) {
		this->dma.ringbuf[i] = 0;
	}
	this->dma.channel = dma_claim_unused_channel(true);
	dma_channel_config dma_config = dma_channel_get_default_config(this->dma.channel);
	channel_config_set_read_increment(&dma_config, false);
	channel_config_set_write_increment(&dma_config, true);
	channel_config_set_ring(&dma_config, true, log2(this->buffered_sample_count));
	channel_config_set_dreq(&dma_config, pio_get_dreq(this->pio.pio, this->pio.sm, false));
	#if PICO_RP2040
	// RP2040-specific code (using 2-channel chained dma)
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
	dma_channel_hw_addr(this->dma.channel)->transfer_count;
	return dma_channel_hw_addr(this->dma.channel)->write_addr - (size_t)this->dma.ringbuf;
}

template<typename T>
void INMP441<T>::read_audio_left(T* buf, size_t len) {
	size_t buf_pos = this->dma.read_head;
	if (buf_pos & 0b1 == 1) buf_pos += 1;
	for (size_t i = 0; i < len; i += 1) {
		buf[i] = (T)(this->dma.ringbuf[(buf_pos+i*2) % this->buffered_sample_count] / INMP441_SIGNAL_DIVISOR);
	}
	this->dma.read_head = (this->dma.read_head + (len*2)) % this->buffered_sample_count;
}

template<typename T>
void INMP441<T>::read_audio_right(T* buf, size_t len) {
	size_t buf_pos = this->dma.read_head;
	if (buf_pos & 0b1 == 0) buf_pos += 1;
	for (size_t i = 0; i < len; i += 1) {
		buf[i] = (T)(this->dma.ringbuf[(buf_pos+i*2) % this->buffered_sample_count] / INMP441_SIGNAL_DIVISOR);
	}
	this->dma.read_head = (this->dma.read_head + (len*2)) % this->buffered_sample_count;
}

template<typename T>
void INMP441<T>::read_audio_interleaved(T* buf, size_t len) {
	size_t buf_pos = this->dma.read_head;
	if (buf_pos & 0b1 == 1) buf_pos += 1;
	for (size_t i = 0; i < len; i += 1) {
		buf[i] = (T)(this->dma.ringbuf[(buf_pos+i) % this->buffered_sample_count] / INMP441_SIGNAL_DIVISOR);
	}
	this->dma.read_head = (this->dma.read_head + len*2) % this->buffered_sample_count;
}

template class INMP441<int32_t>;
template class INMP441<float>;
template class INMP441<double>;