/*
 *	UVR W25Q128 MBED Library
 *
 *	@author	 Andres Martinez
 *	@version 0.9a
 *	@date	 8-May-2017
 *
 *	MBED library for Winbond W25Q128 flash memory.
 *
 *	Written for UVic Rocketry
 *
 */

#ifndef W25Q128_H
#define W25Q128_H

#include "mbed.h"
#include <array>

#define SPI_FREQ		100000000
#define SPI_MODE		0
#define SPI_NBIT		8

#define WE_INST			0x06 // write enable
#define WD_INST			0x04 // write disable
#define R_INST			0x03 // read
#define W_INST			0x02 // page write
#define C_ERASE_INST	0x60 // chip erase
#define SE_INST			0x20 // sector erase

#define DUMMY_ADDR		0x0

#define ADDR_BMASK2		0x00ff0000
#define ADDR_BMASK1		0x0000ff00
#define ADDR_BMASK0		0x000000ff

#define ADDR_BSHIFT2	16
#define ADDR_BSHIFT1	8
#define ADDR_BSHIFT0	0

#define PAGE_SIZE		256
#define NUM_BLOCKS		256
#define SECT_PER_BLOCK	16
#define PAGES_PER_SECT	16

#define BLOCK_MASK		0xff0000
#define SECTOR_MASK		0xfff000
#define PAGE_MASK		0xffff00

typedef std::array<char,PAGE_SIZE> page;

class W25Q128 {
public:
	W25Q128(PinName mosi, PinName miso, PinName sclk, PinName cs);

	char read_byte(uint32_t addr);
	void read_page(uint32_t page_num, page &p);

	bool push_page_back(const page &p);

	void erase_chip();
	void erase_sector(uint32_t sector_num);

	uint32_t get_id();

	bool is_busy();

	int get_pages_pushed() const;
	
	static constexpr int MAX_PAGES = NUM_BLOCKS * SECT_PER_BLOCK * PAGES_PER_SECT;
	static constexpr int MAX_BYTES = MAX_PAGES * PAGE_SIZE;

	private:

	int pages_pushed{0};

	SPI spi;
	DigitalOut cs;

	void write_byte(uint32_t addr, char data);
	void write_page(uint32_t addr, const page &p);
	void write_stream(uint32_t addr, char* buf, int count);
	void read_stream(uint32_t addr, char* buf, int count);
	
	void write_enable();
	void write_disable();
	void cs_enable();									   // chip enable
	void cs_disable();									   // chip disable

	W25Q128() = delete;
	W25Q128(const W25Q128& other) = delete;
	W25Q128& operator=(const W25Q128& other) = delete;
};

#endif