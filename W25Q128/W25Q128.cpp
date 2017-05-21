// W25Q128.cpp

#include "W25Q128.hpp"

W25Q128::W25Q128(PinName mosi, PinName miso, PinName sclk, PinName _cs)
: spi(mosi, miso, sclk), cs(_cs)
{
	spi.format(SPI_NBIT, SPI_MODE);
	spi.frequency(SPI_FREQ);
	cs_disable();
}

uint32_t W25Q128::get_id() {
	cs_enable();
	spi.write(0x90);
	spi.write(0);
	spi.write(0);
	spi.write(0);
	char id = spi.write(DUMMY_ADDR);
	cs_disable();
	return id;
}

bool W25Q128::push_page(const page &p)
{
	if (bytes_pushed >= NUM_BYTES)
		return false;

	write_page(bytes_pushed,p);

	bytes_pushed += p.size();

	return true;
}

int W25Q128::get_bytes_pushed() const
{
	return bytes_pushed;
}

char W25Q128::read_byte(uint32_t addr) {
	cs_enable();
	spi.write(R_INST);
	spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
	spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
	spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);
	int response = spi.write(DUMMY_ADDR);
	cs_disable();
	return response;
}

void W25Q128::read_stream(uint32_t addr, char* buf, int count) {
	if (count < 1)
		return;
	cs_enable();
	spi.write(R_INST);
	spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
	spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
	spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);
	for (int i = 0; i < count; i++)
		buf[i] =  spi.write(DUMMY_ADDR);
	cs_disable();
}

void W25Q128::read_page(uint32_t page_num, page &p) {
	int addr = page_num << 8;
	cs_enable();
	spi.write(R_INST);
	spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
	spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
	spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);
	for (auto & byte : p)
		byte = spi.write(DUMMY_ADDR);
	cs_disable();
}

void W25Q128::write_byte(uint32_t addr, char data)
{
	while (is_busy())
	{
		wait_us(5);
	}

	write_enable();
	cs_enable();
	spi.write(W_INST);
	spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
	spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
	spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);
	spi.write(data);
	cs_disable();
	write_disable();
	while (is_busy())
	{
		wait_us(5);
	}
}

void W25Q128::write_stream(uint32_t addr, char* buf, int count)
{
	if (count < 1)
		return;

	while (is_busy())
	{
		wait_us(5);
	}

	write_enable();
	cs_enable();
	spi.write(W_INST);
	spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
	spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
	spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);
	for (int i = 0; i < count; i++)
		spi.write(buf[i]);
	cs_disable();
	write_disable();

	while (is_busy())
	{
		wait_us(5);
	}
}

void W25Q128::write_page(uint32_t addr, const page &p)
{
	while (is_busy())
	{
		wait_us(5);
	}

	write_enable();
	cs_enable();
	spi.write(W_INST);
	spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
	spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
	spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);

	for (const auto & c : p)
		spi.write(c);

	cs_disable();
	write_disable();

	while (is_busy())
	{
		wait_us(5);
	}
}

void W25Q128::erase_chip()
{
	while (is_busy())
	{
		wait_ms(1000);
	}

	write_enable();
	cs_enable();
	spi.write(C_ERASE_INST);
	cs_disable();
	write_disable();

	while (is_busy())
	{
		wait_ms(1000);
	}
	bytes_pushed = 0;
}

void W25Q128::erase_sector(uint32_t sector_num) {
	while (is_busy())
	{
		wait_ms(1);
	}
	int addr = sector_num << 12;
	write_enable();
	cs_enable();
	spi.write(SE_INST);
	spi.write((addr & ADDR_BMASK2) >> ADDR_BSHIFT2);
	spi.write((addr & ADDR_BMASK1) >> ADDR_BSHIFT1);
	spi.write((addr & ADDR_BMASK0) >> ADDR_BSHIFT0);
	cs_disable();
	write_disable();

	while (is_busy())
	{
		wait_ms(1);
	}
}

bool W25Q128::is_busy()
{
	char w_en_bitmask = 0x1;
	cs_enable();
	spi.write(0x5);
	char status = spi.write(0);
	cs_disable();
	return w_en_bitmask & status;
}

void W25Q128::write_enable() {
	cs_enable();
	spi.write(WE_INST);
	cs_disable();
}

void W25Q128::write_disable() {
	cs_enable();
	spi.write(WD_INST);
	cs_disable();
}

void W25Q128::cs_enable() {
	cs = 0;
}

void W25Q128::cs_disable() {
	cs = 1;
}
