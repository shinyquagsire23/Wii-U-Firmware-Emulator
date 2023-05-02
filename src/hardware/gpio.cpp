
#include "common/logger.h"
#include "common/fileutils.h"
#include "common/endian.h"
#include "hardware/gpio.h"
#include "hardware.h"


void SEEPROMController::reset() {
	buffer = FileUtils::load("files/seeprom.bin");
	data = (uint16_t *)buffer.get();
	
	state = LISTEN;
	
	cycles = 11;
	offset = 0;
	value_in = 0;
	value_out = 0;
	//pin_in = false;
	//pin_out = false;
}

void SEEPROMController::prepare() {
	state = LISTEN;
	cycles = 11;
	value_in = 0;
	value_out = 0;
}

void SEEPROMController::write(bool state) {
	pin_in = state;
}

bool SEEPROMController::read() {
	return pin_out;
}

void SEEPROMController::update() {
	//Logger::warning("Seeprom cmd %08x %08x %04x %04x %x %x", state, cycles, value_in, value_out, pin_in, pin_out);

	value_in = (value_in << 1) | pin_in;
	//value_out = (value_out << 1) | pin_out;
	pin_out = !!(value_out & 0x8000);
	value_out <<= 1;

	if (state == LISTEN) {
		//value_in = (value_in << 1) | pin_in;
		if (--cycles == 0) {
			handle_command();
		}
	}
	else if (state == WRITE) {
		//value_in = (value_in << 1) | pin_in;
		if (--cycles == 0) {
			handle_write();
		}
	}
	else if (state == WRITE_DELAY) {
		if (--cycles == 0) {
			value_in = 1;
			cycles = 1;
			state = READ;
		}
	}
	else if (state == READ) {
		//pin_out = value_out & 1;
		//value_out >>= 1;
		if (--cycles == 0) {
			cycles = 2;
			state = DELAY;
		}
	}
	else if (state == DELAY) {
		if (--cycles == 0) {
			prepare();
		}
	}
}

void SEEPROMController::handle_command() {
	int command = value_in >> 8;
	int param = value_in & 0xFF;

	Logger::warning("Seeprom cmd %08x, %08x", command, param);
	
	if (command == 4) {
		cycles = 2;
		state = DELAY;
	}
	else if (command == 5) {
		cycles = 16;
		value_in = 0;
		offset = param;
		state = WRITE;
	}
	else if (command == 6) {
		cycles = 16;
		value_out = Endian::swap16(data[param]);
		state = READ;
	}
	else {
		Logger::warning("Unknown seeprom command: 0x%X", value_in);

		cycles = 16;
		value_out = 0xFFFF;
		state = READ;
	}
	value_in = 0;
}

void SEEPROMController::handle_write() {
	data[offset] = Endian::swap16(value_in);
	state = WRITE_DELAY;
	cycles = 2;
}


void GPIOCommon::reset() {
	seeprom.reset();
}

uint32_t GPIOCommon::read() {
	return seeprom.read() << PIN_EEPROM_DI;
}

void GPIOCommon::write(int pin, bool state) {
	//if (pin <= PIN_EEPROM_DI)
	//	Logger::warning("common gpio pin write: %i (%b)", pin, state);
	if (pin == PIN_EEPROM_CS) {
		if (!state) {
			seeprom.reset();
		}
		else {
			seeprom.reset();
		}
	}
	else if (pin == PIN_EEPROM_SK) {
		if (state) seeprom.update();
	}
	else if (pin == PIN_EEPROM_DO) seeprom.write(state);
	else if (pin == PIN_DEBUG0) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 0: %b", state);*/ this->debug_val &= ~(1<<0); this->debug_val |= (state ? 1<<0 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_DEBUG1) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 1: %b", state);*/ this->debug_val &= ~(1<<1); this->debug_val |= (state ? 1<<1 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_DEBUG2) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 2: %b", state);*/ this->debug_val &= ~(1<<2); this->debug_val |= (state ? 1<<2 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_DEBUG3) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 3: %b", state);*/ this->debug_val &= ~(1<<3); this->debug_val |= (state ? 1<<3 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_DEBUG4) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 4: %b", state);*/ this->debug_val &= ~(1<<4); this->debug_val |= (state ? 1<<4 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_DEBUG5) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 5: %b", state);*/ this->debug_val &= ~(1<<5); this->debug_val |= (state ? 1<<5 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_DEBUG6) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 6: %b", state);*/ this->debug_val &= ~(1<<6); this->debug_val |= (state ? 1<<6 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_DEBUG7) { this->last_debug_val = this->debug_val; /*Logger::info("Debug 7: %b", state);*/ this->debug_val &= ~(1<<7); this->debug_val |= (state ? 1<<7 : 0); /*Logger::info("Debug is: %x", this->debug_val);*/ }
	else if (pin == PIN_TOUCAN) {}
	else {
		Logger::warning("Unknown common gpio pin write: %i (%b)", pin, state);
	}

	if (!this->debug_serial && this->last_debug_val == 0x0F && this->debug_val == 0x8F) {
		this->debug_serial = 1;
		this->debug_bits = 0;
		this->debug_byte = 0;

		Logger::warning("DEBUG SERIAL");
	}
	else if (debug_serial) {
		if (this->last_debug_val != this->debug_val) {
			//Logger::warning("DEBUG TRANS: %08x->%08x %08x", this->last_debug_val, this->debug_val);
			//this->last_debug_val = this->debug_val;
		}

		if (this->debug_bits <= 7 && 
			(this->last_debug_val & 0x80) == 0 && 
			(this->debug_val & 0x80) == 0x80 && 
			(this->debug_val & 0x7E) == 0) {
			this->debug_byte = (this->debug_byte << 1) | (this->debug_val & 1);
			this->debug_bits += 1;
		}
		else if (this->debug_val == 0x8F && this->debug_bits >= 8) {
			//Logger::warning("DEBUG BYTE: %08x", this->debug_byte);

			if (this->debug_byte == '\t' || this->debug_byte == '\n' || this->debug_byte == '\r' || (this->debug_byte >= ' ' && this->debug_byte <= '~')) {
				if (this->debug_byte == '\n') {
					Logger::warning("DEBUG: %s", this->debug_str.c_str());
					this->debug_str = "";
				}
				else {
					this->debug_str += (char)this->debug_byte;
				}
			}
			

			this->debug_bits = 0;
			this->debug_byte = 0;
		}
	}

	if (!this->debug_serial && this->last_debug_val != this->debug_val) {
		Logger::warning("DEBUG BYTE: %08x", this->debug_val);
		this->last_debug_val = this->debug_val;
	}
}


GPIOLatte::GPIOLatte(HDMIController *hdmi) {
	this->hdmi = hdmi;
}

void GPIOLatte::reset() {}

uint32_t GPIOLatte::read() {
	return hdmi->check_interrupts() << 4;
}

void GPIOLatte::write(int pin, bool state) {
	Logger::warning("Unknown latte gpio pin write: %i (%b)", pin, state);
}


GPIOController::GPIOController(GPIOGroup *group) {
	this->group = group;
}

void GPIOController::reset() {
	gpio_enable = 0;
	gpio_out = 0;
	gpio_dir = 0;
	gpio_intlvl = 0;
	gpio_intflag = 0;
	gpio_intmask = 0;
	gpio_owner = 0;
	
	group->reset();
}

uint32_t GPIOController::read(uint32_t addr) {
	switch (addr) {
		case LT_GPIOE_OUT: return gpio_out & gpio_owner;
		case LT_GPIOE_DIR: return gpio_dir & gpio_owner;
		case LT_GPIOE_INTLVL: return gpio_intlvl & gpio_owner;
		case LT_GPIOE_INTFLAG: return gpio_intflag & gpio_owner;
		case LT_GPIOE_INTMASK: return gpio_intmask & gpio_owner;
		
		case LT_GPIO_ENABLE: return gpio_enable;
		case LT_GPIO_OUT: return gpio_out;
		case LT_GPIO_DIR: return gpio_dir;
		case LT_GPIO_IN: return group->read() & ~gpio_dir;
		case LT_GPIO_INTLVL: return gpio_intlvl;
		case LT_GPIO_INTFLAG: return gpio_intflag;
		case LT_GPIO_INTMASK: return gpio_intmask;
		case LT_GPIO_OWNER: return gpio_owner;
	}
	
	Logger::warning("Unknown gpio memory read: 0x%08X", addr);
	return 0;
}

void GPIOController::write(uint32_t addr, uint32_t value) {
	//Logger::warning("gpio memory write: 0x%08X %08x", addr, value);
	if (addr == LT_GPIOE_OUT) {
		uint32_t mask = gpio_dir & gpio_enable & gpio_owner;
		for (int i = 0; i < 32; i++) {
			if (mask & (1 << i)) {
				if ((value & (1 << i)) != (gpio_out & (1 << i))) {
					group->write(i, (value >> i) & 1);
				}
			}
		}
		gpio_out = (gpio_out & ~gpio_owner) | (value & gpio_owner);
	}
	else if (addr == LT_GPIOE_DIR) gpio_dir = (gpio_dir & ~gpio_owner) | (value & gpio_owner);
	else if (addr == LT_GPIOE_INTLVL) gpio_intlvl = (gpio_intlvl & ~gpio_owner) | (value & gpio_owner);
	else if (addr == LT_GPIOE_INTFLAG) gpio_intflag &= ~(value & gpio_owner);
	else if (addr == LT_GPIOE_INTMASK) gpio_intmask = (gpio_intmask & ~gpio_owner) | (value & gpio_owner);
	
	else if (addr == LT_GPIO_ENABLE) gpio_enable = value;
	else if (addr == LT_GPIO_OUT) {
		uint32_t mask = gpio_dir & gpio_enable & ~gpio_owner;
		for (int i = 0; i < 32; i++) {
			if (mask & (1 << i)) 
			{
				if ((value & (1 << i)) != (gpio_out & (1 << i))) {
					group->write(i, (value >> i) & 1);
				}
			}
		}
		gpio_out = (gpio_out & gpio_owner) | (value & ~gpio_owner);
	}
	else if (addr == LT_GPIO_DIR) gpio_dir = (gpio_dir & gpio_owner) | (value & ~gpio_owner);
	else if (addr == LT_GPIO_INTLVL) gpio_intlvl = (gpio_intlvl & gpio_owner) | (value & ~gpio_owner);
	else if (addr == LT_GPIO_INTFLAG) gpio_intflag &= ~value;
	else if (addr == LT_GPIO_INTMASK) gpio_intmask = (gpio_intmask & gpio_owner) | (value & ~gpio_owner);
	else if (addr == LT_GPIO_OWNER) gpio_owner = value;
	else {
		Logger::warning("Unknown gpio memory write: 0x%08X", addr);
	}
}

void GPIOController::update() {
	gpio_intflag |= ~(gpio_intlvl ^ group->read());
}

bool GPIOController::check_interrupts(bool ppc) {
	uint32_t owner = ppc ? gpio_owner : ~gpio_owner;
	return gpio_intflag & gpio_intmask & owner;
}
