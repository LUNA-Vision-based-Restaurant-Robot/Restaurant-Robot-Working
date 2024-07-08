
#include <avr/eeprom.h>

#define LEFT_ADDR 0x00
#define METADATA_START_ADDR 0x02

void save_data_to_eeprom() {
    // Save left variable to EEPROM
    eeprom_update_byte((uint8_t*)LEFT_ADDR, 0);

    // Save table_metadata to EEPROM
    uint16_t metadata_addr = METADATA_START_ADDR;

    eeprom_update_word((uint16_t*)(metadata_addr + 0*6), 1);
    eeprom_update_word((uint16_t*)(metadata_addr + 0*6 + 2), 0);
    eeprom_update_word((uint16_t*)(metadata_addr + 0*6 + 4), 1);

    eeprom_update_word((uint16_t*)(metadata_addr + 1*6), 2);
    eeprom_update_word((uint16_t*)(metadata_addr + 1*6 + 2), 0);
    eeprom_update_word((uint16_t*)(metadata_addr + 1*6 + 4), 2);

    eeprom_update_word((uint16_t*)(metadata_addr + 2*6), 3);
    eeprom_update_word((uint16_t*)(metadata_addr + 2*6 + 2), 1);
    eeprom_update_word((uint16_t*)(metadata_addr + 2*6 + 4), 1);

}

void retrieve_data_from_eeprom() {
    // Retrieve left variable from EEPROM
    uint8_t left = eeprom_read_byte((const uint8_t*)LEFT_ADDR);

    // Retrieve table_metadata from EEPROM
    uint16_t metadata_addr = METADATA_START_ADDR;
    for (uint8_t i = 0; i < 3; i++) {
        uint16_t table_number = eeprom_read_word((const uint16_t*)(metadata_addr + i*6));
        uint16_t start_junction = eeprom_read_word((const uint16_t*)(metadata_addr + i*6 + 2));
        int16_t count = eeprom_read_word((const uint16_t*)(metadata_addr + i*6 + 4));
        // Use the retrieved data as needed
    }
}

int main(void) {
    save_data_to_eeprom();
    retrieve_data_from_eeprom();

    while (1) {
        // Main loop
    }
}
