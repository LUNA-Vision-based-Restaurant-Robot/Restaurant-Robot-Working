void retrieve_data_from_eeprom() {
    // Retrieve left variable from EEPROM
    uint8_t left = eeprom_read_byte((const uint8_t*)LEFT_ADDR);

	//save the value to global variable
	left_count = left;

    // Retrieve table_metadata from EEPROM
    uint16_t metadata_addr = METADATA_START_ADDR;
    for (uint8_t i = 0; i < MAX_TABLE_COUNT; i++) {
        uint16_t table_number = eeprom_read_word((const uint16_t*)(metadata_addr + i*6));
        uint16_t start_junction = eeprom_read_word((const uint16_t*)(metadata_addr + i*6 + 2));
        int16_t count = eeprom_read_word((const uint16_t*)(metadata_addr + i*6 + 4));

		table_metadata[i][0] = start_junction;
		table_metadata[i][1] = count;
    }
}
