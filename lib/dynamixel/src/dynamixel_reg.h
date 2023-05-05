#ifndef DYNAMIXEL_REGS_H
#define DYNAMIXEL_REGS_H

/**
 * @brief Decodes the received EEPROM response from a packet generated with `dynamixel_reg_eeprom_gen_request` into the eeprom buffer.
 *
 * @param response The response to decode
 * @param eeprom Pointer to dynamixel_eeprom object to populate
 */
void dynamixel_reg_eeprom_decode(InfoToParseDXLPacket_t *response, struct dynamixel_eeprom *eeprom);

/**
 * @brief Create a new EEPROM read request
 *
 * @param packet Packet to populate
 * @param packet_buf Buffer for parameters
 * @param id Dynamixel ID to read EEPROM from
 * @return enum DXLLibErrorCode The result of the request generation
 */
enum DXLLibErrorCode dynamixel_reg_eeprom_gen_request(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf, dynamixel_id id);

/**
 * @brief Decodes the received RAM response from a packet generated with `dynamixel_reg_ram_gen_request` into the ram buffer.
 *
 * @param response The response to decode
 * @param ram Pointer to dynamixel_ram object to populate
 */
void dynamixel_reg_ram_decode(InfoToParseDXLPacket_t *response, struct dynamixel_ram *ram);

/**
 * @brief Create a new ram read request
 *
 * @param packet Packet to populate
 * @param packet_buf Buffer for parameters
 * @param id Dynamixel ID to read RAM from
 * @return enum DXLLibErrorCode The result of the request generation
 */
enum DXLLibErrorCode dynamixel_reg_ram_gen_request(InfoToMakeDXLPacket_t *packet, uint8_t *packet_buf, dynamixel_id id);

#endif