#define D818_ADDR 0x2F

#define D818_REG_CONFIG_ADDR 0x00
#define D818_IRQ_MASK_ADDR 0x03
#define D818_REG_ONESHOT_ADDR 0x09
#define D818_REG_ADV_CFG_ADDR 0x0B
#define D818_REG_BUSY_ADDR 0x0C
#define D818_REG_READING_ADDR 0x20

#define D818_CHANNEL_NUM 8


void D818_init();
void D818_read();
uint16_t D818_query(uint8_t chan);