/**
 * Private configuration file for the SSD1306 library.
 */

#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// Choose a bus
#define SSD1306_USE_I2C

// I2C Configuration
#define SSD1306_I2C_PORT        I2C2
#define SSD1306_I2C_ADDR        (0x3C << 1)

// Include only needed fonts
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18

#endif /* __SSD1306_CONF_H__ */
