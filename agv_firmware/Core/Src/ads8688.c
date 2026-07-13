/**
 * @file    ads8688.c
 * @brief   ADS8688 16-bit SAR ADC driver (Software SPI bit-bang)
 * @note    Giao tiếp SPI Mode 0 (CPOL=0, CPHA=0) bằng bit-bang GPIO.
 *          - CLK  idle LOW, data được lấy mẫu trên cạnh lên (rising edge)
 *          - MSB first
 *          - CS active LOW
 *
 *          Giao thức ADS8688 (32-bit frame per CS cycle):
 *          ┌──────────────────────────────────────────────────────┐
 *          │  Bit 31..16 (MOSI)  │  Bit 15..0 (MOSI = don't care)│
 *          │  = Command/Address  │                                │
 *          ├──────────────────────────────────────────────────────┤
 *          │  Bit 31..16 (MISO)  │  Bit 15..0 (MISO)             │
 *          │  = output data      │  = ADC conversion data         │
 *          └──────────────────────────────────────────────────────┘
 *
 * @author  AGV Firmware Team
 */

#include "ads8688.h"

/* ======================== PRIVATE: TIMING DELAY ============================ */

/**
 * @brief  Delay ngắn ~500ns cho software SPI timing.
 *         STM32H5 @ 250MHz → 1 NOP = 4ns → 125 NOPs ≈ 500ns.
 *         Tần số SPI ≈ 1/(2×500ns) = 1MHz, an toàn cho ADS8688 (max 17MHz).
 */
static void ads8688_delay(void) {
    for (volatile int i = 0; i < 250; i++) {
        __asm("nop");
    }
}

/* ======================== PRIVATE: GPIO BIT-BANG SPI ======================= */

/**
 * @brief  Kéo CS xuống LOW để bắt đầu giao tiếp.
 */
static inline void ads8688_cs_low(void) {
    HAL_GPIO_WritePin(T_ADC_CS_GPIO_Port, T_ADC_CS_Pin, GPIO_PIN_RESET);
    ads8688_delay();
}

/**
 * @brief  Kéo CS lên HIGH để kết thúc giao tiếp.
 */
static inline void ads8688_cs_high(void) {
    ads8688_delay();
    HAL_GPIO_WritePin(T_ADC_CS_GPIO_Port, T_ADC_CS_Pin, GPIO_PIN_SET);
    ads8688_delay();
}

/**
 * @brief  Kéo CLK xuống LOW.
 */
static inline void ads8688_clk_low(void) {
    HAL_GPIO_WritePin(T_ADC_CLK_GPIO_Port, T_ADC_CLK_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Kéo CLK lên HIGH.
 */
static inline void ads8688_clk_high(void) {
    HAL_GPIO_WritePin(T_ADC_CLK_GPIO_Port, T_ADC_CLK_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Ghi bit ra chân SDI (MOSI).
 * @param  bit Giá trị bit (0 hoặc 1)
 */
static inline void ads8688_sdi_write(uint8_t bit) {
    HAL_GPIO_WritePin(T_ADC_SDI_GPIO_Port, T_ADC_SDI_Pin,
                      bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Đọc bit từ chân SDO (MISO).
 * @return 0 hoặc 1
 */
static inline uint8_t ads8688_sdo_read(void) {
    return (HAL_GPIO_ReadPin(T_ADC_SDO_GPIO_Port, T_ADC_SDO_Pin) == GPIO_PIN_SET)
               ? 1
               : 0;
}

/**
 * @brief  Truyền/nhận 1 byte qua software SPI (MSB first, Mode 1).
 *         - CLK idle LOW (CPOL = 0)
 *         - Setup data trên cạnh lên (rising edge)
 *         - Lấy mẫu data trên cạnh xuống (falling edge) (CPHA = 1)
 * @param  tx_byte  Byte cần gửi (MOSI)
 * @return Byte nhận về (MISO)
 */
static uint8_t ads8688_spi_transfer_byte(uint8_t tx_byte) {
    uint8_t rx_byte = 0;

    for (int8_t bit = 7; bit >= 0; bit--) {
        /* Cạnh lên (Rising Edge) - Thay đổi dữ liệu MOSI */
        ads8688_clk_high();
        ads8688_sdi_write((tx_byte >> bit) & 0x01);
        ads8688_delay();

        /* Cạnh xuống (Falling Edge) - Lấy mẫu dữ liệu MISO */
        ads8688_clk_low();
        ads8688_delay();
        rx_byte |= (ads8688_sdo_read() << bit);
    }

    return rx_byte;
}

/**
 * @brief  Truyền/nhận 16-bit qua software SPI.
 * @param  tx_word  16-bit cần gửi (MSB first)
 * @return 16-bit nhận về
 */
static uint16_t ads8688_spi_transfer_16(uint16_t tx_word) {
    uint16_t rx = 0;
    rx = (uint16_t)ads8688_spi_transfer_byte((uint8_t)(tx_word >> 8)) << 8;
    rx |= (uint16_t)ads8688_spi_transfer_byte((uint8_t)(tx_word & 0xFF));
    return rx;
}

/**
 * @brief  Thực hiện 1 SPI frame 32-bit (giao thức chuẩn ADS8688).
 *
 *         Frame 32-bit:
 *         - MOSI[31:16] = command word (16-bit)
 *         - MOSI[15:0]  = don't care (gửi 0x0000)
 *         - MISO[31:16] = output data (channel ID, flags...)
 *         - MISO[15:0]  = ADC conversion result
 *
 * @param  cmd       16-bit command word (gửi trong nửa đầu)
 * @param  adc_data  Con trỏ nhận ADC data 16-bit (nửa sau MISO). NULL nếu không cần.
 * @return 16-bit nửa đầu MISO (output data / status)
 */
static uint16_t ads8688_send_command(uint16_t cmd, uint16_t *adc_data) {
    uint16_t rx_high, rx_low;

    ads8688_cs_low();

    /* Nửa đầu: gửi command, nhận output data */
    rx_high = ads8688_spi_transfer_16(cmd);

    /* Nửa sau: gửi don't care, nhận ADC data */
    rx_low = ads8688_spi_transfer_16(0x0000);

    ads8688_cs_high();

    if (adc_data != NULL) {
        *adc_data = rx_low;
    }

    return rx_high;
}

/* ======================== PRIVATE: VOLTAGE CONVERSION ====================== */

/**
 * @brief  Lấy hệ số Full-Scale Voltage tương ứng với mã dải đo.
 *         ADS8688 sử dụng VREF nội 4.096V.
 * @param  range  Mã dải đo (ADS8688_RANGE_xxx)
 * @return Full-scale voltage (V) cho dải unipolar, hoặc half-scale cho bipolar.
 */
static float ads8688_get_full_scale(uint8_t range) {
    switch (range) {
    case ADS8688_RANGE_P_2_5_VREF:    /* 0 ~ 2.5×VREF = 0-10.24V */
        return 10.24f;
    case ADS8688_RANGE_P_1_25_VREF:   /* 0 ~ 1.25×VREF = 0-5.12V */
        return 5.12f;
    case ADS8688_RANGE_PM_2_5_VREF:   /* ±2.5×VREF = ±10.24V */
        return 20.48f;                /* Full range = 20.48V */
    case ADS8688_RANGE_PM_1_25_VREF:  /* ±1.25×VREF = ±5.12V */
        return 10.24f;
    case ADS8688_RANGE_PM_0_625_VREF: /* ±0.625×VREF = ±2.56V */
        return 5.12f;
    default:
        return 10.24f;               /* Mặc định 0-10V */
    }
}

/**
 * @brief  Chuyển đổi giá trị ADC thô sang điện áp thực (V).
 * @param  raw    Giá trị ADC 16-bit
 * @param  range  Mã dải đo đang cấu hình
 * @return Điện áp (V)
 */
static float ads8688_raw_to_voltage(uint16_t raw, uint8_t range) {
    float full_scale = ads8688_get_full_scale(range);

    /* Dải unipolar: V = raw / 65535 × FullScale */
    if (range == ADS8688_RANGE_P_2_5_VREF ||
        range == ADS8688_RANGE_P_1_25_VREF) {
        return ((float)raw / 65535.0f) * full_scale;
    }

    /* Dải bipolar: dữ liệu 16-bit dạng Two's Complement
     * V = (int16_t)raw / 32768 × HalfScale */
    int16_t signed_raw = (int16_t)raw;
    return ((float)signed_raw / 32768.0f) * (full_scale / 2.0f);
}

/* ======================== PUBLIC API ======================================= */

void ADS8688_Init(ADS8688_HandleTypeDef *hadc, uint8_t range) {
    if (hadc == NULL) {
        return;
    }

    /* Khởi tạo trạng thái GPIO ban đầu */
    ads8688_cs_high();     /* CS HIGH = không chọn chip */
    ads8688_clk_low();     /* CLK LOW = idle (SPI Mode 0) */
    ads8688_sdi_write(0);  /* SDI LOW */

    /* === Hardware Reset === */
    HAL_GPIO_WritePin(T_ADC_RST_GPIO_Port, T_ADC_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);  /* Giữ RST LOW tối thiểu 50ns (datasheet), dùng 1ms cho an toàn */
    HAL_GPIO_WritePin(T_ADC_RST_GPIO_Port, T_ADC_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);  /* Đợi chip khởi động sau reset */

    /* === Software Reset để chắc chắn thanh ghi về giá trị mặc định === */
    ads8688_send_command(ADS8688_CMD_RST, NULL);
    HAL_Delay(5);

    /* === Cấu hình dải đo cho CH0 và CH1 === */
    ADS8688_WriteReg(ADS8688_REG_CH0_INPUT_RANGE, range);
    ADS8688_WriteReg(ADS8688_REG_CH1_INPUT_RANGE, range);

    /* Lưu dải đo vào handle */
    for (int i = 0; i < ADS8688_NUM_CHANNELS; i++) {
        hadc->raw[i] = 0;
        hadc->voltage[i] = 0.0f;
        hadc->range[i] = range;
    }

    /* Power Down các kênh không sử dụng (CH2..CH7) để giảm nhiễu và tiêu thụ */
    /* Bit mask: bit[7:0] tương ứng CH7..CH0, bit=1 → power down */
    ADS8688_WriteReg(ADS8688_REG_CH_PWR_DN, 0xFC); /* 1111_1100 = tắt CH2-CH7 */

    /* Đọc thử 1 lần để ổn định pipeline (giá trị đầu tiên thường không chính xác) */
    ads8688_send_command(ADS8688_CMD_MAN_CH0, NULL);
    ads8688_send_command(ADS8688_CMD_NO_OP, NULL);

    hadc->initialized = true;
}

uint16_t ADS8688_ReadRaw(ADS8688_HandleTypeDef *hadc, uint8_t channel) {
    if (hadc == NULL || channel >= ADS8688_NUM_CHANNELS) {
        return 0;
    }

    /* Bảng lệnh Manual Channel Select cho từng kênh */
    static const uint16_t man_cmd[ADS8688_NUM_CHANNELS] = {
        ADS8688_CMD_MAN_CH0, ADS8688_CMD_MAN_CH1,
        ADS8688_CMD_MAN_CH2, ADS8688_CMD_MAN_CH3,
        ADS8688_CMD_MAN_CH4, ADS8688_CMD_MAN_CH5,
        ADS8688_CMD_MAN_CH6, ADS8688_CMD_MAN_CH7,
    };

    /* Frame 1: Gửi lệnh chọn kênh → ADS8688 bắt đầu chuyển đổi kênh được chọn */
    ads8688_send_command(man_cmd[channel], NULL);

    /* Frame 2: Gửi NO_OP để đọc kết quả chuyển đổi của kênh vừa chọn */
    uint16_t adc_data = 0;
    ads8688_send_command(ADS8688_CMD_NO_OP, &adc_data);

    hadc->raw[channel] = adc_data;
    return adc_data;
}

float ADS8688_ReadVoltage(ADS8688_HandleTypeDef *hadc, uint8_t channel) {
    if (hadc == NULL || channel >= ADS8688_NUM_CHANNELS) {
        return 0.0f;
    }

    uint16_t raw = ADS8688_ReadRaw(hadc, channel);
    float voltage = ads8688_raw_to_voltage(raw, hadc->range[channel]);
    hadc->voltage[channel] = voltage;

    return voltage;
}

void ADS8688_ReadAll(ADS8688_HandleTypeDef *hadc) {
    if (hadc == NULL) {
        return;
    }

    /* Đọc lần lượt CH0 và CH1 */
    ADS8688_ReadVoltage(hadc, ADS8688_CH0);
    ADS8688_ReadVoltage(hadc, ADS8688_CH1);
}

void ADS8688_SetChannelRange(ADS8688_HandleTypeDef *hadc, uint8_t channel,
                             uint8_t range) {
    if (hadc == NULL || channel >= ADS8688_NUM_CHANNELS) {
        return;
    }

    /* Địa chỉ thanh ghi Input Range bắt đầu từ 0x05 cho CH0 */
    uint8_t reg_addr = ADS8688_REG_CH0_INPUT_RANGE + channel;
    ADS8688_WriteReg(reg_addr, range);
    hadc->range[channel] = range;
}

void ADS8688_WriteReg(uint8_t reg_addr, uint8_t data) {
    /*
     * Giao thức ghi thanh ghi ADS8688 (Program Register Write):
     * - MOSI[31:25] = (reg_addr << 1) | 0x01  → 7-bit address + W bit
     * - MOSI[24:17] = data                     → 8-bit register value
     * - MOSI[16:0]  = don't care
     *
     * Tổng cộng 32-bit = 4 bytes.
     * Byte 0: (reg_addr << 1) | 0x01
     * Byte 1: data
     * Byte 2-3: 0x00 (don't care)
     */
    uint16_t cmd_word = (uint16_t)(((reg_addr << 1) | 0x01) << 8) | data;

    ads8688_cs_low();
    ads8688_spi_transfer_16(cmd_word);
    ads8688_spi_transfer_16(0x0000); /* Hoàn thành frame 32-bit */
    ads8688_cs_high();
}

uint8_t ADS8688_ReadReg(uint8_t reg_addr) {
    /*
     * Giao thức đọc thanh ghi ADS8688 (Program Register Read - 2 cycles):
     * 
     * Chu kỳ 1: Gửi lệnh đọc thanh ghi (Register Read Command)
     * - MOSI[31:25] = (reg_addr << 1) | 0x00  → 7-bit address + R bit
     * - MOSI[24:0]  = don't care (gửi 0)
     */
    uint16_t cmd_word = (uint16_t)((reg_addr << 1) & 0xFE) << 8;

    ads8688_cs_low();
    ads8688_spi_transfer_16(cmd_word);
    ads8688_spi_transfer_16(0x0000); /* Hoàn thành frame 32-bit thứ nhất */
    ads8688_cs_high();

    ads8688_delay(); // Khoảng nghỉ ngắn giữa 2 chu kỳ

    /*
     * Chu kỳ 2: Gửi lệnh NO_OP (0x00000000) để đọc dữ liệu trả về
     * - MISO[23:16] = register data (nằm ở byte thấp của nửa đầu rx_high)
     */
    ads8688_cs_low();
    uint16_t rx_high = ads8688_spi_transfer_16(ADS8688_CMD_NO_OP);
    ads8688_spi_transfer_16(0x0000); /* Hoàn thành frame 32-bit thứ hai */
    ads8688_cs_high();

    return (uint8_t)(rx_high & 0xFF);
}
