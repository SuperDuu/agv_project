/**
 * @file    ads8688.h
 * @brief   ADS8688 16-bit SAR ADC driver (Software SPI bit-bang)
 * @note    IC: ADS8688 (Texas Instruments)
 *          - 8 channels, 16-bit resolution, 500kSPS max
 *          - Software SPI on: CLK=PB13, SDI=PB15, SDO=PB14, CS=PG12, RST=PG13
 *          - Input range: 0-5V hoặc 0-10V (cấu hình qua thanh ghi)
 *
 * @author  AGV Firmware Team
 */

#ifndef ADS8688_H
#define ADS8688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ======================== PIN MAPPING (from CubeMX) ======================== */
/* Các chân đã được cấu hình GPIO trong MX_GPIO_Init():
 *   T_ADC_CLK  = PB13  (Output PP)  -> SPI Clock
 *   T_ADC_SDI  = PB15  (Output PP)  -> Master Out / Slave In (MOSI)
 *   T_ADC_SDO  = PB14  (Input)      -> Master In / Slave Out (MISO)
 *   T_ADC_CS   = PG12  (Output PP)  -> Chip Select (active LOW)
 *   T_ADC_RST  = PG13  (Output PP)  -> Hardware Reset (active LOW)
 */

/* ======================== COMMAND REGISTER (16-bit) ======================== */
/* Program Register (upper byte of 16-bit command word)
 * Bit[15:9] = Command, Bit[8:0] = 0
 * Xem ADS8688 Datasheet - Table 4: Command Register
 */
#define ADS8688_CMD_NO_OP           0x0000  /**< Tiếp tục chế độ hiện tại, không thay đổi */
#define ADS8688_CMD_STDBY           0x8200  /**< Chuyển sang chế độ Standby */
#define ADS8688_CMD_PWR_DN          0x8300  /**< Chuyển sang chế độ Power Down */
#define ADS8688_CMD_RST             0x8500  /**< Software Reset toàn bộ thanh ghi */
#define ADS8688_CMD_AUTO_RST        0xA000  /**< Auto Scan + Reset con trỏ sequence */
#define ADS8688_CMD_MAN_CH0         0xC000  /**< Manual select Channel 0 */
#define ADS8688_CMD_MAN_CH1         0xC400  /**< Manual select Channel 1 */
#define ADS8688_CMD_MAN_CH2         0xC800  /**< Manual select Channel 2 */
#define ADS8688_CMD_MAN_CH3         0xCC00  /**< Manual select Channel 3 */
#define ADS8688_CMD_MAN_CH4         0xD000  /**< Manual select Channel 4 */
#define ADS8688_CMD_MAN_CH5         0xD400  /**< Manual select Channel 5 */
#define ADS8688_CMD_MAN_CH6         0xD800  /**< Manual select Channel 6 */
#define ADS8688_CMD_MAN_CH7         0xDC00  /**< Manual select Channel 7 */

/* ======================== PROGRAM REGISTER ADDRESSES ======================= */
/* Thanh ghi cấu hình nội bộ (8-bit address, truy cập qua lệnh Write)
 * Xem ADS8688 Datasheet - Table 5: Program Register Map
 */
#define ADS8688_REG_AUTO_SEQ_EN     0x01  /**< Auto Scan Sequence Enable (bit mask cho 8 kênh) */
#define ADS8688_REG_CH_PWR_DN       0x02  /**< Channel Power Down (bit mask) */
#define ADS8688_REG_FEATURE_SEL     0x03  /**< Feature Select: alarm, SDO tri-state... */

/* Channel Input Range registers (mỗi kênh 1 thanh ghi riêng) */
#define ADS8688_REG_CH0_INPUT_RANGE 0x05  /**< Channel 0 Input Range */
#define ADS8688_REG_CH1_INPUT_RANGE 0x06  /**< Channel 1 Input Range */
#define ADS8688_REG_CH2_INPUT_RANGE 0x07  /**< Channel 2 Input Range */
#define ADS8688_REG_CH3_INPUT_RANGE 0x08  /**< Channel 3 Input Range */
#define ADS8688_REG_CH4_INPUT_RANGE 0x09  /**< Channel 4 Input Range */
#define ADS8688_REG_CH5_INPUT_RANGE 0x0A  /**< Channel 5 Input Range */
#define ADS8688_REG_CH6_INPUT_RANGE 0x0B  /**< Channel 6 Input Range */
#define ADS8688_REG_CH7_INPUT_RANGE 0x0C  /**< Channel 7 Input Range */

/* ======================== INPUT RANGE CODES ================================ */
/* Giá trị ghi vào thanh ghi CHx_INPUT_RANGE để chọn dải đo
 * Xem ADS8688 Datasheet - Table 7
 */
#define ADS8688_RANGE_PM_2_5_VREF   0x00  /**< ±2.5  × VREF (±10.24V nếu VREF=4.096V) */
#define ADS8688_RANGE_PM_1_25_VREF  0x01  /**< ±1.25 × VREF (±5.12V) */
#define ADS8688_RANGE_PM_0_625_VREF 0x02  /**< ±0.625× VREF (±2.56V) */
#define ADS8688_RANGE_P_2_5_VREF    0x05  /**< 0 ~ 2.5 × VREF  (0-10.24V) */
#define ADS8688_RANGE_P_1_25_VREF   0x06  /**< 0 ~ 1.25 × VREF (0-5.12V)  */

/* Alias thân thiện cho ứng dụng */
#define ADS8688_RANGE_0_10V         ADS8688_RANGE_P_2_5_VREF   /**< 0-10V unipolar */
#define ADS8688_RANGE_0_5V          ADS8688_RANGE_P_1_25_VREF  /**< 0-5V unipolar  */

/* ======================== CHANNEL DEFINITIONS ============================== */
#define ADS8688_CH0                 0
#define ADS8688_CH1                 1
#define ADS8688_CH2                 2
#define ADS8688_CH3                 3
#define ADS8688_CH4                 4
#define ADS8688_CH5                 5
#define ADS8688_CH6                 6
#define ADS8688_CH7                 7
#define ADS8688_NUM_CHANNELS        8

/* ======================== DATA STRUCTURE =================================== */

/**
 * @brief Cấu trúc quản lý ADS8688
 */
typedef struct {
    uint16_t raw[ADS8688_NUM_CHANNELS];       /**< Giá trị ADC thô 16-bit của mỗi kênh */
    float    voltage[ADS8688_NUM_CHANNELS];   /**< Giá trị điện áp đã quy đổi (V) */
    uint8_t  range[ADS8688_NUM_CHANNELS];     /**< Mã dải đo hiện tại của mỗi kênh */
    bool     initialized;                     /**< Cờ đánh dấu đã khởi tạo thành công */
} ADS8688_HandleTypeDef;

/* ======================== PUBLIC API ======================================= */

/**
 * @brief  Khởi tạo ADS8688: Hardware reset, software reset, cấu hình dải đo cho CH0 và CH1.
 * @param  hadc  Con trỏ tới handle ADS8688
 * @param  range Mã dải đo áp dụng cho cả CH0 và CH1 (VD: ADS8688_RANGE_0_10V)
 * @note   Gọi sau MX_GPIO_Init(). Các kênh khác sẽ bị Power Down để tiết kiệm năng lượng.
 */
void ADS8688_Init(ADS8688_HandleTypeDef *hadc, uint8_t range);

/**
 * @brief  Đọc giá trị ADC thô (16-bit) từ 1 kênh bằng Manual Channel Select.
 * @param  hadc    Con trỏ tới handle ADS8688
 * @param  channel Kênh cần đọc (ADS8688_CH0 ... ADS8688_CH7)
 * @return Giá trị ADC 16-bit (0x0000 ~ 0xFFFF)
 */
uint16_t ADS8688_ReadRaw(ADS8688_HandleTypeDef *hadc, uint8_t channel);

/**
 * @brief  Đọc giá trị điện áp (V) từ 1 kênh, tự quy đổi theo dải đo đã cấu hình.
 * @param  hadc    Con trỏ tới handle ADS8688
 * @param  channel Kênh cần đọc (ADS8688_CH0 ... ADS8688_CH7)
 * @return Điện áp (V). Trả về 0.0f nếu tham số không hợp lệ.
 */
float ADS8688_ReadVoltage(ADS8688_HandleTypeDef *hadc, uint8_t channel);

/**
 * @brief  Đọc cả 2 kênh AD0, AD1 cùng lúc và cập nhật vào handle.
 * @param  hadc Con trỏ tới handle ADS8688
 * @note   Sau khi gọi, truy cập kết quả qua hadc->raw[0..1] và hadc->voltage[0..1]
 */
void ADS8688_ReadAll(ADS8688_HandleTypeDef *hadc);

/**
 * @brief  Cấu hình dải đo (Input Range) cho 1 kênh cụ thể.
 * @param  hadc    Con trỏ tới handle ADS8688
 * @param  channel Kênh cần cấu hình (ADS8688_CH0 ... ADS8688_CH7)
 * @param  range   Mã dải đo (VD: ADS8688_RANGE_0_10V, ADS8688_RANGE_0_5V)
 */
void ADS8688_SetChannelRange(ADS8688_HandleTypeDef *hadc, uint8_t channel,
                             uint8_t range);

/**
 * @brief  Ghi 1 byte vào thanh ghi nội bộ (Program Register) của ADS8688.
 * @param  reg_addr Địa chỉ thanh ghi (VD: ADS8688_REG_CH0_INPUT_RANGE)
 * @param  data     Giá trị cần ghi (8-bit)
 */
void ADS8688_WriteReg(uint8_t reg_addr, uint8_t data);

/**
 * @brief  Đọc 1 byte từ thanh ghi nội bộ (Program Register) của ADS8688.
 * @param  reg_addr Địa chỉ thanh ghi
 * @return Giá trị 8-bit đọc được
 */
uint8_t ADS8688_ReadReg(uint8_t reg_addr);

#ifdef __cplusplus
}
#endif

#endif /* ADS8688_H */
