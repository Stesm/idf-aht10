#include <stdio.h>
#include <aht10.h>
#include <driver/i2c.h>
#include <esp_log.h>

#define I2C_CMD(err, from) if (err != ESP_OK) {ESP_LOGE("I2C ERROR: ", "%s, %s", from, esp_err_to_name(err));}

#define AHT10_ADDRESS           0x38
#define AHT10_TRIGGER_MEASURE   0b10101100
#define AHT10_NORMAL_MODE       0b00110011
#define AHT10_NOP               0b00000000

static void vAHTRead();
static uint8_t readBit(uint8_t value, uint8_t bit_num);
static char *int_to_bits(uint8_t value, char *buff);
static void vI2CCmd(uint8_t commands[3], int delay);
static esp_err_t xI2CRead(uint8_t *data, uint8_t len);

uint8_t measure[6] = {0, 0, 0, 0, 0, 0};
uint32_t raw_data[3] = {0, 0, 0};
TickType_t latst_read = 0;

void vAHTStatus() {
    uint8_t data;

    xI2CRead(&data, 1);
    ESP_LOGI("STATUS", "Bisy bit - %d, calibration bit - %d", readBit(data, 7), readBit(data, 3));
}

void vAHTInit() 
{
    vTaskDelay(40/portTICK_RATE_MS);

    uint8_t normalMode[3] = {0xE1, 0x08, 0x00};
    vI2CCmd(normalMode, 350);

    vAHTStatus();
}

float fAHTTempetarure() 
{
    vAHTRead();
    if (raw_data[2] == 0) {
        return 0.0;
    }
 
    return ((float) raw_data[2] * 200 / 0x100000) - 50;
}

float fAHTHumidity() 
{
    vAHTRead();
    if (raw_data[1] == 0) {
        return 0.0;
    }
 
    return ((float) raw_data[1] / 0x100000) * 100;
}

static void vAHTStartMeasure()
{
    uint8_t command[3] = {0xAC, 0x33, 0x00};
    vI2CCmd(command, 150);
}

static void vAHTRead() {
    if (latst_read != 0 && (latst_read + 1000) > xTaskGetTickCount()) {
        ESP_LOGI("Measure", "Shown cached data");
        return;
    }

    vAHTStartMeasure();
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};

    esp_err_t measure_read = xI2CRead(data, 6);
    uint8_t *calc_data;
    char buff[9];

    if (measure_read != ESP_OK) {
        calc_data = measure;
    } else {
        calc_data = data;
        for (uint8_t i = 0; i < 6; i++) {
            measure[i] = data[i];
        }
    }

    raw_data[0] = calc_data[0];
    raw_data[1] = (((uint32_t)(calc_data[1]) << 12)) | (((uint32_t) calc_data[2]) << 4) | ((calc_data[3] & 0xF0) >> 4);
    raw_data[2] = ((uint32_t)(calc_data[3] & 0x0F) << 16) | ((uint16_t)calc_data[4] << 8) | calc_data[5];
    
    ESP_LOGI(
        "Measure:", "[t:%f: h: %f] - [%s]", 
        ((float) raw_data[2] * 200 / 0x100000) - 50, 
        ((float) raw_data[1] / 0x100000) * 100, 
        int_to_bits(raw_data[0], buff)
    );

    latst_read = xTaskGetTickCount();
}

static uint8_t readBit(uint8_t value, uint8_t bit_num) {
    if (bit_num > 7) {
        ESP_LOGE("ERROR", "Unsupported bit number, must be bitween 0 and 7");
        return 0;
    } 

    return (value & (1 << bit_num)) ? 1 : 0;
}

static char *int_to_bits(uint8_t value, char *buff)
{
    for (uint8_t idx = 0; idx < 8; idx++) {
        buff[idx] = readBit(value, (7 - idx)) == 1 ? '1' : '0';
    }
    buff[8] = '\0';

    return buff;
}

static void vI2CCmd(uint8_t commands[3], int delay) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    I2C_CMD(i2c_master_start(cmd), "Start");
    I2C_CMD(i2c_master_write_byte(cmd, (AHT10_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK), "Write");
    for (uint8_t i = 0; i < 3; i++) {
        I2C_CMD(i2c_master_write_byte(cmd, commands[i], I2C_MASTER_ACK), "Write");
    }
    I2C_CMD(i2c_master_stop(cmd), "Stop");
    I2C_CMD(i2c_master_cmd_begin(I2C_NUM_0, cmd, 5000), "Exec send");

    i2c_cmd_link_delete(cmd);
    vTaskDelay(delay/portTICK_RATE_MS);
}

static esp_err_t xI2CRead(uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    I2C_CMD(i2c_master_start(cmd), "Start");
    I2C_CMD(i2c_master_write_byte(cmd, (AHT10_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK), "Write");
    for (uint8_t i = 0; i < len; i++) {
        I2C_CMD(i2c_master_read_byte(cmd, (data + i), I2C_MASTER_ACK), "Read");
    }
    I2C_CMD(i2c_master_stop(cmd), "Stop");
    esp_err_t exec = i2c_master_cmd_begin(I2C_NUM_0, cmd, 5000);
    I2C_CMD(exec, "Exec read");
    i2c_cmd_link_delete(cmd);

    return exec;
}