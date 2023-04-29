////
//// Created by 16771 on 2022/12/6.
////
//
//#include "bsp_Power_Detect.h"
//#include "i2c.h"
//#include "stm32f4xx_hal_i2c.h"
//#include "freertos.h"
//#include "queue.h"
//#include "semphr.h"
//#include "task.h"
//#include "timers.h"
//#include "string.h"
//uint32_t ina219_currentDivider_mA;
//uint32_t ina219_powerMultiplier_mW;
//uint32_t ina219_calValue;
//#define BUFFERLEN 4096
//int16_t contBuffer[BUFFERLEN];
//unsigned int bufferPos;
///* Holds currently type of continuous measurement (voltage, power, current).
// * Used to avoid floating point numbers prolification */
//int measureType;
//uint8_t adrs_219 = 0x40; // you can call this from main function when necessary especially when using multiple INA219 on board
//void Ina219_Write_Reg(uint8_t reg, uint16_t value);
//void Ina219_Read_Reg(uint8_t reg, uint16_t *value);
//void Set_Calibration_32V_2A(void);
//int16_t Get_BusVoltage_Raw();
//int16_t Get_ShuntVoltage_raw();
//int16_t Get_Current_raw();
//int16_t Get_Power_raw();
//
//void Ina219_Write_Reg(uint8_t reg, uint16_t value){
//    uint8_t i2c_temp[2];
//    i2c_temp[0] = value>>8;
//    i2c_temp[1] = value;
//    HAL_I2C_Mem_Write(&hi2c2, adrs_219<<1, (uint16_t)reg, 1, i2c_temp, 2, 0xffffffff);
//    vTaskDelay(1);
//}
//
//void Ina219_Read_Reg(uint8_t reg, uint16_t *value){
//    uint8_t i2c_temp[2];
//    HAL_I2C_Mem_Read(&hi2c2, adrs_219<<1, (uint16_t)reg, 1,i2c_temp, 2, 0xffffffff);
//    vTaskDelay(1);
//    *value = ((uint16_t)i2c_temp[0]<<8 )|(uint16_t)i2c_temp[1];
//}
//
//void Set_Calibration_32V_2A(void){
//
//    // VBus_Max = 32V
//    // VShunt_Max = 0.32v = 320mv
//    // RShunt=0.008
//
//    // 1.ȷ��������
//    // MaxPossible_I = VShunt_Max/RShunt = 40A
//
//    // 2.ȷ�������������
//    // MaxExpected_I = 5A
//
//    // 3.����LSBs�ķ�Χ(Min = 15-bit, Max = 12-bit)
//    // MinimumLSB = MaxExpected_I/ 2^15-1
//    // MinimumLSB = 0.00015259A     (152uA per bit)
//    // MaximumLSB = MaxExpected_I/ 2^12-1
//    // MaximumLSB = 0.0012207A      (1220uA per bit)
//    // CurrentLSB = 0.0002  (200uA per bit)
//
//    // 5.����У׼�Ĵ�����ֵ
//    // Cal = trunc(0.04096 / (CurrentLSB * RShunt))
//    // cal = 25600(�е��)
//    ina219_calValue = 25600;
//
//    // 6.���㹦�ʵ�LSB
//    // PowerLSB = 20 * CurrentLSB
//    // PowerLSB = 0.004     (4mW per bit)
//
//    // 7.�������(���Ĵ������λ)ǰ�������ͷ��������ѹ
//    // MaxCurrent = CurrentLSB * 2^15-1
//    // MaxCurrent = 6.5534 A
//    // If Max_Current > Max_Possible_I then
//    //    Max_Current_Before_Overflow = MaxPossible_I
//    // Else
//    //    Max_Current_Before_Overflow = Max_Current
//    // End If
//    // MaxShuntVoltage = Max_Current_Before_Overflow * RShunt (6.5534*0.008)
//    // MaxShuntVoltage = 0.0524272 V
//    // If Max_ShuntVoltage >= VShunt_MAX
//    //    Max_ShuntVoltage_Before_Overflow = VShunt_MAX
//    // Else
//    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
//    // End If
//
//    // 8.���������
//    // MaximumPower = Max_Current_Before_Overflow * VBus_MAX
//    // MaximumPower = 6.5534 * 32 = 209.7088 W
//
//    ina219_currentDivider_mA = 5;// Current LSB = 200uA per bit (1000/200 = 5)
//    ina219_powerMultiplier_mW = 1;// Power LSB = 4mW per bit (2/4)
//
//    Ina219_Write_Reg(INA219_REG_CALIBRATION, ina219_calValue);
//    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V
//                      | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT
//                      | INA219_CONFIG_SADCRES_12BIT_1S_532US
//                      | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
//    Ina219_Write_Reg(INA219_REG_CONFIG, config);
//}
//
//int16_t Get_BusVoltage_Raw(){
//    uint16_t value;
//    Ina219_Read_Reg(INA219_REG_BUSVOLTAGE, &value);
//    return (int16_t) ((value >> 3) * 4);
//}
//
//int16_t Get_ShuntVoltage_raw() {
//    uint16_t value;
//    Ina219_Read_Reg(INA219_REG_SHUNTVOLTAGE, &value);
//    return (int16_t) value;
//}
//
//int16_t Get_Current_raw() {
//    uint16_t value;
//    // ��ʱ������һЩ����ֵֹ�������Ina219��ina219_calValue��reset��(�����üĴ����и�resetλ)
//    // ������ĵ�ѹ���ʿ϶��Ǵ�ģ�������Ҫһ��ʱ��ȥ�ֶ�������
//    Ina219_Write_Reg(INA219_REG_CALIBRATION, ina219_calValue);
//    Ina219_Read_Reg(INA219_REG_CURRENT, &value);
//    return (int16_t) value;
//}
//
//int16_t Get_Power_raw() {
//    uint16_t value;
//    // ��ʱ������һЩ����ֵֹ�������Ina219��ina219_calValue��reset��(�����üĴ����и�resetλ)
//    // ������ĵ�ѹ���ʿ϶��Ǵ�ģ�������Ҫһ��ʱ��ȥ�ֶ�������
//    Ina219_Write_Reg(INA219_REG_CALIBRATION, ina219_calValue);
//    // Now we can safely read the POWER register!
//    Ina219_Read_Reg(INA219_REG_POWER, &value);
//    return (int16_t) value;
//}
//
//float ECF_Get_ShuntVoltage_mV() {
//    int16_t value;
//    value = Get_ShuntVoltage_raw();
//    return (value * 0.01);
//}
//
//float ECF_Get_BusVoltage_V() {
//    int16_t value = Get_BusVoltage_Raw();
//    return (value * 0.001);
//}
//
//float ECF_GetCurrent_mA() {
//    float valueDec = Get_Current_raw();
//    valueDec /= ina219_currentDivider_mA;
//    return valueDec;
//}
//
//float ECF_GetPower_mW() {
//    float valueDec = Get_Power_raw();
//    valueDec *= ina219_powerMultiplier_mW;
//    return valueDec;
//}
//
//int contMeasureInit(uint8_t reg) {
//    HAL_StatusTypeDef status;
//    measureType = reg;
//    /* Set register pointer to desired register */
//    status = HAL_I2C_Master_Transmit(&hi2c2, adrs_219 << 1, &reg, 1,
//                                     0xffffffff);
//    //TODO:��Ͷ����ˣ�
//    if (status != HAL_OK)
//        while (1)
//            ;
//    bufferPos = 0;
//    memset(contBuffer, 0, sizeof(contBuffer));
//    return 0;
//}
//
//
//int contMeasureUpdate(void) {
//    HAL_StatusTypeDef status;
//    uint8_t measure[2];
//    status = HAL_I2C_Master_Receive(&hi2c2, adrs_219 << 1,
//                                    (uint8_t*) &measure, 2, 0xffffffff);
//    if (status != HAL_OK)
//        while (1)
//            ;
//    /* Change endinanness */
//    if (bufferPos < BUFFERLEN)
//        contBuffer[bufferPos++] = ina219_powerMultiplier_mW
//                                  * (((uint16_t) measure[0] << 8) | (uint16_t) measure[1]);
//    return bufferPos;
//}