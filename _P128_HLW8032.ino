#ifdef USES_P128
//#######################################################################################################
//##################################### Plugin 128: HLW8032 - Energy ####################################
//#######################################################################################################
//######################################## DT27 @ Domoticz.cn ###########################################
//#######################################################################################################

#include "_Plugin_Helper.h"

#define PLUGIN_128
#define PLUGIN_ID_128         128
#define PLUGIN_NAME_128       "Energy (AC) - HLW8032"
#define PLUGIN_VALUENAME1_128 "Voltage"       //电压
#define PLUGIN_VALUENAME2_128 "Current"       //电流
#define PLUGIN_VALUENAME3_128 "Power"         //功率
// #define PLUGIN_VALUENAME4_128 "Total Pulses"  //脉冲总数量
#define PLUGIN_VALUENAME5_128 "Energy"        //电量

#define CSE_NOT_CALIBRATED          0xAA      //状态寄存器(State REG)，芯片误差修正功能失效，寄存器数据无效
#define VF                          1.88      //电压系数 (470K*4)/(1K*1000) = 1.88
#define CF                          1         //电流系数 1/(0.001*1000) = 1

struct P128_data_struct : public PluginTaskData_base {

  uint8_t serial_in_buffer[32];
  long voltage_reg = 0;
  long current_reg = 0;
  long power_reg = 0;
  long cf_pulses = 0;
  long total_cf_pulses = 0;
  float energy_voltage = 0;
  float energy_current = 0;
  float energy_power = 0;
  float energy = 0;
  long PF = 0;                 //脉冲计数器
	long PFData = 0;             //脉冲溢出计数器

  uint8_t checksum = 0;

  bool processCseReceived(struct EventStruct *event) {
    uint8_t header = serial_in_buffer[0];
    if ((header & 0xFC) == 0xFC) {
      //  Abnormal hardware
      return false;
    }

    //电压参数寄存器
    long voltage_coefficient = 0;
    if (CSE_NOT_CALIBRATED != header) {
      voltage_coefficient = serial_in_buffer[2] << 16 | serial_in_buffer[3] << 8 | serial_in_buffer[4];
    }

    //电流参数寄存器
    long current_coefficient = 0;
    if (CSE_NOT_CALIBRATED != header) {
      current_coefficient = serial_in_buffer[8] << 16 | serial_in_buffer[9] << 8 | serial_in_buffer[10];
    }

    //功率参数寄存器
    long power_coefficient = 0;
    if (CSE_NOT_CALIBRATED != header) {
      power_coefficient = serial_in_buffer[14] << 16 | serial_in_buffer[15] << 8 | serial_in_buffer[16];
    }

    energy_voltage = 0;
    energy_power = 0;
    energy_current = 0;
    energy = 0;

    //脉冲数量寄存器
    cf_pulses = serial_in_buffer[21] << 8 | serial_in_buffer[22];
    //确认PF进位寄存器是否进位，进位则添加1
		if(bitRead(serial_in_buffer[20], 7) == 1) {
			PFData++;
		}
    //脉冲数量统计
    total_cf_pulses = 65536 * PFData + cf_pulses;

    //如果电压寄存器刷新，则取数据
    if(bitRead(serial_in_buffer[20], 6) == 1){
      //电压寄存器
      voltage_reg = serial_in_buffer[5] << 16 | serial_in_buffer[6] << 8 | serial_in_buffer[7];
      //电压
      energy_voltage = (float)voltage_coefficient / (float)voltage_reg * 1.88;
    }
    //如果电流寄存器更新，则取数据
    if(bitRead(serial_in_buffer[20], 5) == 1)
		{
      //电流寄存器
      current_reg = serial_in_buffer[11] << 16 | serial_in_buffer[12] << 8 | serial_in_buffer[13];
      //电流
      energy_current = (float)current_coefficient / (float)current_reg * CF;
		}
    //如果功率寄存器数据更新，则取数据
    if(bitRead(serial_in_buffer[20], 4) == 1)
		{
			//功率寄存器
      power_reg = serial_in_buffer[17] << 16 | serial_in_buffer[18] << 8 | serial_in_buffer[19];
      //有功功率
      energy_power = ((float)power_coefficient / (float)power_reg) * 1.88 * CF;
		}
    if(power_coefficient>0){
      // energy = total_cf_pulses/((1/power_reg)*(1/(VF*CF))*1000000000*3600);
      energy = total_cf_pulses/((1/(float)power_coefficient)*(1/1.88)*3600000000000);
    }
    return true;
  }

  bool processSerialData() {
    bool found = false;
    while (Serial.available() > 0 && !found) {
      uint8_t serial_in_byte = Serial.read();
      checksum -= serial_in_buffer[2]; // substract from checksum data to be removed
      memmove(serial_in_buffer, serial_in_buffer + 1,
              sizeof(serial_in_buffer) - 1); // scroll buffer
      serial_in_buffer[25] = serial_in_byte; // add new data
      checksum += serial_in_buffer[22]; // add online checksum
      if (checksum == serial_in_buffer[23] && serial_in_buffer[1] == 0x5A) {
        found = true;
      }
    }
    return found;
  }

};

boolean Plugin_128(byte function, struct EventStruct *event, String &string) {
  boolean success = false;

  switch (function) {
    case PLUGIN_DEVICE_ADD: {
      Device[++deviceCount].Number = PLUGIN_ID_128;
      // Device[deviceCount].Type = DEVICE_TYPE_SERIAL;
      Device[deviceCount].VType = Sensor_VType::SENSOR_TYPE_QUAD;
      Device[deviceCount].Ports = 0;
      Device[deviceCount].PullUpOption = false;
      Device[deviceCount].InverseLogicOption = false;
      Device[deviceCount].FormulaOption = true;
      Device[deviceCount].ValueCount = 4;
      Device[deviceCount].SendDataOption = false;
      Device[deviceCount].TimerOption = false;
      Device[deviceCount].TimerOptional = false;
      Device[deviceCount].GlobalSyncOption = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME: {
      string = F(PLUGIN_NAME_128);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES: {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_128));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_128));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_128));
      // (ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME4_128));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME5_128));
      break;
    }

    case PLUGIN_GET_DEVICEGPIONAMES: {
      // No pins selectable, all hard coded
      break;
    }

    case PLUGIN_EXIT: {
      clearPluginTaskData(event->TaskIndex);
      success = true;
      break;
    }

    case PLUGIN_INIT: {
      initPluginTaskData(event->TaskIndex, new P128_data_struct());

      Settings.UseSerial = true; // Enable Serial port
      disableSerialLog(); // disable logging on serial port (used for HLW8032 communication)
      Settings.BaudRate = 4800; // HLW8032的UART接口以4800bps的固定频率工作，发送数据的间隔时间是50mS
      Serial.flush();
      Serial.begin(Settings.BaudRate, SERIAL_8E1);  //数据位8,偶校验位,停止位1
      success = true;
      break;
    }

    case PLUGIN_SERIAL_IN: {
      P128_data_struct* P128_data = static_cast<P128_data_struct*>(getPluginTaskData(event->TaskIndex));
      if (nullptr != P128_data) {
        success = true;
        if (P128_data->processSerialData()) {     //校验数据
          CseReceived(event);
          //更新数据显示
          UserVar[event->BaseVarIndex] = P128_data->energy_voltage;
          UserVar[event->BaseVarIndex + 1] = P128_data->energy_current;
          UserVar[event->BaseVarIndex + 2] = P128_data->energy_power;
          // UserVar[event->BaseVarIndex + 3] = P128_data->total_cf_pulses;
          UserVar[event->BaseVarIndex + 3] = P128_data->energy;
        }
      }
      break;
    }
  }
  return success;
}

bool CseReceived(struct EventStruct *event) {
  P128_data_struct* P128_data = static_cast<P128_data_struct*>(getPluginTaskData(event->TaskIndex));
  if (nullptr == P128_data) {
    return false;
  }
  if (!P128_data->processCseReceived(event)) {
    return false;
  }
  return true;
}

#endif // USES_P128
