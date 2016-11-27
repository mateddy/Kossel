#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

/**
 * configuration_store.cpp
 *
 * Configuration and EEPROM storage
 *
 * IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
 * in the functions below, also increment the version number. This makes sure that
 * the default values are used whenever there is a change to the data, to prevent
 * wrong data being written to the variables.
 *
 * ALSO: Variables in the Store and Retrieve sections must be in the same order.
 *       If a feature is disabled, some data must still be written that, when read,
 *       either sets a Sane Default, or results in No Change to the existing value.
 *
 */

#define EEPROM_VERSION "V11"
#define EEPROM_OFFSET 100

/**
 * V11 EEPROM Layout:
 *
 *  100  Version (char x4)
 *  104  EEPROM Checksum (uint16_t)
 *
 *  106  M92 XYZE  axis_steps_per_unit (float x4)
 *  122  M203 XYZE max_feedrate (float x4)
 *  138  M201 XYZE max_acceleration_units_per_sq_second (uint32_t x4)
 *  154  M204 S    acceleration (float)
 *  158  M204 T    retract_acceleration (float)
 *  162  M205 S    minimumfeedrate (float)
 *  166  M205 T    mintravelfeedrate (float)
 *  170  M205 B    minsegmenttime (ulong)
 *  174  M205 X    max_xy_jerk (float)
 *  178  M205 Z    max_z_jerk (float)
 *  182  M205 E    max_e_jerk (float)
 *  186  M206 XYZ  add_homeing (float x3)
 *
 * ULTIPANEL:
 *  198  M145 S0 H plaPreheatHotendTemp (int)
 *  200  M145 S0 B plaPreheatHPBTemp (int)
 *  202  M145 S0 F plaPreheatFanSpeed (int)
 *
 * PIDTEMP:
 *  204  M301 PID  Kp, Ki, Kd (float x3)
 *
 * DOGLCD:
 *  216  M250 C    lcd_contrast (int)
 *
 * AUTO BED LEVELING
 *  218  M851      zprobe_zoffset (float)
 *
 */

uint16_t eeprom_checksum;
const char version[4] = EEPROM_VERSION;

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size) {
  uint8_t c;
  while (size--) {
    eeprom_write_byte((unsigned char*)pos, *value);
    c = eeprom_read_byte((unsigned char*)pos);
    if (c != *value) {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("Error writing to EEPROM!");
    }
    eeprom_checksum += c;
    pos++;
    value++;
  }
}
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size) {
  do {
    uint8_t c = eeprom_read_byte((unsigned char*)pos);
    *value = c;
    eeprom_checksum += c;
    pos++;
    value++;
  } while(--size);
}

void Config_Postprocess() {
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  reset_acceleration_rates();

  // Call updatePID (similar to when we have processed M301)
  #ifdef PIDTEMP
  updatePID();
  #endif
}

#ifdef EEPROM_SETTINGS
#define DUMMY_PID_VALUE 3000.0f
#define EEPROM_START() int eeprom_index = EEPROM_OFFSET;
#define EEPROM_SKIP(VAR) eeprom_index += sizeof(VAR);
#define EEPROM_WRITE(VAR) _EEPROM_writeData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))
#define EEPROM_READ(VAR) _EEPROM_readData(eeprom_index, (uint8_t*)&VAR, sizeof(VAR))

/**
 * M500 - Store Configuration
 */
void Config_StoreSettings() {
  char ver[4]= "000";
  EEPROM_START()
  EEPROM_WRITE(ver); // invalidate data first
  EEPROM_SKIP(eeprom_checksum); // Skip the checksum slot
  eeprom_checksum = 0; // clear before first "real data"
  EEPROM_WRITE(axis_steps_per_unit);
  EEPROM_WRITE(max_feedrate);
  EEPROM_WRITE(max_acceleration_units_per_sq_second);
  EEPROM_WRITE(acceleration);
  EEPROM_WRITE(retract_acceleration);
  EEPROM_WRITE(minimumfeedrate);
  EEPROM_WRITE(mintravelfeedrate);
  EEPROM_WRITE(minsegmenttime);
  EEPROM_WRITE(max_xy_jerk);
  EEPROM_WRITE(max_z_jerk);
  EEPROM_WRITE(max_e_jerk);
  EEPROM_WRITE(add_homeing);
#ifndef ULTIPANEL
#ifdef PRINT_PLA
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
#endif
#ifdef PRINT_ABS
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#endif
#ifdef PRINT_PLA
  EEPROM_WRITE(plaPreheatHotendTemp);
  EEPROM_WRITE(plaPreheatHPBTemp);
  EEPROM_WRITE(plaPreheatFanSpeed);
#else
  int dummyi = 0;
  EEPROM_WRITE(dummyi);
  EEPROM_WRITE(dummyi);
  EEPROM_WRITE(dummyi);
#endif
  /*#ifdef PRINT_ABS
    EEPROM_WRITE(absPreheatHotendTemp);
    EEPROM_WRITE(absPreheatHPBTemp);
    EEPROM_WRITE(absPreheatFanSpeed);
    #else
    int dummyj = 0;
    EEPROM_WRITE(dummyj);
    EEPROM_WRITE(dummyj);
    EEPROM_WRITE(dummyj);
    #endif*/
#ifdef PIDTEMP
  EEPROM_WRITE(Kp);
  EEPROM_WRITE(Ki);
  EEPROM_WRITE(Kd);
#else
  float dummy = DUMMY_PID_VALUE;
  EEPROM_WRITE(dummy);
  dummy = 0.0f;
  EEPROM_WRITE(dummy);
  EEPROM_WRITE(dummy);
#endif
#ifndef DOGLCD
  int lcd_contrast = 32;
#endif
  EEPROM_WRITE(lcd_contrast);
#if !HAS_BED_PROBE
  float zprobe_zoffset = 0.0f;
#endif
  EEPROM_WRITE(zprobe_zoffset);
  uint16_t final_checksum = eeprom_checksum, eeprom_size = eeprom_index;
  eeprom_index=EEPROM_OFFSET;
  EEPROM_WRITE(version); // validate data
  EEPROM_WRITE(final_checksum);
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("Settings Stored (", (unsigned long)eeprom_size);
  SERIAL_ECHOLNPGM(" bytes)");
}

/**
 * M501 - Retrieve Configuration
 */
void Config_RetrieveSettings() {
  EEPROM_START();
  char stored_ver[4];
  EEPROM_READ(stored_ver); // read stored version
  uint16_t stored_checksum;
  EEPROM_READ(stored_checksum);
  //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
  if (strncmp(version,stored_ver,3) == 0) {
    eeprom_checksum = 0; // clear before reading first "real data"

    // version number match
    EEPROM_READ(axis_steps_per_unit);
    EEPROM_READ(max_feedrate);
    EEPROM_READ(max_acceleration_units_per_sq_second);
    EEPROM_READ(acceleration);
    EEPROM_READ(retract_acceleration);
    EEPROM_READ(minimumfeedrate);
    EEPROM_READ(mintravelfeedrate);
    EEPROM_READ(minsegmenttime);
    EEPROM_READ(max_xy_jerk);
    EEPROM_READ(max_z_jerk);
    EEPROM_READ(max_e_jerk);
    EEPROM_READ(add_homeing);
#ifndef ULTIPANEL
#ifdef PRINT_PLA
    int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
#endif
#ifdef PRINT_ABS
    int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
#endif
#endif
#ifdef PRINT_PLA
    EEPROM_READ(plaPreheatHotendTemp);
    EEPROM_READ(plaPreheatHPBTemp);
    EEPROM_READ(plaPreheatFanSpeed);
#endif
    /*#ifdef PRINT_ABS
      EEPROM_READ(absPreheatHotendTemp);
      EEPROM_READ(absPreheatHPBTemp);
      EEPROM_READ(absPreheatFanSpeed);
      #endif*/
#ifndef PIDTEMP
    float Kp,Ki,Kd;
#endif
    // do not need to scale PID values as the values in EEPROM are already scaled
    EEPROM_READ(Kp);
    EEPROM_READ(Ki);
    EEPROM_READ(Kd);
#ifndef DOGLCD
    int lcd_contrast;
#endif
    EEPROM_READ(lcd_contrast);
#if !HAS_BED_PROBE
    float zprobe_zoffset;
#endif
    EEPROM_READ(zprobe_zoffset);

    if (eeprom_checksum == stored_checksum) {
      Config_Postprocess();
      SERIAL_ECHO_START;
      SERIAL_ECHO(version);
      SERIAL_ECHOPAIR(" stored settings retrieved (", (unsigned long)eeprom_index);
      SERIAL_ECHOLNPGM(" bytes)");
    } else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("EEPROM checksum mismatch");
      Config_ResetDefault();
    }
  } else {
    Config_ResetDefault();
  }
#ifdef EEPROM_CHITCHAT
  Config_PrintSettings();
#endif
}
#endif // EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
 */
void Config_ResetDefault() {
  float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[]=DEFAULT_MAX_FEEDRATE;
  long tmp3[]=DEFAULT_MAX_ACCELERATION;
  for (short i=0;i<4;i++) {
    axis_steps_per_unit[i]=tmp1[i];
    max_feedrate[i]=tmp2[i];
    max_acceleration_units_per_sq_second[i]=tmp3[i];
  }

  acceleration=DEFAULT_ACCELERATION;
  retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
  minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
  minsegmenttime=DEFAULT_MINSEGMENTTIME;
  mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
  max_xy_jerk=DEFAULT_XYJERK;
  max_z_jerk=DEFAULT_ZJERK;
  max_e_jerk=DEFAULT_EJERK;
  add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
#ifdef ULTIPANEL
#ifdef PRINT_PLA
  plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
  plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
  plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
#endif
#ifdef PRINT_ABS
  absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
  absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
  absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#endif
#ifdef DOGLCD
  lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
  Kp = DEFAULT_Kp;
  Ki = scalePID_i(DEFAULT_Ki);
  Kd = scalePID_d(DEFAULT_Kd);
#ifdef PID_ADD_EXTRUSION_RATE
  Kc = DEFAULT_Kc;
#endif // PID_ADD_EXTRUSION_RATE
#endif // PIDTEMP

#if HAS_BED_PROBE
  zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
#endif

  Config_Postprocess();
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
}

/**
 * M503 - Print Configuration
 */
#ifdef EEPROM_CHITCHAT
void Config_PrintSettings() {
  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Steps per unit:");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
  SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
  SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
  SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
  SERIAL_ECHOLN("");

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
  SERIAL_ECHOPAIR(" Y",max_feedrate[1] );
  SERIAL_ECHOPAIR(" Z", max_feedrate[2] );
  SERIAL_ECHOPAIR(" E", max_feedrate[3]);
  SERIAL_ECHOLN("");

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] );
  SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] );
  SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
  SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
  SERIAL_ECHOLN("");
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M204 S",acceleration );
  SERIAL_ECHOPAIR(" T" ,retract_acceleration);
  SERIAL_ECHOLN("");

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M205 S",minimumfeedrate );
  SERIAL_ECHOPAIR(" T" ,mintravelfeedrate );
  SERIAL_ECHOPAIR(" B" ,minsegmenttime );
  SERIAL_ECHOPAIR(" X" ,max_xy_jerk );
  SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
  SERIAL_ECHOPAIR(" E" ,max_e_jerk);
  SERIAL_ECHOLN("");

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Home offset (mm):");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M206 X",add_homeing[0] );
  SERIAL_ECHOPAIR(" Y" ,add_homeing[1] );
  SERIAL_ECHOPAIR(" Z" ,add_homeing[2] );
  SERIAL_ECHOLN("");
#ifdef PIDTEMP
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("PID settings:");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M301 P",Kp);
  SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki));
  SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd));
  SERIAL_ECHOLN("");
#endif
  /**
   * Auto Bed Leveling
   */
#if HAS_BED_PROBE
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Z-Probe Offset (mm):");
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("  M851 Z", zprobe_zoffset);
  SERIAL_ECHOLN("");
#endif
}
#endif
