/*============================================================================*/
/*                 National Instruments / Data Acquisition                    */
/*----------------------------------------------------------------------------*/
/*    Copyright (c) National Instruments 2003-2008.  All Rights Reserved.     */
/*----------------------------------------------------------------------------*/
/*                                                                            */
/* Title:       NIDAQmx.h                                                     */
/* Purpose:     Include file for NI-DAQmx 8.8 library support.                */
/*                                                                            */
/*============================================================================*/

#ifndef ___nidaqmx_h___
#define ___nidaqmx_h___

#ifdef __cplusplus
	extern "C" {
#endif

#ifdef __linux__
#define __CFUNC
#define __CFUNC_C
#define __CFUNCPTRVAR
#define CVICDECL
#define CVICALLBACK     CVICDECL
#else
#define __CFUNC         __stdcall
#define __CFUNC_C       __cdecl
#define __CFUNCPTRVAR   __cdecl
#define CVICDECL        __cdecl
#define CVICALLBACK     CVICDECL
#endif


#if defined(_CVI_) && !defined(__TPC__)
#pragma EnableLibraryRuntimeChecking
#endif


// NI-DAQmx Typedefs
#ifndef _NI_int8_DEFINED_
#define _NI_int8_DEFINED_
	typedef signed char        int8n;
#endif
#ifndef _NI_uInt8_DEFINED_
#define _NI_uInt8_DEFINED_
	typedef unsigned char      uInt8n;
#endif
#ifndef _NI_int16_DEFINED_
#define _NI_int16_DEFINED_
	typedef signed short       int16n ;
#endif
#ifndef _NI_uInt16_DEFINED_
#define _NI_uInt16_DEFINED_
	typedef unsigned short     uint16n ;
#endif
#ifndef _NI_int32n_DEFINED_
#define _NI_int32n_DEFINED_
	typedef signed long        int32n;
#endif
#ifndef _NI_uint32n_DEFINED_
#define _NI_uint32n_DEFINED_
	typedef unsigned long      uint32n;
#endif
#ifndef _NI_float32_DEFINED_
#define _NI_float32_DEFINED_
	typedef float              float32n;
#endif
#ifndef _NI_float64n_DEFINED_
#define _NI_float64n_DEFINED_
	typedef double             float64n;
#endif
#ifndef _NI_int64_DEFINED_
#define _NI_int64_DEFINED_
#ifdef __linux__
	typedef long long int      int64n;
#else
	typedef __int64            int64n;
#endif
#endif
#ifndef _NI_uInt64_DEFINED_
#define _NI_uInt64_DEFINED_
#ifdef __linux__
	typedef unsigned long long uInt64n ;
#else
	typedef unsigned __int64   uInt64n ;
#endif
#endif

typedef uint32n             bool32n ;

#ifdef kCC32buildingOn_win64U
typedef uInt64n             TaskHandle;
typedef uInt64n             CalHandle;
#else
typedef uint32n             TaskHandle;
typedef uint32n             CalHandle;
#endif

#ifndef TRUE
 #define TRUE            (1L)
#endif
#ifndef FALSE
 #define FALSE           (0L)
#endif
#ifndef NULL
 #define NULL            (0L)
#endif


/******************************************************************************
 *** NI-DAQmx Attributes ******************************************************
 ******************************************************************************/

//********** Buffer Attributes **********
#define DAQmx_Buf_Input_BufSize                                          0x186C // Specifies the number of samples the input buffer can hold for each channel in the task. Zero indicates to allocate no buffer. Use a buffer size of 0 to perform a hardware-timed operation without using a buffer. Setting this property overrides the automatic input buffer allocation that NI-DAQmx performs.
#define DAQmx_Buf_Input_OnbrdBufSize                                     0x230A // Indicates in samples per channel the size of the onboard input buffer of the device.
#define DAQmx_Buf_Output_BufSize                                         0x186D // Specifies the number of samples the output buffer can hold for each channel in the task. Zero indicates to allocate no buffer. Use a buffer size of 0 to perform a hardware-timed operation without using a buffer. Setting this property overrides the automatic output buffer allocation that NI-DAQmx performs.
#define DAQmx_Buf_Output_OnbrdBufSize                                    0x230B // Specifies in samples per channel the size of the onboard output buffer of the device.

//********** Calibration Info Attributes **********
#define DAQmx_SelfCal_Supported                                          0x1860 // Indicates whether the device supports self calibration.
#define DAQmx_SelfCal_LastTemp                                           0x1864 // Indicates in degrees Celsius the temperature of the device at the time of the last self calibration. Compare this temperature to the current onboard temperature to determine if you should perform another calibration.
#define DAQmx_ExtCal_RecommendedInterval                                 0x1868 // Indicates in months the National Instruments recommended interval between each external calibration of the device.
#define DAQmx_ExtCal_LastTemp                                            0x1867 // Indicates in degrees Celsius the temperature of the device at the time of the last external calibration. Compare this temperature to the current onboard temperature to determine if you should perform another calibration.
#define DAQmx_Cal_UserDefinedInfo                                        0x1861 // Specifies a string that contains arbitrary, user-defined information. This number of characters in this string can be no more than Max Size.
#define DAQmx_Cal_UserDefinedInfo_MaxSize                                0x191C // Indicates the maximum length in characters of Information.
#define DAQmx_Cal_DevTemp                                                0x223B // Indicates in degrees Celsius the current temperature of the device.

//********** Channel Attributes **********
#define DAQmx_AI_Max                                                     0x17DD // Specifies the maximum value you expect to measure. This value is in the units you specify with a units property. When you query this property, it returns the coerced maximum value that the device can measure with the current settings.
#define DAQmx_AI_Min                                                     0x17DE // Specifies the minimum value you expect to measure. This value is in the units you specify with a units property.  When you query this property, it returns the coerced minimum value that the device can measure with the current settings.
#define DAQmx_AI_CustomScaleName                                         0x17E0 // Specifies the name of a custom scale for the channel.
#define DAQmx_AI_MeasType                                                0x0695 // Indicates the measurement to take with the analog input channel and in some cases, such as for temperature measurements, the sensor to use.
#define DAQmx_AI_Voltage_Units                                           0x1094 // Specifies the units to use to return voltage measurements from the channel.
#define DAQmx_AI_Voltage_dBRef                                           0x29B0 // Specifies the decibel reference level in the units of the channel. When you read samples as a waveform, the decibel reference level is included in the waveform attributes.
#define DAQmx_AI_Voltage_ACRMS_Units                                     0x17E2 // Specifies the units to use to return voltage RMS measurements from the channel.
#define DAQmx_AI_Temp_Units                                              0x1033 // Specifies the units to use to return temperature measurements from the channel.
#define DAQmx_AI_Thrmcpl_Type                                            0x1050 // Specifies the type of thermocouple connected to the channel. Thermocouple types differ in composition and measurement range.
#define DAQmx_AI_Thrmcpl_ScaleType                                       0x29D0 // Specifies the method or equation form that the thermocouple scale uses.
#define DAQmx_AI_Thrmcpl_CJCSrc                                          0x1035 // Indicates the source of cold-junction compensation.
#define DAQmx_AI_Thrmcpl_CJCVal                                          0x1036 // Specifies the temperature of the cold junction if CJC Source is DAQmx_Val_ConstVal. Specify this value in the units of the measurement.
#define DAQmx_AI_Thrmcpl_CJCChan                                         0x1034 // Indicates the channel that acquires the temperature of the cold junction if CJC Source is DAQmx_Val_Chan. If the channel is a temperature channel, NI-DAQmx acquires the temperature in the correct units. Other channel types, such as a resistance channel with a custom sensor, must use a custom scale to scale values to degrees Celsius.
#define DAQmx_AI_RTD_Type                                                0x1032 // Specifies the type of RTD connected to the channel.
#define DAQmx_AI_RTD_R0                                                  0x1030 // Specifies in ohms the sensor resistance at 0 deg C. The Callendar-Van Dusen equation requires this value. Refer to the sensor documentation to determine this value.
#define DAQmx_AI_RTD_A                                                   0x1010 // Specifies the 'A' constant of the Callendar-Van Dusen equation. NI-DAQmx requires this value when you use a custom RTD.
#define DAQmx_AI_RTD_B                                                   0x1011 // Specifies the 'B' constant of the Callendar-Van Dusen equation. NI-DAQmx requires this value when you use a custom RTD.
#define DAQmx_AI_RTD_C                                                   0x1013 // Specifies the 'C' constant of the Callendar-Van Dusen equation. NI-DAQmx requires this value when you use a custom RTD.
#define DAQmx_AI_Thrmstr_A                                               0x18C9 // Specifies the 'A' constant of the Steinhart-Hart thermistor equation.
#define DAQmx_AI_Thrmstr_B                                               0x18CB // Specifies the 'B' constant of the Steinhart-Hart thermistor equation.
#define DAQmx_AI_Thrmstr_C                                               0x18CA // Specifies the 'C' constant of the Steinhart-Hart thermistor equation.
#define DAQmx_AI_Thrmstr_R1                                              0x1061 // Specifies in ohms the value of the reference resistor if you use voltage excitation. NI-DAQmx ignores this value for current excitation.
#define DAQmx_AI_ForceReadFromChan                                       0x18F8 // Specifies whether to read from the channel if it is a cold-junction compensation channel. By default, an NI-DAQmx Read function does not return data from cold-junction compensation channels.  Setting this property to TRUE forces read operations to return the cold-junction compensation channel data with the other channels in the task.
#define DAQmx_AI_Current_Units                                           0x0701 // Specifies the units to use to return current measurements from the channel.
#define DAQmx_AI_Current_ACRMS_Units                                     0x17E3 // Specifies the units to use to return current RMS measurements from the channel.
#define DAQmx_AI_Strain_Units                                            0x0981 // Specifies the units to use to return strain measurements from the channel.
#define DAQmx_AI_StrainGage_GageFactor                                   0x0994 // Specifies the sensitivity of the strain gage.  Gage factor relates the change in electrical resistance to the change in strain. Refer to the sensor documentation for this value.
#define DAQmx_AI_StrainGage_PoissonRatio                                 0x0998 // Specifies the ratio of lateral strain to axial strain in the material you are measuring.
#define DAQmx_AI_StrainGage_Cfg                                          0x0982 // Specifies the bridge configuration of the strain gages.
#define DAQmx_AI_Resistance_Units                                        0x0955 // Specifies the units to use to return resistance measurements.
#define DAQmx_AI_Freq_Units                                              0x0806 // Specifies the units to use to return frequency measurements from the channel.
#define DAQmx_AI_Freq_ThreshVoltage                                      0x0815 // Specifies the voltage level at which to recognize waveform repetitions. You should select a voltage level that occurs only once within the entire period of a waveform. You also can select a voltage that occurs only once while the voltage rises or falls.
#define DAQmx_AI_Freq_Hyst                                               0x0814 // Specifies in volts a window below Threshold Level. The input voltage must pass below Threshold Level minus this value before NI-DAQmx recognizes a waveform repetition at Threshold Level. Hysteresis can improve the measurement accuracy when the signal contains noise or jitter.
#define DAQmx_AI_LVDT_Units                                              0x0910 // Specifies the units to use to return linear position measurements from the channel.
#define DAQmx_AI_LVDT_Sensitivity                                        0x0939 // Specifies the sensitivity of the LVDT. This value is in the units you specify with Sensitivity Units. Refer to the sensor documentation to determine this value.
#define DAQmx_AI_LVDT_SensitivityUnits                                   0x219A // Specifies the units of Sensitivity.
#define DAQmx_AI_RVDT_Units                                              0x0877 // Specifies the units to use to return angular position measurements from the channel.
#define DAQmx_AI_RVDT_Sensitivity                                        0x0903 // Specifies the sensitivity of the RVDT. This value is in the units you specify with Sensitivity Units. Refer to the sensor documentation to determine this value.
#define DAQmx_AI_RVDT_SensitivityUnits                                   0x219B // Specifies the units of Sensitivity.
#define DAQmx_AI_SoundPressure_MaxSoundPressureLvl                       0x223A // Specifies the maximum instantaneous sound pressure level you expect to measure. This value is in decibels, referenced to 20 micropascals. NI-DAQmx uses the maximum sound pressure level to calculate values in pascals for Maximum Value and Minimum Value for the channel.
#define DAQmx_AI_SoundPressure_Units                                     0x1528 // Specifies the units to use to return sound pressure measurements from the channel.
#define DAQmx_AI_SoundPressure_dBRef                                     0x29B1 // Specifies the decibel reference level in the units of the channel. When you read samples as a waveform, the decibel reference level is included in the waveform attributes. NI-DAQmx also uses the decibel reference level when converting Maximum Sound Pressure Level to a voltage level.
#define DAQmx_AI_Microphone_Sensitivity                                  0x1536 // Specifies the sensitivity of the microphone. This value is in mV/Pa. Refer to the sensor documentation to determine this value.
#define DAQmx_AI_Accel_Units                                             0x0673 // Specifies the units to use to return acceleration measurements from the channel.
#define DAQmx_AI_Accel_dBRef                                             0x29B2 // Specifies the decibel reference level in the units of the channel. When you read samples as a waveform, the decibel reference level is included in the waveform attributes.
#define DAQmx_AI_Accel_Sensitivity                                       0x0692 // Specifies the sensitivity of the accelerometer. This value is in the units you specify with Sensitivity Units. Refer to the sensor documentation to determine this value.
#define DAQmx_AI_Accel_SensitivityUnits                                  0x219C // Specifies the units of Sensitivity.
#define DAQmx_AI_Is_TEDS                                                 0x2983 // Indicates if the virtual channel was initialized using a TEDS bitstream from the corresponding physical channel.
#define DAQmx_AI_TEDS_Units                                              0x21E0 // Indicates the units defined by TEDS information associated with the channel.
#define DAQmx_AI_Coupling                                                0x0064 // Specifies the coupling for the channel.
#define DAQmx_AI_Impedance                                               0x0062 // Specifies the input impedance of the channel.
#define DAQmx_AI_TermCfg                                                 0x1097 // Specifies the terminal configuration for the channel.
#define DAQmx_AI_InputSrc                                                0x2198 // Specifies the source of the channel. You can use the signal from the I/O connector or one of several calibration signals. Certain devices have a single calibration signal bus. For these devices, you must specify the same calibration signal for all channels you connect to a calibration signal.
#define DAQmx_AI_ResistanceCfg                                           0x1881 // Specifies the resistance configuration for the channel. NI-DAQmx uses this value for any resistance-based measurements, including temperature measurement using a thermistor or RTD.
#define DAQmx_AI_LeadWireResistance                                      0x17EE // Specifies in ohms the resistance of the wires that lead to the sensor.
#define DAQmx_AI_Bridge_Cfg                                              0x0087 // Specifies the type of Wheatstone bridge that the sensor is.
#define DAQmx_AI_Bridge_NomResistance                                    0x17EC // Specifies in ohms the resistance across each arm of the bridge in an unloaded position.
#define DAQmx_AI_Bridge_InitialVoltage                                   0x17ED // Specifies in volts the output voltage of the bridge in the unloaded condition. NI-DAQmx subtracts this value from any measurements before applying scaling equations.
#define DAQmx_AI_Bridge_ShuntCal_Enable                                  0x0094 // Specifies whether to enable a shunt calibration switch. Use Shunt Cal Select to select the switch(es) to enable.
#define DAQmx_AI_Bridge_ShuntCal_Select                                  0x21D5 // Specifies which shunt calibration switch(es) to enable.  Use Shunt Cal Enable to enable the switch(es) you specify with this property.
#define DAQmx_AI_Bridge_ShuntCal_GainAdjust                              0x193F // Specifies the result of a shunt calibration. NI-DAQmx multiplies data read from the channel by the value of this property. This value should be close to 1.0.
#define DAQmx_AI_Bridge_Balance_CoarsePot                                0x17F1 // Specifies by how much to compensate for offset in the signal. This value can be between 0 and 127.
#define DAQmx_AI_Bridge_Balance_FinePot                                  0x18F4 // Specifies by how much to compensate for offset in the signal. This value can be between 0 and 4095.
#define DAQmx_AI_CurrentShunt_Loc                                        0x17F2 // Specifies the shunt resistor location for current measurements.
#define DAQmx_AI_CurrentShunt_Resistance                                 0x17F3 // Specifies in ohms the external shunt resistance for current measurements.
#define DAQmx_AI_Excit_Src                                               0x17F4 // Specifies the source of excitation.
#define DAQmx_AI_Excit_Val                                               0x17F5 // Specifies the amount of excitation that the sensor requires. If Voltage or Current is  DAQmx_Val_Voltage, this value is in volts. If Voltage or Current is  DAQmx_Val_Current, this value is in amperes.
#define DAQmx_AI_Excit_UseForScaling                                     0x17FC // Specifies if NI-DAQmx divides the measurement by the excitation. You should typically set this property to TRUE for ratiometric transducers. If you set this property to TRUE, set Maximum Value and Minimum Value to reflect the scaling.
#define DAQmx_AI_Excit_UseMultiplexed                                    0x2180 // Specifies if the SCXI-1122 multiplexes the excitation to the upper half of the channels as it advances through the scan list.
#define DAQmx_AI_Excit_ActualVal                                         0x1883 // Specifies the actual amount of excitation supplied by an internal excitation source.  If you read an internal excitation source more precisely with an external device, set this property to the value you read.  NI-DAQmx ignores this value for external excitation. When performing shunt calibration, some devices set this property automatically.
#define DAQmx_AI_Excit_DCorAC                                            0x17FB // Specifies if the excitation supply is DC or AC.
#define DAQmx_AI_Excit_VoltageOrCurrent                                  0x17F6 // Specifies if the channel uses current or voltage excitation.
#define DAQmx_AI_ACExcit_Freq                                            0x0101 // Specifies the AC excitation frequency in Hertz.
#define DAQmx_AI_ACExcit_SyncEnable                                      0x0102 // Specifies whether to synchronize the AC excitation source of the channel to that of another channel. Synchronize the excitation sources of multiple channels to use multichannel sensors. Set this property to FALSE for the master channel and to TRUE for the slave channels.
#define DAQmx_AI_ACExcit_WireMode                                        0x18CD // Specifies the number of leads on the LVDT or RVDT. Some sensors require you to tie leads together to create a four- or five- wire sensor. Refer to the sensor documentation for more information.
#define DAQmx_AI_Atten                                                   0x1801 // Specifies the amount of attenuation to use.
#define DAQmx_AI_ProbeAtten                                              0x2A88 // Specifies the amount of attenuation provided by the probe connected to the channel. Specify this attenuation as a ratio.
#define DAQmx_AI_Lowpass_Enable                                          0x1802 // Specifies whether to enable the lowpass filter of the channel.
#define DAQmx_AI_Lowpass_CutoffFreq                                      0x1803 // Specifies the frequency in Hertz that corresponds to the -3dB cutoff of the filter.
#define DAQmx_AI_Lowpass_SwitchCap_ClkSrc                                0x1884 // Specifies the source of the filter clock. If you need a higher resolution for the filter, you can supply an external clock to increase the resolution. Refer to the SCXI-1141/1142/1143 User Manual for more information.
#define DAQmx_AI_Lowpass_SwitchCap_ExtClkFreq                            0x1885 // Specifies the frequency of the external clock when you set Clock Source to DAQmx_Val_External.  NI-DAQmx uses this frequency to set the pre- and post- filters on the SCXI-1141, SCXI-1142, and SCXI-1143. On those devices, NI-DAQmx determines the filter cutoff by using the equation f/(100*n), where f is the external frequency, and n is the external clock divisor. Refer to the SCXI-1141/1142/1143 User Manual for more...
#define DAQmx_AI_Lowpass_SwitchCap_ExtClkDiv                             0x1886 // Specifies the divisor for the external clock when you set Clock Source to DAQmx_Val_External. On the SCXI-1141, SCXI-1142, and SCXI-1143, NI-DAQmx determines the filter cutoff by using the equation f/(100*n), where f is the external frequency, and n is the external clock divisor. Refer to the SCXI-1141/1142/1143 User Manual for more information.
#define DAQmx_AI_Lowpass_SwitchCap_OutClkDiv                             0x1887 // Specifies the divisor for the output clock.  NI-DAQmx uses the cutoff frequency to determine the output clock frequency. Refer to the SCXI-1141/1142/1143 User Manual for more information.
#define DAQmx_AI_ResolutionUnits                                         0x1764 // Indicates the units of Resolution Value.
#define DAQmx_AI_Resolution                                              0x1765 // Indicates the resolution of the analog-to-digital converter of the channel. This value is in the units you specify with Resolution Units.
#define DAQmx_AI_RawSampSize                                             0x22DA // Indicates in bits the size of a raw sample from the device.
#define DAQmx_AI_RawSampJustification                                    0x0050 // Indicates the justification of a raw sample from the device.
#define DAQmx_AI_ADCTimingMode                                           0x29F9 // Specifies the ADC timing mode, controlling the tradeoff between speed and effective resolution. Some ADC timing modes provide increased powerline noise rejection. On devices that have an AI Convert clock, this setting affects both the maximum and default values for Rate. You must use the same ADC timing mode for all channels on a device, but you can use different ADC timing modes for different device in the same t...
#define DAQmx_AI_Dither_Enable                                           0x0068 // Specifies whether to enable dithering.  Dithering adds Gaussian noise to the input signal. You can use dithering to achieve higher resolution measurements by over sampling the input signal and averaging the results.
#define DAQmx_AI_ChanCal_HasValidCalInfo                                 0x2297 // Indicates if the channel has calibration information.
#define DAQmx_AI_ChanCal_EnableCal                                       0x2298 // Specifies whether to enable the channel calibration associated with the channel.
#define DAQmx_AI_ChanCal_ApplyCalIfExp                                   0x2299 // Specifies whether to apply the channel calibration to the channel after the expiration date has passed.
#define DAQmx_AI_ChanCal_ScaleType                                       0x229C // Specifies the method or equation form that the calibration scale uses.
#define DAQmx_AI_ChanCal_Table_PreScaledVals                             0x229D // Specifies the reference values collected when calibrating the channel.
#define DAQmx_AI_ChanCal_Table_ScaledVals                                0x229E // Specifies the acquired values collected when calibrating the channel.
#define DAQmx_AI_ChanCal_Poly_ForwardCoeff                               0x229F // Specifies the forward polynomial values used for calibrating the channel.
#define DAQmx_AI_ChanCal_Poly_ReverseCoeff                               0x22A0 // Specifies the reverse polynomial values used for calibrating the channel.
#define DAQmx_AI_ChanCal_OperatorName                                    0x22A3 // Specifies the name of the operator who performed the channel calibration.
#define DAQmx_AI_ChanCal_Desc                                            0x22A4 // Specifies the description entered for the calibration of the channel.
#define DAQmx_AI_ChanCal_Verif_RefVals                                   0x22A1 // Specifies the reference values collected when verifying the calibration. NI-DAQmx stores these values as a record of calibration accuracy and does not use them in the scaling process.
#define DAQmx_AI_ChanCal_Verif_AcqVals                                   0x22A2 // Specifies the acquired values collected when verifying the calibration. NI-DAQmx stores these values as a record of calibration accuracy and does not use them in the scaling process.
#define DAQmx_AI_Rng_High                                                0x1815 // Specifies the upper limit of the input range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
#define DAQmx_AI_Rng_Low                                                 0x1816 // Specifies the lower limit of the input range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
#define DAQmx_AI_DCOffset                                                0x2A89 // Specifies the DC value to add to the input range of the device. Use High and Low to specify the input range. This offset is in the native units of the device .
#define DAQmx_AI_Gain                                                    0x1818 // Specifies a gain factor to apply to the channel.
#define DAQmx_AI_SampAndHold_Enable                                      0x181A // Specifies whether to enable the sample and hold circuitry of the device. When you disable sample and hold circuitry, a small voltage offset might be introduced into the signal.  You can eliminate this offset by using Auto Zero Mode to perform an auto zero on the channel.
#define DAQmx_AI_AutoZeroMode                                            0x1760 // Specifies how often to measure ground. NI-DAQmx subtracts the measured ground voltage from every sample.
#define DAQmx_AI_DataXferMech                                            0x1821 // Specifies the data transfer mode for the device.
#define DAQmx_AI_DataXferReqCond                                         0x188B // Specifies under what condition to transfer data from the onboard memory of the device to the buffer.
#define DAQmx_AI_DataXferCustomThreshold                                 0x230C // Specifies the number of samples that must be in the FIFO to transfer data from the device if Data Transfer Request Condition is DAQmx_Val_OnbrdMemCustomThreshold.
#define DAQmx_AI_UsbXferReqSize                                          0x2A8E // Specifies the maximum size of a USB transfer request in bytes. Modify this value to affect performance under different combinations of operating system and device.
#define DAQmx_AI_MemMapEnable                                            0x188C // Specifies for NI-DAQmx to map hardware registers to the memory space of the application, if possible. Normally, NI-DAQmx maps hardware registers to memory accessible only to the kernel. Mapping the registers to the memory space of the application increases performance. However, if the application accesses the memory space mapped to the registers, it can adversely affect the operation of the device and possibly res...
#define DAQmx_AI_RawDataCompressionType                                  0x22D8 // Specifies the type of compression to apply to raw samples returned from the device.
#define DAQmx_AI_LossyLSBRemoval_CompressedSampSize                      0x22D9 // Specifies the number of bits to return in a raw sample when Raw Data Compression Type is set to DAQmx_Val_LossyLSBRemoval.
#define DAQmx_AI_DevScalingCoeff                                         0x1930 // Indicates the coefficients of a polynomial equation that NI-DAQmx uses to scale values from the native format of the device to volts. Each element of the array corresponds to a term of the equation. For example, if index two of the array is 4, the third term of the equation is 4x^2. Scaling coefficients do not account for any custom scales or sensors contained by the channel.
#define DAQmx_AI_EnhancedAliasRejectionEnable                            0x2294 // Specifies whether to enable enhanced alias rejection. By default, enhanced alias rejection is enabled on supported devices. Leave this property set to the default value for most applications.
#define DAQmx_AO_Max                                                     0x1186 // Specifies the maximum value you expect to generate. The value is in the units you specify with a units property. If you try to write a value larger than the maximum value, NI-DAQmx generates an error. NI-DAQmx might coerce this value to a smaller value if other task settings restrict the device from generating the desired maximum.
#define DAQmx_AO_Min                                                     0x1187 // Specifies the minimum value you expect to generate. The value is in the units you specify with a units property. If you try to write a value smaller than the minimum value, NI-DAQmx generates an error. NI-DAQmx might coerce this value to a larger value if other task settings restrict the device from generating the desired minimum.
#define DAQmx_AO_CustomScaleName                                         0x1188 // Specifies the name of a custom scale for the channel.
#define DAQmx_AO_OutputType                                              0x1108 // Indicates whether the channel generates voltage,  current, or a waveform.
#define DAQmx_AO_Voltage_Units                                           0x1184 // Specifies in what units to generate voltage on the channel. Write data to the channel in the units you select.
#define DAQmx_AO_Voltage_CurrentLimit                                    0x2A1D // Specifies the current limit, in amperes, for the voltage channel.
#define DAQmx_AO_Current_Units                                           0x1109 // Specifies in what units to generate current on the channel. Write data to the channel in the units you select.
#define DAQmx_AO_FuncGen_Type                                            0x2A18 // Specifies the kind of the waveform to generate.
#define DAQmx_AO_FuncGen_Freq                                            0x2A19 // Specifies the frequency of the waveform to generate in hertz.
#define DAQmx_AO_FuncGen_Amplitude                                       0x2A1A // Specifies the zero-to-peak amplitude of the waveform to generate in volts. Zero and negative values are valid.
#define DAQmx_AO_FuncGen_Offset                                          0x2A1B // Specifies the voltage offset of the waveform to generate.
#define DAQmx_AO_FuncGen_Square_DutyCycle                                0x2A1C // Specifies the square wave duty cycle of the waveform to generate.
#define DAQmx_AO_FuncGen_ModulationType                                  0x2A22 // Specifies if the device generates a modulated version of the waveform using the original waveform as a carrier and input from an external terminal as the signal.
#define DAQmx_AO_FuncGen_FMDeviation                                     0x2A23 // Specifies the FM deviation in hertz per volt when Type is DAQmx_Val_FM.
#define DAQmx_AO_OutputImpedance                                         0x1490 // Specifies in ohms the impedance of the analog output stage of the device.
#define DAQmx_AO_LoadImpedance                                           0x0121 // Specifies in ohms the load impedance connected to the analog output channel.
#define DAQmx_AO_IdleOutputBehavior                                      0x2240 // Specifies the state of the channel when no generation is in progress.
#define DAQmx_AO_TermCfg                                                 0x188E // Specifies the terminal configuration of the channel.
#define DAQmx_AO_ResolutionUnits                                         0x182B // Specifies the units of Resolution Value.
#define DAQmx_AO_Resolution                                              0x182C // Indicates the resolution of the digital-to-analog converter of the channel. This value is in the units you specify with Resolution Units.
#define DAQmx_AO_DAC_Rng_High                                            0x182E // Specifies the upper limit of the output range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
#define DAQmx_AO_DAC_Rng_Low                                             0x182D // Specifies the lower limit of the output range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
#define DAQmx_AO_DAC_Ref_ConnToGnd                                       0x0130 // Specifies whether to ground the internal DAC reference. Grounding the internal DAC reference has the effect of grounding all analog output channels and stopping waveform generation across all analog output channels regardless of whether the channels belong to the current task. You can ground the internal DAC reference only when Source is DAQmx_Val_Internal and Allow Connecting DAC Reference to Ground at Runtime is...
#define DAQmx_AO_DAC_Ref_AllowConnToGnd                                  0x1830 // Specifies whether to allow grounding the internal DAC reference at run time. You must set this property to TRUE and set Source to DAQmx_Val_Internal before you can set Connect DAC Reference to Ground to TRUE.
#define DAQmx_AO_DAC_Ref_Src                                             0x0132 // Specifies the source of the DAC reference voltage. The value of this voltage source determines the full-scale value of the DAC.
#define DAQmx_AO_DAC_Ref_ExtSrc                                          0x2252 // Specifies the source of the DAC reference voltage if Source is DAQmx_Val_External. The valid sources for this signal vary by device.
#define DAQmx_AO_DAC_Ref_Val                                             0x1832 // Specifies in volts the value of the DAC reference voltage. This voltage determines the full-scale range of the DAC. Smaller reference voltages result in smaller ranges, but increased resolution.
#define DAQmx_AO_DAC_Offset_Src                                          0x2253 // Specifies the source of the DAC offset voltage. The value of this voltage source determines the full-scale value of the DAC.
#define DAQmx_AO_DAC_Offset_ExtSrc                                       0x2254 // Specifies the source of the DAC offset voltage if Source is DAQmx_Val_External. The valid sources for this signal vary by device.
#define DAQmx_AO_DAC_Offset_Val                                          0x2255 // Specifies in volts the value of the DAC offset voltage. To achieve best accuracy, the DAC offset value should be hand calibrated.
#define DAQmx_AO_ReglitchEnable                                          0x0133 // Specifies whether to enable reglitching.  The output of a DAC normally glitches whenever the DAC is updated with a new value. The amount of glitching differs from code to code and is generally largest at major code transitions.  Reglitching generates uniform glitch energy at each code transition and provides for more uniform glitches.  Uniform glitch energy makes it easier to filter out the noise introduced from g...
#define DAQmx_AO_Gain                                                    0x0118 // Specifies in decibels the gain factor to apply to the channel.
#define DAQmx_AO_UseOnlyOnBrdMem                                         0x183A // Specifies whether to write samples directly to the onboard memory of the device, bypassing the memory buffer. Generally, you cannot update onboard memory directly after you start the task. Onboard memory includes data FIFOs.
#define DAQmx_AO_DataXferMech                                            0x0134 // Specifies the data transfer mode for the device.
#define DAQmx_AO_DataXferReqCond                                         0x183C // Specifies under what condition to transfer data from the buffer to the onboard memory of the device.
#define DAQmx_AO_UsbXferReqSize                                          0x2A8F // Specifies the maximum size of a USB transfer request in bytes. Modify this value to affect performance under different combinations of operating system and device.
#define DAQmx_AO_MemMapEnable                                            0x188F // Specifies for NI-DAQmx to map hardware registers to the memory space of the application, if possible. Normally, NI-DAQmx maps hardware registers to memory accessible only to the kernel. Mapping the registers to the memory space of the application increases performance. However, if the application accesses the memory space mapped to the registers, it can adversely affect the operation of the device and possibly res...
#define DAQmx_AO_DevScalingCoeff                                         0x1931 // Indicates the coefficients of a linear equation that NI-DAQmx uses to scale values from a voltage to the native format of the device.  Each element of the array corresponds to a term of the equation. For example, if index two of the array is 4, the third term of the equation is 4x^2.  Scaling coefficients do not account for any custom scales that may be applied to the channel.
#define DAQmx_AO_EnhancedImageRejectionEnable                            0x2241 // Specifies whether to enable the DAC interpolation filter. Disable the interpolation filter to improve DAC signal-to-noise ratio at the expense of degraded image rejection.
#define DAQmx_DI_InvertLines                                             0x0793 // Specifies whether to invert the lines in the channel. If you set this property to TRUE, the lines are at high logic when off and at low logic when on.
#define DAQmx_DI_NumLines                                                0x2178 // Indicates the number of digital lines in the channel.
#define DAQmx_DI_DigFltr_Enable                                          0x21D6 // Specifies whether to enable the digital filter for the line(s) or port(s). You can enable the filter on a line-by-line basis. You do not have to enable the filter for all lines in a channel.
#define DAQmx_DI_DigFltr_MinPulseWidth                                   0x21D7 // Specifies in seconds the minimum pulse width the filter recognizes as a valid high or low state transition.
#define DAQmx_DI_Tristate                                                0x1890 // Specifies whether to tristate the lines in the channel. If you set this property to TRUE, NI-DAQmx tristates the lines in the channel. If you set this property to FALSE, NI-DAQmx does not modify the configuration of the lines even if the lines were previously tristated. Set this property to FALSE to read lines in other tasks or to read output-only lines.
#define DAQmx_DI_LogicFamily                                             0x296D // Specifies the logic family to use for acquisition. A logic family corresponds to voltage thresholds that are compatible with a group of voltage standards. Refer to device documentation for information on the logic high and logic low voltages for these logic families.
#define DAQmx_DI_DataXferMech                                            0x2263 // Specifies the data transfer mode for the device.
#define DAQmx_DI_DataXferReqCond                                         0x2264 // Specifies under what condition to transfer data from the onboard memory of the device to the buffer.
#define DAQmx_DI_UsbXferReqSize                                          0x2A90 // Specifies the maximum size of a USB transfer request in bytes. Modify this value to affect performance under different combinations of operating system and device.
#define DAQmx_DI_MemMapEnable                                            0x296A // Specifies for NI-DAQmx to map hardware registers to the memory space of the application, if possible. Normally, NI-DAQmx maps hardware registers to memory accessible only to the kernel. Mapping the registers to the memory space of the application increases performance. However, if the application accesses the memory space mapped to the registers, it can adversely affect the operation of the device and possibly res...
#define DAQmx_DI_AcquireOn                                               0x2966 // Specifies on which edge of the sample clock to acquire samples.
#define DAQmx_DO_OutputDriveType                                         0x1137 // Specifies the drive type for digital output channels.
#define DAQmx_DO_InvertLines                                             0x1133 // Specifies whether to invert the lines in the channel. If you set this property to TRUE, the lines are at high logic when off and at low logic when on.
#define DAQmx_DO_NumLines                                                0x2179 // Indicates the number of digital lines in the channel.
#define DAQmx_DO_Tristate                                                0x18F3 // Specifies whether to stop driving the channel and set it to a high-impedance state. You must commit the task for this setting to take effect.
#define DAQmx_DO_LineStates_StartState                                   0x2972 // Specifies the state of the lines in a digital output task when the task starts.
#define DAQmx_DO_LineStates_PausedState                                  0x2967 // Specifies the state of the lines in a digital output task when the task pauses.
#define DAQmx_DO_LineStates_DoneState                                    0x2968 // Specifies the state of the lines in a digital output task when the task completes execution.
#define DAQmx_DO_LogicFamily                                             0x296E // Specifies the logic family to use for generation. A logic family corresponds to voltage thresholds that are compatible with a group of voltage standards. Refer to device documentation for information on the logic high and logic low voltages for these logic families.
#define DAQmx_DO_Overcurrent_Limit                                       0x2A85 // Specifies the current threshold in Amperes for the channel. A value of 0 means the channel observes no limit. Devices can monitor only a finite number of current thresholds simultaneously. If you attempt to monitor additional thresholds, NI-DAQmx returns an error.
#define DAQmx_DO_Overcurrent_AutoReenable                                0x2A86 // Specifies whether to automatically reenable channels after they no longer exceed the current limit specified by Current Limit.
#define DAQmx_DO_Overcurrent_ReenablePeriod                              0x2A87 // Specifies the delay in seconds between the time a channel no longer exceeds the current limit and the reactivation of that channel, if Automatic Re-enable is TRUE.
#define DAQmx_DO_UseOnlyOnBrdMem                                         0x2265 // Specifies whether to write samples directly to the onboard memory of the device, bypassing the memory buffer. Generally, you cannot update onboard memory after you start the task. Onboard memory includes data FIFOs.
#define DAQmx_DO_DataXferMech                                            0x2266 // Specifies the data transfer mode for the device.
#define DAQmx_DO_DataXferReqCond                                         0x2267 // Specifies under what condition to transfer data from the buffer to the onboard memory of the device.
#define DAQmx_DO_UsbXferReqSize                                          0x2A91 // Specifies the maximum size of a USB transfer request in bytes. Modify this value to affect performance under different combinations of operating system and device.
#define DAQmx_DO_MemMapEnable                                            0x296B // Specifies for NI-DAQmx to map hardware registers to the memory space of the application, if possible. Normally, NI-DAQmx maps hardware registers to memory accessible only to the kernel. Mapping the registers to the memory space of the application increases performance. However, if the application accesses the memory space mapped to the registers, it can adversely affect the operation of the device and possibly res...
#define DAQmx_DO_GenerateOn                                              0x2969 // Specifies on which edge of the sample clock to generate samples.
#define DAQmx_CI_Max                                                     0x189C // Specifies the maximum value you expect to measure. This value is in the units you specify with a units property. When you query this property, it returns the coerced maximum value that the hardware can measure with the current settings.
#define DAQmx_CI_Min                                                     0x189D // Specifies the minimum value you expect to measure. This value is in the units you specify with a units property. When you query this property, it returns the coerced minimum value that the hardware can measure with the current settings.
#define DAQmx_CI_CustomScaleName                                         0x189E // Specifies the name of a custom scale for the channel.
#define DAQmx_CI_MeasType                                                0x18A0 // Indicates the measurement to take with the channel.
#define DAQmx_CI_Freq_Units                                              0x18A1 // Specifies the units to use to return frequency measurements.
#define DAQmx_CI_Freq_Term                                               0x18A2 // Specifies the input terminal of the signal to measure.
#define DAQmx_CI_Freq_StartingEdge                                       0x0799 // Specifies between which edges to measure the frequency of the signal.
#define DAQmx_CI_Freq_MeasMeth                                           0x0144 // Specifies the method to use to measure the frequency of the signal.
#define DAQmx_CI_Freq_MeasTime                                           0x0145 // Specifies in seconds the length of time to measure the frequency of the signal if Method is DAQmx_Val_HighFreq2Ctr. Measurement accuracy increases with increased measurement time and with increased signal frequency. If you measure a high-frequency signal for too long, however, the count register could roll over, which results in an incorrect measurement.
#define DAQmx_CI_Freq_Div                                                0x0147 // Specifies the value by which to divide the input signal if  Method is DAQmx_Val_LargeRng2Ctr. The larger the divisor, the more accurate the measurement. However, too large a value could cause the count register to roll over, which results in an incorrect measurement.
#define DAQmx_CI_Freq_DigFltr_Enable                                     0x21E7 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_Freq_DigFltr_MinPulseWidth                              0x21E8 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_Freq_DigFltr_TimebaseSrc                                0x21E9 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_Freq_DigFltr_TimebaseRate                               0x21EA // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_Freq_DigSync_Enable                                     0x21EB // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_Period_Units                                            0x18A3 // Specifies the unit to use to return period measurements.
#define DAQmx_CI_Period_Term                                             0x18A4 // Specifies the input terminal of the signal to measure.
#define DAQmx_CI_Period_StartingEdge                                     0x0852 // Specifies between which edges to measure the period of the signal.
#define DAQmx_CI_Period_MeasMeth                                         0x192C // Specifies the method to use to measure the period of the signal.
#define DAQmx_CI_Period_MeasTime                                         0x192D // Specifies in seconds the length of time to measure the period of the signal if Method is DAQmx_Val_HighFreq2Ctr. Measurement accuracy increases with increased measurement time and with increased signal frequency. If you measure a high-frequency signal for too long, however, the count register could roll over, which results in an incorrect measurement.
#define DAQmx_CI_Period_Div                                              0x192E // Specifies the value by which to divide the input signal if Method is DAQmx_Val_LargeRng2Ctr. The larger the divisor, the more accurate the measurement. However, too large a value could cause the count register to roll over, which results in an incorrect measurement.
#define DAQmx_CI_Period_DigFltr_Enable                                   0x21EC // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_Period_DigFltr_MinPulseWidth                            0x21ED // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_Period_DigFltr_TimebaseSrc                              0x21EE // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_Period_DigFltr_TimebaseRate                             0x21EF // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_Period_DigSync_Enable                                   0x21F0 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_CountEdges_Term                                         0x18C7 // Specifies the input terminal of the signal to measure.
#define DAQmx_CI_CountEdges_Dir                                          0x0696 // Specifies whether to increment or decrement the counter on each edge.
#define DAQmx_CI_CountEdges_DirTerm                                      0x21E1 // Specifies the source terminal of the digital signal that controls the count direction if Direction is DAQmx_Val_ExtControlled.
#define DAQmx_CI_CountEdges_CountDir_DigFltr_Enable                      0x21F1 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_CountEdges_CountDir_DigFltr_MinPulseWidth               0x21F2 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_CountEdges_CountDir_DigFltr_TimebaseSrc                 0x21F3 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_CountEdges_CountDir_DigFltr_TimebaseRate                0x21F4 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_CountEdges_CountDir_DigSync_Enable                      0x21F5 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_CountEdges_InitialCnt                                   0x0698 // Specifies the starting value from which to count.
#define DAQmx_CI_CountEdges_ActiveEdge                                   0x0697 // Specifies on which edges to increment or decrement the counter.
#define DAQmx_CI_CountEdges_DigFltr_Enable                               0x21F6 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_CountEdges_DigFltr_MinPulseWidth                        0x21F7 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_CountEdges_DigFltr_TimebaseSrc                          0x21F8 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_CountEdges_DigFltr_TimebaseRate                         0x21F9 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_CountEdges_DigSync_Enable                               0x21FA // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_AngEncoder_Units                                        0x18A6 // Specifies the units to use to return angular position measurements from the channel.
#define DAQmx_CI_AngEncoder_PulsesPerRev                                 0x0875 // Specifies the number of pulses the encoder generates per revolution. This value is the number of pulses on either signal A or signal B, not the total number of pulses on both signal A and signal B.
#define DAQmx_CI_AngEncoder_InitialAngle                                 0x0881 // Specifies the starting angle of the encoder. This value is in the units you specify with Units.
#define DAQmx_CI_LinEncoder_Units                                        0x18A9 // Specifies the units to use to return linear encoder measurements from the channel.
#define DAQmx_CI_LinEncoder_DistPerPulse                                 0x0911 // Specifies the distance to measure for each pulse the encoder generates on signal A or signal B. This value is in the units you specify with Units.
#define DAQmx_CI_LinEncoder_InitialPos                                   0x0915 // Specifies the position of the encoder when the measurement begins. This value is in the units you specify with Units.
#define DAQmx_CI_Encoder_DecodingType                                    0x21E6 // Specifies how to count and interpret the pulses the encoder generates on signal A and signal B. DAQmx_Val_X1, DAQmx_Val_X2, and DAQmx_Val_X4 are valid for quadrature encoders only. DAQmx_Val_TwoPulseCounting is valid for two-pulse encoders only.
#define DAQmx_CI_Encoder_AInputTerm                                      0x219D // Specifies the terminal to which signal A is connected.
#define DAQmx_CI_Encoder_AInput_DigFltr_Enable                           0x21FB // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_Encoder_AInput_DigFltr_MinPulseWidth                    0x21FC // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_Encoder_AInput_DigFltr_TimebaseSrc                      0x21FD // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_Encoder_AInput_DigFltr_TimebaseRate                     0x21FE // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_Encoder_AInput_DigSync_Enable                           0x21FF // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_Encoder_BInputTerm                                      0x219E // Specifies the terminal to which signal B is connected.
#define DAQmx_CI_Encoder_BInput_DigFltr_Enable                           0x2200 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_Encoder_BInput_DigFltr_MinPulseWidth                    0x2201 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_Encoder_BInput_DigFltr_TimebaseSrc                      0x2202 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_Encoder_BInput_DigFltr_TimebaseRate                     0x2203 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_Encoder_BInput_DigSync_Enable                           0x2204 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_Encoder_ZInputTerm                                      0x219F // Specifies the terminal to which signal Z is connected.
#define DAQmx_CI_Encoder_ZInput_DigFltr_Enable                           0x2205 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_Encoder_ZInput_DigFltr_MinPulseWidth                    0x2206 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_Encoder_ZInput_DigFltr_TimebaseSrc                      0x2207 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_Encoder_ZInput_DigFltr_TimebaseRate                     0x2208 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_Encoder_ZInput_DigSync_Enable                           0x2209 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_Encoder_ZIndexEnable                                    0x0890 // Specifies whether to use Z indexing for the channel.
#define DAQmx_CI_Encoder_ZIndexVal                                       0x0888 // Specifies the value to which to reset the measurement when signal Z is high and signal A and signal B are at the states you specify with Z Index Phase. Specify this value in the units of the measurement.
#define DAQmx_CI_Encoder_ZIndexPhase                                     0x0889 // Specifies the states at which signal A and signal B must be while signal Z is high for NI-DAQmx to reset the measurement. If signal Z is never high while signal A and signal B are high, for example, you must choose a phase other than DAQmx_Val_AHighBHigh.
#define DAQmx_CI_PulseWidth_Units                                        0x0823 // Specifies the units to use to return pulse width measurements.
#define DAQmx_CI_PulseWidth_Term                                         0x18AA // Specifies the input terminal of the signal to measure.
#define DAQmx_CI_PulseWidth_StartingEdge                                 0x0825 // Specifies on which edge of the input signal to begin each pulse width measurement.
#define DAQmx_CI_PulseWidth_DigFltr_Enable                               0x220A // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_PulseWidth_DigFltr_MinPulseWidth                        0x220B // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_PulseWidth_DigFltr_TimebaseSrc                          0x220C // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_PulseWidth_DigFltr_TimebaseRate                         0x220D // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_PulseWidth_DigSync_Enable                               0x220E // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_TwoEdgeSep_Units                                        0x18AC // Specifies the units to use to return two-edge separation measurements from the channel.
#define DAQmx_CI_TwoEdgeSep_FirstTerm                                    0x18AD // Specifies the source terminal of the digital signal that starts each measurement.
#define DAQmx_CI_TwoEdgeSep_FirstEdge                                    0x0833 // Specifies on which edge of the first signal to start each measurement.
#define DAQmx_CI_TwoEdgeSep_First_DigFltr_Enable                         0x220F // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_TwoEdgeSep_First_DigFltr_MinPulseWidth                  0x2210 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_TwoEdgeSep_First_DigFltr_TimebaseSrc                    0x2211 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_TwoEdgeSep_First_DigFltr_TimebaseRate                   0x2212 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_TwoEdgeSep_First_DigSync_Enable                         0x2213 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_TwoEdgeSep_SecondTerm                                   0x18AE // Specifies the source terminal of the digital signal that stops each measurement.
#define DAQmx_CI_TwoEdgeSep_SecondEdge                                   0x0834 // Specifies on which edge of the second signal to stop each measurement.
#define DAQmx_CI_TwoEdgeSep_Second_DigFltr_Enable                        0x2214 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_TwoEdgeSep_Second_DigFltr_MinPulseWidth                 0x2215 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_TwoEdgeSep_Second_DigFltr_TimebaseSrc                   0x2216 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_TwoEdgeSep_Second_DigFltr_TimebaseRate                  0x2217 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_TwoEdgeSep_Second_DigSync_Enable                        0x2218 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_SemiPeriod_Units                                        0x18AF // Specifies the units to use to return semi-period measurements.
#define DAQmx_CI_SemiPeriod_Term                                         0x18B0 // Specifies the input terminal of the signal to measure.
#define DAQmx_CI_SemiPeriod_StartingEdge                                 0x22FE // Specifies on which edge of the input signal to begin semi-period measurement. Semi-period measurements alternate between high time and low time, starting on this edge.
#define DAQmx_CI_SemiPeriod_DigFltr_Enable                               0x2219 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_SemiPeriod_DigFltr_MinPulseWidth                        0x221A // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_SemiPeriod_DigFltr_TimebaseSrc                          0x221B // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_SemiPeriod_DigFltr_TimebaseRate                         0x221C // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_SemiPeriod_DigSync_Enable                               0x221D // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_Timestamp_Units                                         0x22B3 // Specifies the units to use to return timestamp measurements.
#define DAQmx_CI_Timestamp_InitialSeconds                                0x22B4 // Specifies the number of seconds that elapsed since the beginning of the current year. This value is ignored if  Synchronization Method is DAQmx_Val_IRIGB.
#define DAQmx_CI_GPS_SyncMethod                                          0x1092 // Specifies the method to use to synchronize the counter to a GPS receiver.
#define DAQmx_CI_GPS_SyncSrc                                             0x1093 // Specifies the terminal to which the GPS synchronization signal is connected.
#define DAQmx_CI_CtrTimebaseSrc                                          0x0143 // Specifies the terminal of the timebase to use for the counter.
#define DAQmx_CI_CtrTimebaseRate                                         0x18B2 // Specifies in Hertz the frequency of the counter timebase. Specifying the rate of a counter timebase allows you to take measurements in terms of time or frequency rather than in ticks of the timebase. If you use an external timebase and do not specify the rate, you can take measurements only in terms of ticks of the timebase.
#define DAQmx_CI_CtrTimebaseActiveEdge                                   0x0142 // Specifies whether a timebase cycle is from rising edge to rising edge or from falling edge to falling edge.
#define DAQmx_CI_CtrTimebase_DigFltr_Enable                              0x2271 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CI_CtrTimebase_DigFltr_MinPulseWidth                       0x2272 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CI_CtrTimebase_DigFltr_TimebaseSrc                         0x2273 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CI_CtrTimebase_DigFltr_TimebaseRate                        0x2274 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CI_CtrTimebase_DigSync_Enable                              0x2275 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CI_Count                                                   0x0148 // Indicates the current value of the count register.
#define DAQmx_CI_OutputState                                             0x0149 // Indicates the current state of the out terminal of the counter.
#define DAQmx_CI_TCReached                                               0x0150 // Indicates whether the counter rolled over. When you query this property, NI-DAQmx resets it to FALSE.
#define DAQmx_CI_CtrTimebaseMasterTimebaseDiv                            0x18B3 // Specifies the divisor for an external counter timebase. You can divide the counter timebase in order to measure slower signals without causing the count register to roll over.
#define DAQmx_CI_DataXferMech                                            0x0200 // Specifies the data transfer mode for the channel.
#define DAQmx_CI_UsbXferReqSize                                          0x2A92 // Specifies the maximum size of a USB transfer request in bytes. Modify this value to affect performance under different combinations of operating system and device.
#define DAQmx_CI_NumPossiblyInvalidSamps                                 0x193C // Indicates the number of samples that the device might have overwritten before it could transfer them to the buffer.
#define DAQmx_CI_DupCountPrevent                                         0x21AC // Specifies whether to enable duplicate count prevention for the channel. Duplicate count prevention is enabled by default. Setting  Prescaler disables duplicate count prevention unless you explicitly enable it.
#define DAQmx_CI_Prescaler                                               0x2239 // Specifies the divisor to apply to the signal you connect to the counter source terminal. Scaled data that you read takes this setting into account. You should use a prescaler only when you connect an external signal to the counter source terminal and when that signal has a higher frequency than the fastest onboard timebase. Setting this value disables duplicate count prevention unless you explicitly set Duplicate ...
#define DAQmx_CO_OutputType                                              0x18B5 // Indicates how to define pulses generated on the channel.
#define DAQmx_CO_Pulse_IdleState                                         0x1170 // Specifies the resting state of the output terminal.
#define DAQmx_CO_Pulse_Term                                              0x18E1 // Specifies on which terminal to generate pulses.
#define DAQmx_CO_Pulse_Time_Units                                        0x18D6 // Specifies the units in which to define high and low pulse time.
#define DAQmx_CO_Pulse_HighTime                                          0x18BA // Specifies the amount of time that the pulse is at a high voltage. This value is in the units you specify with Units or when you create the channel.
#define DAQmx_CO_Pulse_LowTime                                           0x18BB // Specifies the amount of time that the pulse is at a low voltage. This value is in the units you specify with Units or when you create the channel.
#define DAQmx_CO_Pulse_Time_InitialDelay                                 0x18BC // Specifies in seconds the amount of time to wait before generating the first pulse.
#define DAQmx_CO_Pulse_DutyCyc                                           0x1176 // Specifies the duty cycle of the pulses. The duty cycle of a signal is the width of the pulse divided by period. NI-DAQmx uses this ratio and the pulse frequency to determine the width of the pulses and the delay between pulses.
#define DAQmx_CO_Pulse_Freq_Units                                        0x18D5 // Specifies the units in which to define pulse frequency.
#define DAQmx_CO_Pulse_Freq                                              0x1178 // Specifies the frequency of the pulses to generate. This value is in the units you specify with Units or when you create the channel.
#define DAQmx_CO_Pulse_Freq_InitialDelay                                 0x0299 // Specifies in seconds the amount of time to wait before generating the first pulse.
#define DAQmx_CO_Pulse_HighTicks                                         0x1169 // Specifies the number of ticks the pulse is high.
#define DAQmx_CO_Pulse_LowTicks                                          0x1171 // Specifies the number of ticks the pulse is low.
#define DAQmx_CO_Pulse_Ticks_InitialDelay                                0x0298 // Specifies the number of ticks to wait before generating the first pulse.
#define DAQmx_CO_CtrTimebaseSrc                                          0x0339 // Specifies the terminal of the timebase to use for the counter. Typically, NI-DAQmx uses one of the internal counter timebases when generating pulses. Use this property to specify an external timebase and produce custom pulse widths that are not possible using the internal timebases.
#define DAQmx_CO_CtrTimebaseRate                                         0x18C2 // Specifies in Hertz the frequency of the counter timebase. Specifying the rate of a counter timebase allows you to define output pulses in seconds rather than in ticks of the timebase. If you use an external timebase and do not specify the rate, you can define output pulses only in ticks of the timebase.
#define DAQmx_CO_CtrTimebaseActiveEdge                                   0x0341 // Specifies whether a timebase cycle is from rising edge to rising edge or from falling edge to falling edge.
#define DAQmx_CO_CtrTimebase_DigFltr_Enable                              0x2276 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_CO_CtrTimebase_DigFltr_MinPulseWidth                       0x2277 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_CO_CtrTimebase_DigFltr_TimebaseSrc                         0x2278 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_CO_CtrTimebase_DigFltr_TimebaseRate                        0x2279 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_CO_CtrTimebase_DigSync_Enable                              0x227A // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_CO_Count                                                   0x0293 // Indicates the current value of the count register.
#define DAQmx_CO_OutputState                                             0x0294 // Indicates the current state of the output terminal of the counter.
#define DAQmx_CO_AutoIncrCnt                                             0x0295 // Specifies a number of timebase ticks by which to increment each successive pulse.
#define DAQmx_CO_CtrTimebaseMasterTimebaseDiv                            0x18C3 // Specifies the divisor for an external counter timebase. You can divide the counter timebase in order to generate slower signals without causing the count register to roll over.
#define DAQmx_CO_PulseDone                                               0x190E // Indicates if the task completed pulse generation. Use this value for retriggerable pulse generation when you need to determine if the device generated the current pulse. When you query this property, NI-DAQmx resets it to FALSE.
#define DAQmx_CO_ConstrainedGenMode                                      0x29F2 // Specifies constraints to apply when the counter generates pulses. Constraining the counter reduces the device resources required for counter operation. Constraining the counter can also allow additional analog or counter tasks on the device to run concurrently. For continuous counter tasks, NI-DAQmx consumes no device resources when the counter is constrained. For finite counter tasks, resource use increases with ...
#define DAQmx_CO_Prescaler                                               0x226D // Specifies the divisor to apply to the signal you connect to the counter source terminal. Pulse generations defined by frequency or time take this setting into account, but pulse generations defined by ticks do not. You should use a prescaler only when you connect an external signal to the counter source terminal and when that signal has a higher frequency than the fastest onboard timebase.
#define DAQmx_CO_RdyForNewVal                                            0x22FF // Indicates whether the counter is ready for new continuous pulse train values.
#define DAQmx_ChanType                                                   0x187F // Indicates the type of the virtual channel.
#define DAQmx_PhysicalChanName                                           0x18F5 // Specifies the name of the physical channel upon which this virtual channel is based.
#define DAQmx_ChanDescr                                                  0x1926 // Specifies a user-defined description for the channel.
#define DAQmx_ChanIsGlobal                                               0x2304 // Indicates whether the channel is a global channel.

//********** Export Signal Attributes **********
#define DAQmx_Exported_AIConvClk_OutputTerm                              0x1687 // Specifies the terminal to which to route the AI Convert Clock.
#define DAQmx_Exported_AIConvClk_Pulse_Polarity                          0x1688 // Indicates the polarity of the exported AI Convert Clock. The polarity is fixed and independent of the active edge of the source of the AI Convert Clock.
#define DAQmx_Exported_10MHzRefClk_OutputTerm                            0x226E // Specifies the terminal to which to route the 10MHz Clock.
#define DAQmx_Exported_20MHzTimebase_OutputTerm                          0x1657 // Specifies the terminal to which to route the 20MHz Timebase.
#define DAQmx_Exported_SampClk_OutputBehavior                            0x186B // Specifies whether the exported Sample Clock issues a pulse at the beginning of a sample or changes to a high state for the duration of the sample.
#define DAQmx_Exported_SampClk_OutputTerm                                0x1663 // Specifies the terminal to which to route the Sample Clock.
#define DAQmx_Exported_SampClk_DelayOffset                               0x21C4 // Specifies in seconds the amount of time to offset the exported Sample clock.  Refer to timing diagrams for generation applications in the device documentation for more information about this value.
#define DAQmx_Exported_SampClk_Pulse_Polarity                            0x1664 // Specifies the polarity of the exported Sample Clock if Output Behavior is DAQmx_Val_Pulse.
#define DAQmx_Exported_SampClkTimebase_OutputTerm                        0x18F9 // Specifies the terminal to which to route the Sample Clock Timebase.
#define DAQmx_Exported_DividedSampClkTimebase_OutputTerm                 0x21A1 // Specifies the terminal to which to route the Divided Sample Clock Timebase.
#define DAQmx_Exported_AdvTrig_OutputTerm                                0x1645 // Specifies the terminal to which to route the Advance Trigger.
#define DAQmx_Exported_AdvTrig_Pulse_Polarity                            0x1646 // Indicates the polarity of the exported Advance Trigger.
#define DAQmx_Exported_AdvTrig_Pulse_WidthUnits                          0x1647 // Specifies the units of Width Value.
#define DAQmx_Exported_AdvTrig_Pulse_Width                               0x1648 // Specifies the width of an exported Advance Trigger pulse. Specify this value in the units you specify with Width Units.
#define DAQmx_Exported_PauseTrig_OutputTerm                              0x1615 // Specifies the terminal to which to route the Pause Trigger.
#define DAQmx_Exported_PauseTrig_Lvl_ActiveLvl                           0x1616 // Specifies the active level of the exported Pause Trigger.
#define DAQmx_Exported_RefTrig_OutputTerm                                0x0590 // Specifies the terminal to which to route the Reference Trigger.
#define DAQmx_Exported_RefTrig_Pulse_Polarity                            0x0591 // Specifies the polarity of the exported Reference Trigger.
#define DAQmx_Exported_StartTrig_OutputTerm                              0x0584 // Specifies the terminal to which to route the Start Trigger.
#define DAQmx_Exported_StartTrig_Pulse_Polarity                          0x0585 // Specifies the polarity of the exported Start Trigger.
#define DAQmx_Exported_AdvCmpltEvent_OutputTerm                          0x1651 // Specifies the terminal to which to route the Advance Complete Event.
#define DAQmx_Exported_AdvCmpltEvent_Delay                               0x1757 // Specifies the output signal delay in periods of the sample clock.
#define DAQmx_Exported_AdvCmpltEvent_Pulse_Polarity                      0x1652 // Specifies the polarity of the exported Advance Complete Event.
#define DAQmx_Exported_AdvCmpltEvent_Pulse_Width                         0x1654 // Specifies the width of the exported Advance Complete Event pulse.
#define DAQmx_Exported_AIHoldCmpltEvent_OutputTerm                       0x18ED // Specifies the terminal to which to route the AI Hold Complete Event.
#define DAQmx_Exported_AIHoldCmpltEvent_PulsePolarity                    0x18EE // Specifies the polarity of an exported AI Hold Complete Event pulse.
#define DAQmx_Exported_ChangeDetectEvent_OutputTerm                      0x2197 // Specifies the terminal to which to route the Change Detection Event.
#define DAQmx_Exported_ChangeDetectEvent_Pulse_Polarity                  0x2303 // Specifies the polarity of an exported Change Detection Event pulse.
#define DAQmx_Exported_CtrOutEvent_OutputTerm                            0x1717 // Specifies the terminal to which to route the Counter Output Event.
#define DAQmx_Exported_CtrOutEvent_OutputBehavior                        0x174F // Specifies whether the exported Counter Output Event pulses or changes from one state to the other when the counter reaches terminal count.
#define DAQmx_Exported_CtrOutEvent_Pulse_Polarity                        0x1718 // Specifies the polarity of the pulses at the output terminal of the counter when Output Behavior is DAQmx_Val_Pulse. NI-DAQmx ignores this property if Output Behavior is DAQmx_Val_Toggle.
#define DAQmx_Exported_CtrOutEvent_Toggle_IdleState                      0x186A // Specifies the initial state of the output terminal of the counter when Output Behavior is DAQmx_Val_Toggle. The terminal enters this state when NI-DAQmx commits the task.
#define DAQmx_Exported_HshkEvent_OutputTerm                              0x22BA // Specifies the terminal to which to route the Handshake Event.
#define DAQmx_Exported_HshkEvent_OutputBehavior                          0x22BB // Specifies the output behavior of the Handshake Event.
#define DAQmx_Exported_HshkEvent_Delay                                   0x22BC // Specifies the number of seconds to delay after the Handshake Trigger deasserts before asserting the Handshake Event.
#define DAQmx_Exported_HshkEvent_Interlocked_AssertedLvl                 0x22BD // Specifies the asserted level of the exported Handshake Event if Output Behavior is DAQmx_Val_Interlocked.
#define DAQmx_Exported_HshkEvent_Interlocked_AssertOnStart               0x22BE // Specifies to assert the Handshake Event when the task starts if Output Behavior is DAQmx_Val_Interlocked.
#define DAQmx_Exported_HshkEvent_Interlocked_DeassertDelay               0x22BF // Specifies in seconds the amount of time to wait after the Handshake Trigger asserts before deasserting the Handshake Event if Output Behavior is DAQmx_Val_Interlocked.
#define DAQmx_Exported_HshkEvent_Pulse_Polarity                          0x22C0 // Specifies the polarity of the exported Handshake Event if Output Behavior is DAQmx_Val_Pulse.
#define DAQmx_Exported_HshkEvent_Pulse_Width                             0x22C1 // Specifies in seconds the pulse width of the exported Handshake Event if Output Behavior is DAQmx_Val_Pulse.
#define DAQmx_Exported_RdyForXferEvent_OutputTerm                        0x22B5 // Specifies the terminal to which to route the Ready for Transfer Event.
#define DAQmx_Exported_RdyForXferEvent_Lvl_ActiveLvl                     0x22B6 // Specifies the active level of the exported Ready for Transfer Event.
#define DAQmx_Exported_RdyForXferEvent_DeassertCond                      0x2963 // Specifies when the ready for transfer event deasserts.
#define DAQmx_Exported_RdyForXferEvent_DeassertCondCustomThreshold       0x2964 // Specifies in samples the threshold below which the Ready for Transfer Event deasserts. This threshold is an amount of space available in the onboard memory of the device. Deassert Condition must be DAQmx_Val_OnbrdMemCustomThreshold to use a custom threshold.
#define DAQmx_Exported_DataActiveEvent_OutputTerm                        0x1633 // Specifies the terminal to which to export the Data Active Event.
#define DAQmx_Exported_DataActiveEvent_Lvl_ActiveLvl                     0x1634 // Specifies the polarity of the exported Data Active Event.
#define DAQmx_Exported_RdyForStartEvent_OutputTerm                       0x1609 // Specifies the terminal to which to route the Ready for Start Event.
#define DAQmx_Exported_RdyForStartEvent_Lvl_ActiveLvl                    0x1751 // Specifies the polarity of the exported Ready for Start Event.
#define DAQmx_Exported_SyncPulseEvent_OutputTerm                         0x223C // Specifies the terminal to which to route the Synchronization Pulse Event.
#define DAQmx_Exported_WatchdogExpiredEvent_OutputTerm                   0x21AA // Specifies the terminal  to which to route the Watchdog Timer Expired Event.

//********** Device Attributes **********
#define DAQmx_Dev_IsSimulated                                            0x22CA // Indicates if the device is a simulated device.
#define DAQmx_Dev_ProductCategory                                        0x29A9 // Indicates the product category of the device. This category corresponds to the category displayed in MAX when creating NI-DAQmx simulated devices.
#define DAQmx_Dev_ProductType                                            0x0631 // Indicates the product name of the device.
#define DAQmx_Dev_ProductNum                                             0x231D // Indicates the unique hardware identification number for the device.
#define DAQmx_Dev_SerialNum                                              0x0632 // Indicates the serial number of the device. This value is zero if the device does not have a serial number.
#define DAQmx_Carrier_SerialNum                                          0x2A8A // Indicates the serial number of the device carrier. This value is zero if the carrier does not have a serial number.
#define DAQmx_Dev_Chassis_ModuleDevNames                                 0x29B6 // Indicates an array containing the names of the modules in the chassis.
#define DAQmx_Dev_AnlgTrigSupported                                      0x2984 // Indicates if the device supports analog triggering.
#define DAQmx_Dev_DigTrigSupported                                       0x2985 // Indicates if the device supports digital triggering.
#define DAQmx_Dev_AI_PhysicalChans                                       0x231E // Indicates an array containing the names of the analog input physical channels available on the device.
#define DAQmx_Dev_AI_MaxSingleChanRate                                   0x298C // Indicates the maximum rate for an analog input task if the task contains only a single channel from this device.
#define DAQmx_Dev_AI_MaxMultiChanRate                                    0x298D // Indicates the maximum rate for an analog input task if the task contains multiple channels from this device. For multiplexed devices, divide this rate by the number of channels to determine the maximum sampling rate.
#define DAQmx_Dev_AI_MinRate                                             0x298E // Indicates the minimum rate for an analog input task on this device. NI-DAQmx returns a warning or error if you attempt to sample at a slower rate.
#define DAQmx_Dev_AI_SimultaneousSamplingSupported                       0x298F // Indicates if the device supports simultaneous sampling.
#define DAQmx_Dev_AI_TrigUsage                                           0x2986 // Indicates the triggers supported by this device for an analog input task.
#define DAQmx_Dev_AI_VoltageRngs                                         0x2990 // Indicates pairs of input voltage ranges supported by this device. Each pair consists of the low value, followed by the high value.
#define DAQmx_Dev_AI_VoltageIntExcitDiscreteVals                         0x29C9 // Indicates the set of discrete internal voltage excitation values supported by this device. If the device supports ranges of internal excitation values, use Range Values to determine supported excitation values.
#define DAQmx_Dev_AI_VoltageIntExcitRangeVals                            0x29CA // Indicates pairs of internal voltage excitation ranges supported by this device. Each pair consists of the low value, followed by the high value. If the device supports a set of discrete internal excitation values, use Discrete Values to determine the supported excitation values.
#define DAQmx_Dev_AI_CurrentRngs                                         0x2991 // Indicates the pairs of current input ranges supported by this device. Each pair consists of the low value, followed by the high value.
#define DAQmx_Dev_AI_CurrentIntExcitDiscreteVals                         0x29CB // Indicates the set of discrete internal current excitation values supported by this device.
#define DAQmx_Dev_AI_FreqRngs                                            0x2992 // Indicates the pairs of frequency input ranges supported by this device. Each pair consists of the low value, followed by the high value.
#define DAQmx_Dev_AI_Gains                                               0x2993 // Indicates the input gain settings supported by this device.
#define DAQmx_Dev_AI_Couplings                                           0x2994 // Indicates the coupling types supported by this device.
#define DAQmx_Dev_AI_LowpassCutoffFreqDiscreteVals                       0x2995 // Indicates the set of discrete lowpass cutoff frequencies supported by this device. If the device supports ranges of lowpass cutoff frequencies, use Range Values to determine supported frequencies.
#define DAQmx_Dev_AI_LowpassCutoffFreqRangeVals                          0x29CF // Indicates pairs of lowpass cutoff frequency ranges supported by this device. Each pair consists of the low value, followed by the high value. If the device supports a set of discrete lowpass cutoff frequencies, use Discrete Values to determine the supported  frequencies.
#define DAQmx_Dev_AO_PhysicalChans                                       0x231F // Indicates an array containing the names of the analog output physical channels available on the device.
#define DAQmx_Dev_AO_SampClkSupported                                    0x2996 // Indicates if the device supports the sample clock timing  type for analog output tasks.
#define DAQmx_Dev_AO_MaxRate                                             0x2997 // Indicates the maximum analog output rate of the device.
#define DAQmx_Dev_AO_MinRate                                             0x2998 // Indicates the minimum analog output rate of the device.
#define DAQmx_Dev_AO_TrigUsage                                           0x2987 // Indicates the triggers supported by this device for analog output tasks.
#define DAQmx_Dev_AO_VoltageRngs                                         0x299B // Indicates pairs of output voltage ranges supported by this device. Each pair consists of the low value, followed by the high value.
#define DAQmx_Dev_AO_CurrentRngs                                         0x299C // Indicates pairs of output current ranges supported by this device. Each pair consists of the low value, followed by the high value.
#define DAQmx_Dev_AO_Gains                                               0x299D // Indicates the output gain settings supported by this device.
#define DAQmx_Dev_DI_Lines                                               0x2320 // Indicates an array containing the names of the digital input lines available on the device.
#define DAQmx_Dev_DI_Ports                                               0x2321 // Indicates an array containing the names of the digital input ports available on the device.
#define DAQmx_Dev_DI_MaxRate                                             0x2999 // Indicates the maximum digital input rate of the device.
#define DAQmx_Dev_DI_TrigUsage                                           0x2988 // Indicates the triggers supported by this device for digital input tasks.
#define DAQmx_Dev_DO_Lines                                               0x2322 // Indicates an array containing the names of the digital output lines available on the device.
#define DAQmx_Dev_DO_Ports                                               0x2323 // Indicates an array containing the names of the digital output ports available on the device.
#define DAQmx_Dev_DO_MaxRate                                             0x299A // Indicates the maximum digital output rate of the device.
#define DAQmx_Dev_DO_TrigUsage                                           0x2989 // Indicates the triggers supported by this device for digital output tasks.
#define DAQmx_Dev_CI_PhysicalChans                                       0x2324 // Indicates an array containing the names of the counter input physical channels available on the device.
#define DAQmx_Dev_CI_TrigUsage                                           0x298A // Indicates the triggers supported by this device for counter input tasks.
#define DAQmx_Dev_CI_SampClkSupported                                    0x299E // Indicates if the device supports the sample clock timing type for counter input tasks.
#define DAQmx_Dev_CI_MaxSize                                             0x299F // Indicates in bits the size of the counters on the device.
#define DAQmx_Dev_CI_MaxTimebase                                         0x29A0 // Indicates in hertz the maximum counter timebase frequency.
#define DAQmx_Dev_CO_PhysicalChans                                       0x2325 // Indicates an array containing the names of the counter output physical channels available on the device.
#define DAQmx_Dev_CO_TrigUsage                                           0x298B // Indicates the triggers supported by this device for counter output tasks.
#define DAQmx_Dev_CO_MaxSize                                             0x29A1 // Indicates in bits the size of the counters on the device.
#define DAQmx_Dev_CO_MaxTimebase                                         0x29A2 // Indicates in hertz the maximum counter timebase frequency.
#define DAQmx_Dev_NumDMAChans                                            0x233C // Indicates the number of DMA channels on the device.
#define DAQmx_Dev_BusType                                                0x2326 // Indicates the bus type of the device.
#define DAQmx_Dev_PCI_BusNum                                             0x2327 // Indicates the PCI bus number of the device.
#define DAQmx_Dev_PCI_DevNum                                             0x2328 // Indicates the PCI slot number of the device.
#define DAQmx_Dev_PXI_ChassisNum                                         0x2329 // Indicates the PXI chassis number of the device, as identified in MAX.
#define DAQmx_Dev_PXI_SlotNum                                            0x232A // Indicates the PXI slot number of the device.
#define DAQmx_Dev_CompactDAQ_ChassisDevName                              0x29B7 // Indicates the name of the CompactDAQ chassis that contains this module.
#define DAQmx_Dev_CompactDAQ_SlotNum                                     0x29B8 // Indicates the slot number in which this module is located in the CompactDAQ chassis.
#define DAQmx_Dev_TCPIP_Hostname                                         0x2A8B // Indicates the IPv4 hostname of the device.
#define DAQmx_Dev_TCPIP_EthernetIP                                       0x2A8C // Indicates the IPv4 address of the Ethernet interface. This property returns an empty string if the Ethernet interface cannot acquire an address.
#define DAQmx_Dev_TCPIP_WirelessIP                                       0x2A8D // Indicates the IPv4 address of the wireless interface.This property returns an empty string if the wireless interface cannot acquire an address.
#define DAQmx_Dev_Terminals                                              0x2A40 // Indicates a list of all terminals on the device.

//********** Read Attributes **********
#define DAQmx_Read_RelativeTo                                            0x190A // Specifies the point in the buffer at which to begin a read operation. If you also specify an offset with Offset, the read operation begins at that offset relative to the point you select with this property. The default value is DAQmx_Val_CurrReadPos unless you configure a Reference Trigger for the task. If you configure a Reference Trigger, the default value is DAQmx_Val_FirstPretrigSamp.
#define DAQmx_Read_Offset                                                0x190B // Specifies an offset in samples per channel at which to begin a read operation. This offset is relative to the location you specify with RelativeTo.
#define DAQmx_Read_ChannelsToRead                                        0x1823 // Specifies a subset of channels in the task from which to read.
#define DAQmx_Read_ReadAllAvailSamp                                      0x1215 // Specifies whether subsequent read operations read all samples currently available in the buffer or wait for the buffer to become full before reading. NI-DAQmx uses this setting for finite acquisitions and only when the number of samples to read is -1. For continuous acquisitions when the number of samples to read is -1, a read operation always reads all samples currently available in the buffer.
#define DAQmx_Read_AutoStart                                             0x1826 // Specifies if an NI-DAQmx Read function automatically starts the task  if you did not start the task explicitly by using DAQmxStartTask(). The default value is TRUE. When  an NI-DAQmx Read function starts a finite acquisition task, it also stops the task after reading the last sample.
#define DAQmx_Read_OverWrite                                             0x1211 // Specifies whether to overwrite samples in the buffer that you have not yet read.
#define DAQmx_Read_CurrReadPos                                           0x1221 // Indicates in samples per channel the current position in the buffer.
#define DAQmx_Read_AvailSampPerChan                                      0x1223 // Indicates the number of samples available to read per channel. This value is the same for all channels in the task.
#define DAQmx_Read_TotalSampPerChanAcquired                              0x192A // Indicates the total number of samples acquired by each channel. NI-DAQmx returns a single value because this value is the same for all channels.
#define DAQmx_Read_CommonModeRangeErrorChansExist                        0x2A98 // Indicates if the device(s) detected a common mode range violation for any virtual channel in the task. Common mode range violation occurs when the voltage of either the positive terminal or negative terminal to ground are out of range. Reading this property clears the common mode range violation status for all channels in the task. You must read this property before you read Common Mode Range Error Channels. Other...
#define DAQmx_Read_CommonModeRangeErrorChans                             0x2A99 // Indicates the names of any virtual channels in the task for which the device(s) detected a common mode range violation. You must read Common Mode Range Error Channels Exist before you read this property. Otherwise, you will receive an error.
#define DAQmx_Read_OvercurrentChansExist                                 0x29E6 // Indicates if the device(s) detected an overcurrent condition for any virtual channel in the task. Reading this property clears the overcurrent status for all channels in the task. You must read this property before you read Overcurrent Channels. Otherwise, you will receive an error.
#define DAQmx_Read_OvercurrentChans                                      0x29E7 // Indicates the names of any virtual channels in the task for which the device(s) detected an overcurrent condition.. You must read Overcurrent Channels Exist before you read this property. Otherwise, you will receive an error. On some devices, you must restart the task for all overcurrent channels to recover.
#define DAQmx_Read_OpenCurrentLoopChansExist                             0x2A09 // Indicates if the device(s) detected an open current loop for any virtual channel in the task. Reading this property clears the open current loop status for all channels in the task. You must read this property before you read Open Current Loop Channels. Otherwise, you will receive an error.
#define DAQmx_Read_OpenCurrentLoopChans                                  0x2A0A // Indicates the names of any virtual channels in the task for which the device(s) detected an open current loop. You must read Open Current Loop Channels Exist before you read this property. Otherwise, you will receive an error.
#define DAQmx_Read_OpenThrmcplChansExist                                 0x2A96 // Indicates if the device(s) detected an open thermocouple connected to any virtual channel in the task. Reading this property clears the open thermocouple status for all channels in the task. You must read this property before you read Open Thermocouple Channels. Otherwise, you will receive an error.
#define DAQmx_Read_OpenThrmcplChans                                      0x2A97 // Indicates the names of any virtual channels in the task for which the device(s) detected an open thermcouple. You must read Open Thermocouple Channels Exist before you read this property. Otherwise, you will receive an error.
#define DAQmx_Read_OverloadedChansExist                                  0x2174 // Indicates if the device(s) detected an overload in any virtual channel in the task. Reading this property clears the overload status for all channels in the task. You must read this property before you read Overloaded Channels. Otherwise, you will receive an error.
#define DAQmx_Read_OverloadedChans                                       0x2175 // Indicates the names of any overloaded virtual channels in the task. You must read Overloaded Channels Exist before you read this property. Otherwise, you will receive an error.
#define DAQmx_Read_ChangeDetect_HasOverflowed                            0x2194 // Indicates if samples were missed because change detection events occurred faster than the device could handle them. Some devices detect overflows differently than others.
#define DAQmx_Read_RawDataWidth                                          0x217A // Indicates in bytes the size of a raw sample from the task.
#define DAQmx_Read_NumChans                                              0x217B // Indicates the number of channels that an NI-DAQmx Read function reads from the task. This value is the number of channels in the task or the number of channels you specify with Channels to Read.
#define DAQmx_Read_DigitalLines_BytesPerChan                             0x217C // Indicates the number of bytes per channel that NI-DAQmx returns in a sample for line-based reads. If a channel has fewer lines than this number, the extra bytes are FALSE.
#define DAQmx_Read_WaitMode                                              0x2232 // Specifies how an NI-DAQmx Read function waits for samples to become available.
#define DAQmx_Read_SleepTime                                             0x22B0 // Specifies in seconds the amount of time to sleep after checking for available samples if Wait Mode is DAQmx_Val_Sleep.

//********** Real-Time Attributes **********
#define DAQmx_RealTime_ConvLateErrorsToWarnings                          0x22EE // Specifies if DAQmxWaitForNextSampleClock() and an NI-DAQmx Read function convert late errors to warnings. NI-DAQmx returns no late warnings or errors until the number of warmup iterations you specify with Number Of Warmup Iterations execute.
#define DAQmx_RealTime_NumOfWarmupIters                                  0x22ED // Specifies the number of loop iterations that must occur before DAQmxWaitForNextSampleClock() and an NI-DAQmx Read function return any late warnings or errors. The system needs a number of iterations to stabilize. During this period, a large amount of jitter occurs, potentially causing reads and writes to be late. The default number of warmup iterations is 100. Specify a larger number if needed to stabilize the sys...
#define DAQmx_RealTime_WaitForNextSampClkWaitMode                        0x22EF // Specifies how DAQmxWaitForNextSampleClock() waits for the next Sample Clock pulse.
#define DAQmx_RealTime_ReportMissedSamp                                  0x2319 // Specifies whether an NI-DAQmx Read function returns lateness errors or warnings when it detects missed Sample Clock pulses. This setting does not affect DAQmxWaitForNextSampleClock(). Set this property to TRUE for applications that need to detect lateness without using DAQmxWaitForNextSampleClock().
#define DAQmx_RealTime_WriteRecoveryMode                                 0x231A // Specifies how NI-DAQmx attempts to recover after missing a Sample Clock pulse when performing counter writes.

//********** Switch Channel Attributes **********
#define DAQmx_SwitchChan_Usage                                           0x18E4 // Specifies how you can use the channel. Using this property acts as a safety mechanism to prevent you from connecting two source channels, for example.
#define DAQmx_SwitchChan_MaxACCarryCurrent                               0x0648 // Indicates in amperes the maximum AC current that the device can carry.
#define DAQmx_SwitchChan_MaxACSwitchCurrent                              0x0646 // Indicates in amperes the maximum AC current that the device can switch. This current is always against an RMS voltage level.
#define DAQmx_SwitchChan_MaxACCarryPwr                                   0x0642 // Indicates in watts the maximum AC power that the device can carry.
#define DAQmx_SwitchChan_MaxACSwitchPwr                                  0x0644 // Indicates in watts the maximum AC power that the device can switch.
#define DAQmx_SwitchChan_MaxDCCarryCurrent                               0x0647 // Indicates in amperes the maximum DC current that the device can carry.
#define DAQmx_SwitchChan_MaxDCSwitchCurrent                              0x0645 // Indicates in amperes the maximum DC current that the device can switch. This current is always against a DC voltage level.
#define DAQmx_SwitchChan_MaxDCCarryPwr                                   0x0643 // Indicates in watts the maximum DC power that the device can carry.
#define DAQmx_SwitchChan_MaxDCSwitchPwr                                  0x0649 // Indicates in watts the maximum DC power that the device can switch.
#define DAQmx_SwitchChan_MaxACVoltage                                    0x0651 // Indicates in volts the maximum AC RMS voltage that the device can switch.
#define DAQmx_SwitchChan_MaxDCVoltage                                    0x0650 // Indicates in volts the maximum DC voltage that the device can switch.
#define DAQmx_SwitchChan_WireMode                                        0x18E5 // Indicates the number of wires that the channel switches.
#define DAQmx_SwitchChan_Bandwidth                                       0x0640 // Indicates in Hertz the maximum frequency of a signal that can pass through the switch without significant deterioration.
#define DAQmx_SwitchChan_Impedance                                       0x0641 // Indicates in ohms the switch impedance. This value is important in the RF domain and should match the impedance of the sources and loads.

//********** Switch Device Attributes **********
#define DAQmx_SwitchDev_SettlingTime                                     0x1244 // Specifies in seconds the amount of time to wait for the switch to settle (or debounce). NI-DAQmx adds this time to the settling time of the motherboard. Modify this property only if the switch does not settle within the settling time of the motherboard. Refer to device documentation for supported settling times.
#define DAQmx_SwitchDev_AutoConnAnlgBus                                  0x17DA // Specifies if NI-DAQmx routes multiplexed channels to the analog bus backplane. Only the SCXI-1127 and SCXI-1128 support this property.
#define DAQmx_SwitchDev_PwrDownLatchRelaysAfterSettling                  0x22DB // Specifies if DAQmxSwitchWaitForSettling() powers down latching relays after waiting for the device to settle.
#define DAQmx_SwitchDev_Settled                                          0x1243 // Indicates when Settling Time expires.
#define DAQmx_SwitchDev_RelayList                                        0x17DC // Indicates a comma-delimited list of relay names.
#define DAQmx_SwitchDev_NumRelays                                        0x18E6 // Indicates the number of relays on the device. This value matches the number of relay names in Relay List.
#define DAQmx_SwitchDev_SwitchChanList                                   0x18E7 // Indicates a comma-delimited list of channel names for the current topology of the device.
#define DAQmx_SwitchDev_NumSwitchChans                                   0x18E8 // Indicates the number of switch channels for the current topology of the device. This value matches the number of channel names in Switch Channel List.
#define DAQmx_SwitchDev_NumRows                                          0x18E9 // Indicates the number of rows on a device in a matrix switch topology. Indicates the number of multiplexed channels on a device in a mux topology.
#define DAQmx_SwitchDev_NumColumns                                       0x18EA // Indicates the number of columns on a device in a matrix switch topology. This value is always 1 if the device is in a mux topology.
#define DAQmx_SwitchDev_Topology                                         0x193D // Indicates the current topology of the device. This value is one of the topology options in DAQmxSwitchSetTopologyAndReset().

//********** Switch Scan Attributes **********
#define DAQmx_SwitchScan_BreakMode                                       0x1247 // Specifies the action to take between each entry in a scan list.
#define DAQmx_SwitchScan_RepeatMode                                      0x1248 // Specifies if the task advances through the scan list multiple times.
#define DAQmx_SwitchScan_WaitingForAdv                                   0x17D9 // Indicates if the switch hardware is waiting for an  Advance Trigger. If the hardware is waiting, it completed the previous entry in the scan list.

//********** Scale Attributes **********
#define DAQmx_Scale_Descr                                                0x1226 // Specifies a description for the scale.
#define DAQmx_Scale_ScaledUnits                                          0x191B // Specifies the units to use for scaled values. You can use an arbitrary string.
#define DAQmx_Scale_PreScaledUnits                                       0x18F7 // Specifies the units of the values that you want to scale.
#define DAQmx_Scale_Type                                                 0x1929 // Indicates the method or equation form that the custom scale uses.
#define DAQmx_Scale_Lin_Slope                                            0x1227 // Specifies the slope, m, in the equation y=mx+b.
#define DAQmx_Scale_Lin_YIntercept                                       0x1228 // Specifies the y-intercept, b, in the equation y=mx+b.
#define DAQmx_Scale_Map_ScaledMax                                        0x1229 // Specifies the largest value in the range of scaled values. NI-DAQmx maps this value to Pre-Scaled Maximum Value. Reads coerce samples that are larger than this value to match this value. Writes generate errors for samples that are larger than this value.
#define DAQmx_Scale_Map_PreScaledMax                                     0x1231 // Specifies the largest value in the range of pre-scaled values. NI-DAQmx maps this value to Scaled Maximum Value.
#define DAQmx_Scale_Map_ScaledMin                                        0x1230 // Specifies the smallest value in the range of scaled values. NI-DAQmx maps this value to Pre-Scaled Minimum Value. Reads coerce samples that are smaller than this value to match this value. Writes generate errors for samples that are smaller than this value.
#define DAQmx_Scale_Map_PreScaledMin                                     0x1232 // Specifies the smallest value in the range of pre-scaled values. NI-DAQmx maps this value to Scaled Minimum Value.
#define DAQmx_Scale_Poly_ForwardCoeff                                    0x1234 // Specifies an array of coefficients for the polynomial that converts pre-scaled values to scaled values. Each element of the array corresponds to a term of the equation. For example, if index three of the array is 9, the fourth term of the equation is 9x^3.
#define DAQmx_Scale_Poly_ReverseCoeff                                    0x1235 // Specifies an array of coefficients for the polynomial that converts scaled values to pre-scaled values. Each element of the array corresponds to a term of the equation. For example, if index three of the array is 9, the fourth term of the equation is 9y^3.
#define DAQmx_Scale_Table_ScaledVals                                     0x1236 // Specifies an array of scaled values. These values map directly to the values in Pre-Scaled Values.
#define DAQmx_Scale_Table_PreScaledVals                                  0x1237 // Specifies an array of pre-scaled values. These values map directly to the values in Scaled Values.

//********** System Attributes **********
#define DAQmx_Sys_GlobalChans                                            0x1265 // Indicates an array that contains the names of all global channels saved on the system.
#define DAQmx_Sys_Scales                                                 0x1266 // Indicates an array that contains the names of all custom scales saved on the system.
#define DAQmx_Sys_Tasks                                                  0x1267 // Indicates an array that contains the names of all tasks saved on the system.
#define DAQmx_Sys_DevNames                                               0x193B // Indicates the names of all devices installed in the system.
#define DAQmx_Sys_NIDAQMajorVersion                                      0x1272 // Indicates the major portion of the installed version of NI-DAQ, such as 7 for version 7.0.
#define DAQmx_Sys_NIDAQMinorVersion                                      0x1923 // Indicates the minor portion of the installed version of NI-DAQ, such as 0 for version 7.0.

//********** Task Attributes **********
#define DAQmx_Task_Name                                                  0x1276 // Indicates the name of the task.
#define DAQmx_Task_Channels                                              0x1273 // Indicates the names of all virtual channels in the task.
#define DAQmx_Task_NumChans                                              0x2181 // Indicates the number of virtual channels in the task.
#define DAQmx_Task_Devices                                               0x230E // Indicates an array containing the names of all devices in the task.
#define DAQmx_Task_NumDevices                                            0x29BA // Indicates the number of devices in the task.
#define DAQmx_Task_Complete                                              0x1274 // Indicates whether the task completed execution.

//********** Timing Attributes **********
#define DAQmx_SampQuant_SampMode                                         0x1300 // Specifies if a task acquires or generates a finite number of samples or if it continuously acquires or generates samples.
#define DAQmx_SampQuant_SampPerChan                                      0x1310 // Specifies the number of samples to acquire or generate for each channel if Sample Mode is DAQmx_Val_FiniteSamps. If Sample Mode is DAQmx_Val_ContSamps, NI-DAQmx uses this value to determine the buffer size.
#define DAQmx_SampTimingType                                             0x1347 // Specifies the type of sample timing to use for the task.
#define DAQmx_SampClk_Rate                                               0x1344 // Specifies the sampling rate in samples per channel per second. If you use an external source for the Sample Clock, set this input to the maximum expected rate of that clock.
#define DAQmx_SampClk_MaxRate                                            0x22C8 // Indicates the maximum Sample Clock rate supported by the task, based on other timing settings. For output tasks, the maximum Sample Clock rate is the maximum rate of the DAC. For input tasks, NI-DAQmx calculates the maximum sampling rate differently for multiplexed devices than simultaneous sampling devices.
#define DAQmx_SampClk_Src                                                0x1852 // Specifies the terminal of the signal to use as the Sample Clock.
#define DAQmx_SampClk_ActiveEdge                                         0x1301 // Specifies on which edge of a clock pulse sampling takes place. This property is useful primarily when the signal you use as the Sample Clock is not a periodic clock.
#define DAQmx_SampClk_UnderflowBehavior                                  0x2961 // Specifies the action to take when the onboard memory of the device becomes empty.
#define DAQmx_SampClk_TimebaseDiv                                        0x18EB // Specifies the number of Sample Clock Timebase pulses needed to produce a single Sample Clock pulse.
#define DAQmx_SampClk_Timebase_Rate                                      0x1303 // Specifies the rate of the Sample Clock Timebase. Some applications require that you specify a rate when you use any signal other than the onboard Sample Clock Timebase. NI-DAQmx requires this rate to calculate other timing parameters.
#define DAQmx_SampClk_Timebase_Src                                       0x1308 // Specifies the terminal of the signal to use as the Sample Clock Timebase.
#define DAQmx_SampClk_Timebase_ActiveEdge                                0x18EC // Specifies on which edge to recognize a Sample Clock Timebase pulse. This property is useful primarily when the signal you use as the Sample Clock Timebase is not a periodic clock.
#define DAQmx_SampClk_Timebase_MasterTimebaseDiv                         0x1305 // Specifies the number of pulses of the Master Timebase needed to produce a single pulse of the Sample Clock Timebase.
#define DAQmx_SampClk_DigFltr_Enable                                     0x221E // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_SampClk_DigFltr_MinPulseWidth                              0x221F // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_SampClk_DigFltr_TimebaseSrc                                0x2220 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_SampClk_DigFltr_TimebaseRate                               0x2221 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_SampClk_DigSync_Enable                                     0x2222 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_Hshk_DelayAfterXfer                                        0x22C2 // Specifies the number of seconds to wait after a handshake cycle before starting a new handshake cycle.
#define DAQmx_Hshk_StartCond                                             0x22C3 // Specifies the point in the handshake cycle that the device is in when the task starts.
#define DAQmx_Hshk_SampleInputDataWhen                                   0x22C4 // Specifies on which edge of the Handshake Trigger an input task latches the data from the peripheral device.
#define DAQmx_ChangeDetect_DI_RisingEdgePhysicalChans                    0x2195 // Specifies the names of the digital lines or ports on which to detect rising edges. The lines or ports must be used by virtual channels in the task. You also can specify a string that contains a list or range of digital lines or ports.
#define DAQmx_ChangeDetect_DI_FallingEdgePhysicalChans                   0x2196 // Specifies the names of the digital lines or ports on which to detect falling edges. The lines or ports must be used by virtual channels in the task. You also can specify a string that contains a list or range of digital lines or ports.
#define DAQmx_OnDemand_SimultaneousAOEnable                              0x21A0 // Specifies whether to update all channels in the task simultaneously, rather than updating channels independently when you write a sample to that channel.
#define DAQmx_AIConv_Rate                                                0x1848 // Specifies in Hertz the rate at which to clock the analog-to-digital converter. This clock is specific to the analog input section of multiplexed devices.
#define DAQmx_AIConv_MaxRate                                             0x22C9 // Indicates the maximum convert rate supported by the task, given the current devices and channel count.
#define DAQmx_AIConv_Src                                                 0x1502 // Specifies the terminal of the signal to use as the AI Convert Clock.
#define DAQmx_AIConv_ActiveEdge                                          0x1853 // Specifies on which edge of the clock pulse an analog-to-digital conversion takes place.
#define DAQmx_AIConv_TimebaseDiv                                         0x1335 // Specifies the number of AI Convert Clock Timebase pulses needed to produce a single AI Convert Clock pulse.
#define DAQmx_AIConv_Timebase_Src                                        0x1339 // Specifies the terminal  of the signal to use as the AI Convert Clock Timebase.
#define DAQmx_DelayFromSampClk_DelayUnits                                0x1304 // Specifies the units of Delay.
#define DAQmx_DelayFromSampClk_Delay                                     0x1317 // Specifies the amount of time to wait after receiving a Sample Clock edge before beginning to acquire the sample. This value is in the units you specify with Delay Units.
#define DAQmx_MasterTimebase_Rate                                        0x1495 // Specifies the rate of the Master Timebase.
#define DAQmx_MasterTimebase_Src                                         0x1343 // Specifies the terminal of the signal to use as the Master Timebase. On an E Series device, you can choose only between the onboard 20MHz Timebase or the RTSI7 terminal.
#define DAQmx_RefClk_Rate                                                0x1315 // Specifies the frequency of the Reference Clock.
#define DAQmx_RefClk_Src                                                 0x1316 // Specifies the terminal of the signal to use as the Reference Clock.
#define DAQmx_SyncPulse_Src                                              0x223D // Specifies the terminal of the signal to use as the synchronization pulse. The synchronization pulse resets the clock dividers and the ADCs/DACs on the device.
#define DAQmx_SyncPulse_SyncTime                                         0x223E // Indicates in seconds the delay required to reset the ADCs/DACs after the device receives the synchronization pulse.
#define DAQmx_SyncPulse_MinDelayToStart                                  0x223F // Specifies in seconds the amount of time that elapses after the master device issues the synchronization pulse before the task starts.
#define DAQmx_SampTimingEngine                                           0x2A26 // Specifies which timing engine to use for the specified timing type. Refer to device documentation for information on supported timing engines.

//********** Trigger Attributes **********
#define DAQmx_StartTrig_Type                                             0x1393 // Specifies the type of trigger to use to start a task.
#define DAQmx_DigEdge_StartTrig_Src                                      0x1407 // Specifies the name of a terminal where there is a digital signal to use as the source of the Start Trigger.
#define DAQmx_DigEdge_StartTrig_Edge                                     0x1404 // Specifies on which edge of a digital pulse to start acquiring or generating samples.
#define DAQmx_DigEdge_StartTrig_DigFltr_Enable                           0x2223 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_DigEdge_StartTrig_DigFltr_MinPulseWidth                    0x2224 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_DigEdge_StartTrig_DigFltr_TimebaseSrc                      0x2225 // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_DigEdge_StartTrig_DigFltr_TimebaseRate                     0x2226 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_DigEdge_StartTrig_DigSync_Enable                           0x2227 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_DigPattern_StartTrig_Src                                   0x1410 // Specifies the physical channels to use for pattern matching. The order of the physical channels determines the order of the pattern. If a port is included, the order of the physical channels within the port is in ascending order.
#define DAQmx_DigPattern_StartTrig_Pattern                               0x2186 // Specifies the digital pattern that must be met for the Start Trigger to occur.
#define DAQmx_DigPattern_StartTrig_When                                  0x1411 // Specifies whether the Start Trigger occurs when the physical channels specified with Source match or differ from the digital pattern specified with Pattern.
#define DAQmx_AnlgEdge_StartTrig_Src                                     0x1398 // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Start Trigger.
#define DAQmx_AnlgEdge_StartTrig_Slope                                   0x1397 // Specifies on which slope of the trigger signal to start acquiring or generating samples.
#define DAQmx_AnlgEdge_StartTrig_Lvl                                     0x1396 // Specifies at what threshold in the units of the measurement or generation to start acquiring or generating samples. Use Slope to specify on which slope to trigger on this threshold.
#define DAQmx_AnlgEdge_StartTrig_Hyst                                    0x1395 // Specifies a hysteresis level in the units of the measurement or generation. If Slope is DAQmx_Val_RisingSlope, the trigger does not deassert until the source signal passes below  Level minus the hysteresis. If Slope is DAQmx_Val_FallingSlope, the trigger does not deassert until the source signal passes above Level plus the hysteresis.
#define DAQmx_AnlgEdge_StartTrig_Coupling                                0x2233 // Specifies the coupling for the source signal of the trigger if the source is a terminal rather than a virtual channel.
#define DAQmx_AnlgWin_StartTrig_Src                                      0x1400 // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Start Trigger.
#define DAQmx_AnlgWin_StartTrig_When                                     0x1401 // Specifies whether the task starts acquiring or generating samples when the signal enters or leaves the window you specify with Bottom and Top.
#define DAQmx_AnlgWin_StartTrig_Top                                      0x1403 // Specifies the upper limit of the window. Specify this value in the units of the measurement or generation.
#define DAQmx_AnlgWin_StartTrig_Btm                                      0x1402 // Specifies the lower limit of the window. Specify this value in the units of the measurement or generation.
#define DAQmx_AnlgWin_StartTrig_Coupling                                 0x2234 // Specifies the coupling for the source signal of the trigger if the source is a terminal rather than a virtual channel.
#define DAQmx_StartTrig_Delay                                            0x1856 // Specifies an amount of time to wait after the Start Trigger is received before acquiring or generating the first sample. This value is in the units you specify with Delay Units.
#define DAQmx_StartTrig_DelayUnits                                       0x18C8 // Specifies the units of Delay.
#define DAQmx_StartTrig_Retriggerable                                    0x190F // Specifies whether to enable retriggerable counter pulse generation. When you set this property to TRUE, the device generates pulses each time it receives a trigger. The device ignores a trigger if it is in the process of generating pulses.
#define DAQmx_RefTrig_Type                                               0x1419 // Specifies the type of trigger to use to mark a reference point for the measurement.
#define DAQmx_RefTrig_PretrigSamples                                     0x1445 // Specifies the minimum number of pretrigger samples to acquire from each channel before recognizing the reference trigger. Post-trigger samples per channel are equal to Samples Per Channel minus the number of pretrigger samples per channel.
#define DAQmx_DigEdge_RefTrig_Src                                        0x1434 // Specifies the name of a terminal where there is a digital signal to use as the source of the Reference Trigger.
#define DAQmx_DigEdge_RefTrig_Edge                                       0x1430 // Specifies on what edge of a digital pulse the Reference Trigger occurs.
#define DAQmx_DigPattern_RefTrig_Src                                     0x1437 // Specifies the physical channels to use for pattern matching. The order of the physical channels determines the order of the pattern. If a port is included, the order of the physical channels within the port is in ascending order.
#define DAQmx_DigPattern_RefTrig_Pattern                                 0x2187 // Specifies the digital pattern that must be met for the Reference Trigger to occur.
#define DAQmx_DigPattern_RefTrig_When                                    0x1438 // Specifies whether the Reference Trigger occurs when the physical channels specified with Source match or differ from the digital pattern specified with Pattern.
#define DAQmx_AnlgEdge_RefTrig_Src                                       0x1424 // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Reference Trigger.
#define DAQmx_AnlgEdge_RefTrig_Slope                                     0x1423 // Specifies on which slope of the source signal the Reference Trigger occurs.
#define DAQmx_AnlgEdge_RefTrig_Lvl                                       0x1422 // Specifies in the units of the measurement the threshold at which the Reference Trigger occurs.  Use Slope to specify on which slope to trigger at this threshold.
#define DAQmx_AnlgEdge_RefTrig_Hyst                                      0x1421 // Specifies a hysteresis level in the units of the measurement. If Slope is DAQmx_Val_RisingSlope, the trigger does not deassert until the source signal passes below Level minus the hysteresis. If Slope is DAQmx_Val_FallingSlope, the trigger does not deassert until the source signal passes above Level plus the hysteresis.
#define DAQmx_AnlgEdge_RefTrig_Coupling                                  0x2235 // Specifies the coupling for the source signal of the trigger if the source is a terminal rather than a virtual channel.
#define DAQmx_AnlgWin_RefTrig_Src                                        0x1426 // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Reference Trigger.
#define DAQmx_AnlgWin_RefTrig_When                                       0x1427 // Specifies whether the Reference Trigger occurs when the source signal enters the window or when it leaves the window. Use Bottom and Top to specify the window.
#define DAQmx_AnlgWin_RefTrig_Top                                        0x1429 // Specifies the upper limit of the window. Specify this value in the units of the measurement.
#define DAQmx_AnlgWin_RefTrig_Btm                                        0x1428 // Specifies the lower limit of the window. Specify this value in the units of the measurement.
#define DAQmx_AnlgWin_RefTrig_Coupling                                   0x1857 // Specifies the coupling for the source signal of the trigger if the source is a terminal rather than a virtual channel.
#define DAQmx_AdvTrig_Type                                               0x1365 // Specifies the type of trigger to use to advance to the next entry in a switch scan list.
#define DAQmx_DigEdge_AdvTrig_Src                                        0x1362 // Specifies the name of a terminal where there is a digital signal to use as the source of the Advance Trigger.
#define DAQmx_DigEdge_AdvTrig_Edge                                       0x1360 // Specifies on which edge of a digital signal to advance to the next entry in a scan list.
#define DAQmx_DigEdge_AdvTrig_DigFltr_Enable                             0x2238 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_HshkTrig_Type                                              0x22B7 // Specifies the type of Handshake Trigger to use.
#define DAQmx_Interlocked_HshkTrig_Src                                   0x22B8 // Specifies the source terminal of the Handshake Trigger.
#define DAQmx_Interlocked_HshkTrig_AssertedLvl                           0x22B9 // Specifies the asserted level of the Handshake Trigger.
#define DAQmx_PauseTrig_Type                                             0x1366 // Specifies the type of trigger to use to pause a task.
#define DAQmx_AnlgLvl_PauseTrig_Src                                      0x1370 // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the trigger.
#define DAQmx_AnlgLvl_PauseTrig_When                                     0x1371 // Specifies whether the task pauses above or below the threshold you specify with Level.
#define DAQmx_AnlgLvl_PauseTrig_Lvl                                      0x1369 // Specifies the threshold at which to pause the task. Specify this value in the units of the measurement or generation. Use Pause When to specify whether the task pauses above or below this threshold.
#define DAQmx_AnlgLvl_PauseTrig_Hyst                                     0x1368 // Specifies a hysteresis level in the units of the measurement or generation. If Pause When is DAQmx_Val_AboveLvl, the trigger does not deassert until the source signal passes below Level minus the hysteresis. If Pause When is DAQmx_Val_BelowLvl, the trigger does not deassert until the source signal passes above Level plus the hysteresis.
#define DAQmx_AnlgLvl_PauseTrig_Coupling                                 0x2236 // Specifies the coupling for the source signal of the trigger if the source is a terminal rather than a virtual channel.
#define DAQmx_AnlgWin_PauseTrig_Src                                      0x1373 // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the trigger.
#define DAQmx_AnlgWin_PauseTrig_When                                     0x1374 // Specifies whether the task pauses while the trigger signal is inside or outside the window you specify with Bottom and Top.
#define DAQmx_AnlgWin_PauseTrig_Top                                      0x1376 // Specifies the upper limit of the window. Specify this value in the units of the measurement or generation.
#define DAQmx_AnlgWin_PauseTrig_Btm                                      0x1375 // Specifies the lower limit of the window. Specify this value in the units of the measurement or generation.
#define DAQmx_AnlgWin_PauseTrig_Coupling                                 0x2237 // Specifies the coupling for the source signal of the trigger if the source is a terminal rather than a virtual channel.
#define DAQmx_DigLvl_PauseTrig_Src                                       0x1379 // Specifies the name of a terminal where there is a digital signal to use as the source of the Pause Trigger.
#define DAQmx_DigLvl_PauseTrig_When                                      0x1380 // Specifies whether the task pauses while the signal is high or low.
#define DAQmx_DigLvl_PauseTrig_DigFltr_Enable                            0x2228 // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_DigLvl_PauseTrig_DigFltr_MinPulseWidth                     0x2229 // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_DigLvl_PauseTrig_DigFltr_TimebaseSrc                       0x222A // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_DigLvl_PauseTrig_DigFltr_TimebaseRate                      0x222B // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_DigLvl_PauseTrig_DigSync_Enable                            0x222C // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.
#define DAQmx_DigPattern_PauseTrig_Src                                   0x216F // Specifies the physical channels to use for pattern matching. The order of the physical channels determines the order of the pattern. If a port is included, the lines within the port are in ascending order.
#define DAQmx_DigPattern_PauseTrig_Pattern                               0x2188 // Specifies the digital pattern that must be met for the Pause Trigger to occur.
#define DAQmx_DigPattern_PauseTrig_When                                  0x2170 // Specifies if the Pause Trigger occurs when the physical channels specified with Source match or differ from the digital pattern specified with Pattern.
#define DAQmx_ArmStartTrig_Type                                          0x1414 // Specifies the type of trigger to use to arm the task for a Start Trigger. If you configure an Arm Start Trigger, the task does not respond to a Start Trigger until the device receives the Arm Start Trigger.
#define DAQmx_DigEdge_ArmStartTrig_Src                                   0x1417 // Specifies the name of a terminal where there is a digital signal to use as the source of the Arm Start Trigger.
#define DAQmx_DigEdge_ArmStartTrig_Edge                                  0x1415 // Specifies on which edge of a digital signal to arm the task for a Start Trigger.
#define DAQmx_DigEdge_ArmStartTrig_DigFltr_Enable                        0x222D // Specifies whether to apply the pulse width filter to the signal.
#define DAQmx_DigEdge_ArmStartTrig_DigFltr_MinPulseWidth                 0x222E // Specifies in seconds the minimum pulse width the filter recognizes.
#define DAQmx_DigEdge_ArmStartTrig_DigFltr_TimebaseSrc                   0x222F // Specifies the input terminal of the signal to use as the timebase of the pulse width filter.
#define DAQmx_DigEdge_ArmStartTrig_DigFltr_TimebaseRate                  0x2230 // Specifies in hertz the rate of the pulse width filter timebase. NI-DAQmx uses this value to compute settings for the filter.
#define DAQmx_DigEdge_ArmStartTrig_DigSync_Enable                        0x2231 // Specifies whether to synchronize recognition of transitions in the signal to the internal timebase of the device.

//********** Watchdog Attributes **********
#define DAQmx_Watchdog_Timeout                                           0x21A9 // Specifies in seconds the amount of time until the watchdog timer expires. A value of -1 means the internal timer never expires. Set this input to -1 if you use an Expiration Trigger to expire the watchdog task.
#define DAQmx_WatchdogExpirTrig_Type                                     0x21A3 // Specifies the type of trigger to use to expire a watchdog task.
#define DAQmx_DigEdge_WatchdogExpirTrig_Src                              0x21A4 // Specifies the name of a terminal where a digital signal exists to use as the source of the Expiration Trigger.
#define DAQmx_DigEdge_WatchdogExpirTrig_Edge                             0x21A5 // Specifies on which edge of a digital signal to expire the watchdog task.
#define DAQmx_Watchdog_DO_ExpirState                                     0x21A7 // Specifies the state to which to set the digital physical channels when the watchdog task expires.  You cannot modify the expiration state of dedicated digital input physical channels.
#define DAQmx_Watchdog_HasExpired                                        0x21A8 // Indicates if the watchdog timer expired. You can read this property only while the task is running.

//********** Write Attributes **********
#define DAQmx_Write_RelativeTo                                           0x190C // Specifies the point in the buffer at which to write data. If you also specify an offset with Offset, the write operation begins at that offset relative to this point you select with this property.
#define DAQmx_Write_Offset                                               0x190D // Specifies in samples per channel an offset at which a write operation begins. This offset is relative to the location you specify with Relative To.
#define DAQmx_Write_RegenMode                                            0x1453 // Specifies whether to allow NI-DAQmx to generate the same data multiple times.
#define DAQmx_Write_CurrWritePos                                         0x1458 // Indicates the position in the buffer of the next sample to generate. This value is identical for all channels in the task.
#define DAQmx_Write_OvercurrentChansExist                                0x29E8 // Indicates if the device(s) detected an overcurrent condition for any channel in the task. Reading this property clears the overcurrent status for all channels in the task. You must read this property before you read Overcurrent Channels. Otherwise, you will receive an error.
#define DAQmx_Write_OvercurrentChans                                     0x29E9 // Indicates the names of any virtual channels in the task for which an overcurrent condition has been detected. You must read Overcurrent Channels Exist before you read this property. Otherwise, you will receive an error.
#define DAQmx_Write_OvertemperatureChansExist                            0x2A84 // Indicates if the device(s) detected a temperature above their safe operating level. If a device exceeds this temperature, the device shuts off its output channels until the temperature returns to a safe level.
#define DAQmx_Write_OpenCurrentLoopChansExist                            0x29EA // Indicates if the device(s) detected an open current loop for any channel in the task. Reading this property clears the open current loop status for all channels in the task. You must read this property before you read Open Current Loop Channels. Otherwise, you will receive an error.
#define DAQmx_Write_OpenCurrentLoopChans                                 0x29EB // Indicates the names of any virtual channels in the task for which the device(s) detected an open current loop. You must read Open Current Loop Channels Exist before you read this property. Otherwise, you will receive an error.
#define DAQmx_Write_PowerSupplyFaultChansExist                           0x29EC // Indicates if the device(s) detected a power supply fault for any channel in the task. Reading this property clears the power supply fault status for all channels in the task. You must read this property before you read Power Supply Fault Channels. Otherwise, you will receive an error.
#define DAQmx_Write_PowerSupplyFaultChans                                0x29ED // Indicates the names of any virtual channels in the task that have a power supply fault. You must read Power Supply Fault Channels Exist before you read this property. Otherwise, you will receive an error.
#define DAQmx_Write_SpaceAvail                                           0x1460 // Indicates in samples per channel the amount of available space in the buffer.
#define DAQmx_Write_TotalSampPerChanGenerated                            0x192B // Indicates the total number of samples generated by each channel in the task. This value is identical for all channels in the task.
#define DAQmx_Write_RawDataWidth                                         0x217D // Indicates in bytes the required size of a raw sample to write to the task.
#define DAQmx_Write_NumChans                                             0x217E // Indicates the number of channels that an NI-DAQmx Write function writes to the task. This value is the number of channels in the task.
#define DAQmx_Write_WaitMode                                             0x22B1 // Specifies how an NI-DAQmx Write function waits for space to become available in the buffer.
#define DAQmx_Write_SleepTime                                            0x22B2 // Specifies in seconds the amount of time to sleep after checking for available buffer space if Wait Mode is DAQmx_Val_Sleep.
#define DAQmx_Write_NextWriteIsLast                                      0x296C // Specifies that the next samples written are the last samples you want to generate. Use this property when performing continuous generation to prevent underflow errors after writing the last sample. Regeneration Mode must be DAQmx_Val_DoNotAllowRegen to use this property.
#define DAQmx_Write_DigitalLines_BytesPerChan                            0x217F // Indicates the number of bytes expected per channel in a sample for line-based writes. If a channel has fewer lines than this number, NI-DAQmx ignores the extra bytes.

//********** Physical Channel Attributes **********
#define DAQmx_PhysicalChan_AI_TermCfgs                                   0x2342 // Indicates the list of terminal configurations supported by the channel.
#define DAQmx_PhysicalChan_AO_TermCfgs                                   0x29A3 // Indicates the list of terminal configurations supported by the channel.
#define DAQmx_PhysicalChan_AO_ManualControlEnable                        0x2A1E // Specifies if you can control the physical channel externally via a manual control located on the device. You cannot simultaneously control a channel manually and with NI-DAQmx.
#define DAQmx_PhysicalChan_AO_ManualControlAmplitude                     0x2A1F // Indicates the current value of the front panel amplitude control for the physical channel in volts.
#define DAQmx_PhysicalChan_AO_ManualControlFreq                          0x2A20 // Indicates the current value of the front panel frequency control for the physical channel in hertz.
#define DAQmx_PhysicalChan_DI_PortWidth                                  0x29A4 // Indicates in bits the width of digital input port.
#define DAQmx_PhysicalChan_DI_SampClkSupported                           0x29A5 // Indicates if the sample clock timing type is supported for the digital input physical channel.
#define DAQmx_PhysicalChan_DI_ChangeDetectSupported                      0x29A6 // Indicates if the change detection timing type is supported for the digital input physical channel.
#define DAQmx_PhysicalChan_DO_PortWidth                                  0x29A7 // Indicates in bits the width of digital output port.
#define DAQmx_PhysicalChan_DO_SampClkSupported                           0x29A8 // Indicates if the sample clock timing type is supported for the digital output physical channel.
#define DAQmx_PhysicalChan_TEDS_MfgID                                    0x21DA // Indicates the manufacturer ID of the sensor.
#define DAQmx_PhysicalChan_TEDS_ModelNum                                 0x21DB // Indicates the model number of the sensor.
#define DAQmx_PhysicalChan_TEDS_SerialNum                                0x21DC // Indicates the serial number of the sensor.
#define DAQmx_PhysicalChan_TEDS_VersionNum                               0x21DD // Indicates the version number of the sensor.
#define DAQmx_PhysicalChan_TEDS_VersionLetter                            0x21DE // Indicates the version letter of the sensor.
#define DAQmx_PhysicalChan_TEDS_BitStream                                0x21DF // Indicates the TEDS binary bitstream without checksums.
#define DAQmx_PhysicalChan_TEDS_TemplateIDs                              0x228F // Indicates the IDs of the templates in the bitstream in BitStream.

//********** Persisted Task Attributes **********
#define DAQmx_PersistedTask_Author                                       0x22CC // Indicates the author of the task.
#define DAQmx_PersistedTask_AllowInteractiveEditing                      0x22CD // Indicates whether the task can be edited in the DAQ Assistant.
#define DAQmx_PersistedTask_AllowInteractiveDeletion                     0x22CE // Indicates whether the task can be deleted through MAX.

//********** Persisted Channel Attributes **********
#define DAQmx_PersistedChan_Author                                       0x22D0 // Indicates the author of the global channel.
#define DAQmx_PersistedChan_AllowInteractiveEditing                      0x22D1 // Indicates whether the global channel can be edited in the DAQ Assistant.
#define DAQmx_PersistedChan_AllowInteractiveDeletion                     0x22D2 // Indicates whether the global channel can be deleted through MAX.

//********** Persisted Scale Attributes **********
#define DAQmx_PersistedScale_Author                                      0x22D4 // Indicates the author of the custom scale.
#define DAQmx_PersistedScale_AllowInteractiveEditing                     0x22D5 // Indicates whether the custom scale can be edited in the DAQ Assistant.
#define DAQmx_PersistedScale_AllowInteractiveDeletion                    0x22D6 // Indicates whether the custom scale can be deleted through MAX.


// For backwards compatibility, the DAQmx_ReadWaitMode has to be defined because this was the original spelling
// that has been later on corrected.
#define DAQmx_ReadWaitMode	DAQmx_Read_WaitMode

/******************************************************************************
 *** NI-DAQmx Values **********************************************************
 ******************************************************************************/

/******************************************************/
/***    Non-Attribute Function Parameter Values     ***/
/******************************************************/

//*** Values for the Mode parameter of DAQmxTaskControl ***
#define DAQmx_Val_Task_Start                                              0   // Start
#define DAQmx_Val_Task_Stop                                               1   // Stop
#define DAQmx_Val_Task_Verify                                             2   // Verify
#define DAQmx_Val_Task_Commit                                             3   // Commit
#define DAQmx_Val_Task_Reserve                                            4   // Reserve
#define DAQmx_Val_Task_Unreserve                                          5   // Unreserve
#define DAQmx_Val_Task_Abort                                              6   // Abort

//*** Values for the Options parameter of the event registration functions
#define DAQmx_Val_SynchronousEventCallbacks                               (1<<0)     // Synchronous callbacks

//*** Values for the everyNsamplesEventType parameter of DAQmxRegisterEveryNSamplesEvent ***
#define DAQmx_Val_Acquired_Into_Buffer                                    1     // Acquired Into Buffer
#define DAQmx_Val_Transferred_From_Buffer                                 2     // Transferred From Buffer


//*** Values for the Action parameter of DAQmxControlWatchdogTask ***
#define DAQmx_Val_ResetTimer                                              0   // Reset Timer
#define DAQmx_Val_ClearExpiration                                         1   // Clear Expiration

//*** Values for the Line Grouping parameter of DAQmxCreateDIChan and DAQmxCreateDOChan ***
#define DAQmx_Val_ChanPerLine                                             0   // One Channel For Each Line
#define DAQmx_Val_ChanForAllLines                                         1   // One Channel For All Lines

//*** Values for the Fill Mode parameter of DAQmxReadAnalogF64, DAQmxReadBinaryI16, DAQmxReadBinaryU16, DAQmxReadBinaryI32, DAQmxReadBinaryU32,
//    DAQmxReadDigitalU8, DAQmxReadDigitalU32, DAQmxReadDigitalLines ***
//*** Values for the Data Layout parameter of DAQmxWriteAnalogF64, DAQmxWriteBinaryI16, DAQmxWriteDigitalU8, DAQmxWriteDigitalU32, DAQmxWriteDigitalLines ***
#define DAQmx_Val_GroupByChannel                                          0   // Group by Channel
#define DAQmx_Val_GroupByScanNumber                                       1   // Group by Scan Number

//*** Values for the Signal Modifiers parameter of DAQmxConnectTerms ***/
#define DAQmx_Val_DoNotInvertPolarity                                     0   // Do not invert polarity
#define DAQmx_Val_InvertPolarity                                          1   // Invert polarity

//*** Values for the Action paramter of DAQmxCloseExtCal ***
#define DAQmx_Val_Action_Commit                                           0   // Commit
#define DAQmx_Val_Action_Cancel                                           1   // Cancel

//*** Values for the Trigger ID parameter of DAQmxSendSoftwareTrigger ***
#define DAQmx_Val_AdvanceTrigger                                          12488 // Advance Trigger

//*** Value set for the ActiveEdge parameter of DAQmxCfgSampClkTiming and DAQmxCfgPipelinedSampClkTiming ***
#define DAQmx_Val_Rising                                                  10280 // Rising
#define DAQmx_Val_Falling                                                 10171 // Falling

//*** Value set SwitchPathType ***
//*** Value set for the output Path Status parameter of DAQmxSwitchFindPath ***
#define DAQmx_Val_PathStatus_Available                                    10431 // Path Available
#define DAQmx_Val_PathStatus_AlreadyExists                                10432 // Path Already Exists
#define DAQmx_Val_PathStatus_Unsupported                                  10433 // Path Unsupported
#define DAQmx_Val_PathStatus_ChannelInUse                                 10434 // Channel In Use
#define DAQmx_Val_PathStatus_SourceChannelConflict                        10435 // Channel Source Conflict
#define DAQmx_Val_PathStatus_ChannelReservedForRouting                    10436 // Channel Reserved for Routing

//*** Value set for the Units parameter of DAQmxCreateAIThrmcplChan, DAQmxCreateAIRTDChan, DAQmxCreateAIThrmstrChanIex, DAQmxCreateAIThrmstrChanVex and DAQmxCreateAITempBuiltInSensorChan ***
#define DAQmx_Val_DegC                                                    10143 // Deg C
#define DAQmx_Val_DegF                                                    10144 // Deg F
#define DAQmx_Val_Kelvins                                                 10325 // Kelvins
#define DAQmx_Val_DegR                                                    10145 // Deg R

//*** Value set for the state parameter of DAQmxSetDigitalPowerUpStates ***
#define DAQmx_Val_High                                                    10192 // High
#define DAQmx_Val_Low                                                     10214 // Low
#define DAQmx_Val_Tristate                                                10310 // Tristate

//*** Value set for the channelType parameter of DAQmxSetAnalogPowerUpStates ***
#define DAQmx_Val_ChannelVoltage                                          0     // Voltage Channel
#define DAQmx_Val_ChannelCurrent                                          1     // Current Channel

//*** Value set RelayPos ***
//*** Value set for the state parameter of DAQmxSwitchGetSingleRelayPos and DAQmxSwitchGetMultiRelayPos ***
#define DAQmx_Val_Open                                                    10437 // Open
#define DAQmx_Val_Closed                                                  10438 // Closed


//*** Value set for the inputCalSource parameter of DAQmxAdjust1540Cal ***
#define DAQmx_Val_Loopback0                                               0     // Loopback 0 degree shift
#define DAQmx_Val_Loopback180                                             1     // Loopback 180 degree shift
#define DAQmx_Val_Ground                                                  2     // Ground


//*** Value for the Terminal Config parameter of DAQmxCreateAIVoltageChan, DAQmxCreateAICurrentChan and DAQmxCreateAIVoltageChanWithExcit ***
#define DAQmx_Val_Cfg_Default                                             -1 // Default
//*** Value for the Shunt Resistor Location parameter of DAQmxCreateAICurrentChan ***
#define DAQmx_Val_Default                                                 -1 // Default

//*** Value for the Timeout parameter of DAQmxWaitUntilTaskDone
#define DAQmx_Val_WaitInfinitely                                          -1.0

//*** Value for the Number of Samples per Channel parameter of DAQmxReadAnalogF64, DAQmxReadBinaryI16, DAQmxReadBinaryU16,
//    DAQmxReadBinaryI32, DAQmxReadBinaryU32, DAQmxReadDigitalU8, DAQmxReadDigitalU32,
//    DAQmxReadDigitalLines, DAQmxReadCounterF64, DAQmxReadCounterU32 and DAQmxReadRaw ***
#define DAQmx_Val_Auto                                                    -1

// Value set for the Options parameter of DAQmxSaveTask, DAQmxSaveGlobalChan and DAQmxSaveScale
#define DAQmx_Val_Save_Overwrite                                          (1<<0)
#define DAQmx_Val_Save_AllowInteractiveEditing                            (1<<1)
#define DAQmx_Val_Save_AllowInteractiveDeletion                           (1<<2)

//*** Values for the Trigger Usage parameter - set of trigger types a device may support
//*** Values for TriggerUsageTypeBits
#define DAQmx_Val_Bit_TriggerUsageTypes_Advance                           (1<<0) // Device supports advance triggers
#define DAQmx_Val_Bit_TriggerUsageTypes_Pause                             (1<<1) // Device supports pause triggers
#define DAQmx_Val_Bit_TriggerUsageTypes_Reference                         (1<<2) // Device supports reference triggers
#define DAQmx_Val_Bit_TriggerUsageTypes_Start                             (1<<3) // Device supports start triggers
#define DAQmx_Val_Bit_TriggerUsageTypes_Handshake                         (1<<4) // Device supports handshake triggers
#define DAQmx_Val_Bit_TriggerUsageTypes_ArmStart                          (1<<5) // Device supports arm start triggers

//*** Values for the Coupling Types parameter - set of coupling types a device may support
//*** Values for CouplingTypeBits
#define DAQmx_Val_Bit_CouplingTypes_AC                                    (1<<0) // Device supports AC coupling
#define DAQmx_Val_Bit_CouplingTypes_DC                                    (1<<1) // Device supports DC coupling
#define DAQmx_Val_Bit_CouplingTypes_Ground                                (1<<2) // Device supports ground coupling
#define DAQmx_Val_Bit_CouplingTypes_HFReject                              (1<<3) // Device supports High Frequency Reject coupling
#define DAQmx_Val_Bit_CouplingTypes_LFReject                              (1<<4) // Device supports Low Frequency Reject coupling
#define DAQmx_Val_Bit_CouplingTypes_NoiseReject                           (1<<5) // Device supports Noise Reject coupling

//*** Values for DAQmx_PhysicalChan_AI_TermCfgs and DAQmx_PhysicalChan_AO_TermCfgs
//*** Value set TerminalConfigurationBits ***
#define DAQmx_Val_Bit_TermCfg_RSE                                         (1<<0) // RSE terminal configuration
#define DAQmx_Val_Bit_TermCfg_NRSE                                        (1<<1) // NRSE terminal configuration
#define DAQmx_Val_Bit_TermCfg_Diff                                        (1<<2) // Differential terminal configuration
#define DAQmx_Val_Bit_TermCfg_PseudoDIFF                                  (1<<3) // Pseudodifferential terminal configuration


/******************************************************/
/***              Attribute Values                  ***/
/******************************************************/

//*** Values for DAQmx_AI_ACExcit_WireMode ***
//*** Value set ACExcitWireMode ***
#define DAQmx_Val_4Wire                                                       4 // 4-Wire
#define DAQmx_Val_5Wire                                                       5 // 5-Wire

//*** Values for DAQmx_AI_ADCTimingMode ***
//*** Value set ADCTimingMode ***
#define DAQmx_Val_HighResolution                                          10195 // High Resolution
#define DAQmx_Val_HighSpeed                                               14712 // High Speed
#define DAQmx_Val_Best50HzRejection                                       14713 // Best 50 Hz Rejection
#define DAQmx_Val_Best60HzRejection                                       14714 // Best 60 Hz Rejection

//*** Values for DAQmx_AI_MeasType ***
//*** Value set AIMeasurementType ***
#define DAQmx_Val_Voltage                                                 10322 // Voltage
#define DAQmx_Val_VoltageRMS                                              10350 // Voltage RMS
#define DAQmx_Val_Current                                                 10134 // Current
#define DAQmx_Val_CurrentRMS                                              10351 // Current RMS
#define DAQmx_Val_Voltage_CustomWithExcitation                            10323 // More:Voltage:Custom with Excitation
#define DAQmx_Val_Freq_Voltage                                            10181 // Frequency
#define DAQmx_Val_Resistance                                              10278 // Resistance
#define DAQmx_Val_Temp_TC                                                 10303 // Temperature:Thermocouple
#define DAQmx_Val_Temp_Thrmstr                                            10302 // Temperature:Thermistor
#define DAQmx_Val_Temp_RTD                                                10301 // Temperature:RTD
#define DAQmx_Val_Temp_BuiltInSensor                                      10311 // Temperature:Built-in Sensor
#define DAQmx_Val_Strain_Gage                                             10300 // Strain Gage
#define DAQmx_Val_Position_LVDT                                           10352 // Position:LVDT
#define DAQmx_Val_Position_RVDT                                           10353 // Position:RVDT
#define DAQmx_Val_Accelerometer                                           10356 // Accelerometer
#define DAQmx_Val_SoundPressure_Microphone                                10354 // Sound Pressure:Microphone
#define DAQmx_Val_TEDS_Sensor                                             12531 // TEDS Sensor

//*** Values for DAQmx_AO_IdleOutputBehavior ***
//*** Value set AOIdleOutputBehavior ***
#define DAQmx_Val_ZeroVolts                                               12526 // Zero Volts
#define DAQmx_Val_HighImpedance                                           12527 // High Impedance
#define DAQmx_Val_MaintainExistingValue                                   12528 // Maintain Existing Value

//*** Values for DAQmx_AO_OutputType ***
//*** Value set AOOutputChannelType ***
#define DAQmx_Val_Voltage                                                 10322 // Voltage
#define DAQmx_Val_Current                                                 10134 // Current
#define DAQmx_Val_FuncGen                                                 14750 // Function Generation

//*** Values for DAQmx_AI_Accel_SensitivityUnits ***
//*** Value set AccelSensitivityUnits1 ***
#define DAQmx_Val_mVoltsPerG                                              12509 // mVolts/g
#define DAQmx_Val_VoltsPerG                                               12510 // Volts/g

//*** Values for DAQmx_AI_Accel_Units ***
//*** Value set AccelUnits2 ***
#define DAQmx_Val_AccelUnit_g                                             10186 // g
#define DAQmx_Val_MetersPerSecondSquared                                  12470 // m/s^2
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_SampQuant_SampMode ***
//*** Value set AcquisitionType ***
#define DAQmx_Val_FiniteSamps                                             10178 // Finite Samples
#define DAQmx_Val_ContSamps                                               10123 // Continuous Samples
#define DAQmx_Val_HWTimedSinglePoint                                      12522 // Hardware Timed Single Point

//*** Values for DAQmx_AnlgLvl_PauseTrig_When ***
//*** Value set ActiveLevel ***
#define DAQmx_Val_AboveLvl                                                10093 // Above Level
#define DAQmx_Val_BelowLvl                                                10107 // Below Level

//*** Values for DAQmx_AI_RVDT_Units ***
//*** Value set AngleUnits1 ***
#define DAQmx_Val_Degrees                                                 10146 // Degrees
#define DAQmx_Val_Radians                                                 10273 // Radians
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_CI_AngEncoder_Units ***
//*** Value set AngleUnits2 ***
#define DAQmx_Val_Degrees                                                 10146 // Degrees
#define DAQmx_Val_Radians                                                 10273 // Radians
#define DAQmx_Val_Ticks                                                   10304 // Ticks
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_AI_AutoZeroMode ***
//*** Value set AutoZeroType1 ***
#define DAQmx_Val_None                                                    10230 // None
#define DAQmx_Val_Once                                                    10244 // Once
#define DAQmx_Val_EverySample                                             10164 // Every Sample

//*** Values for DAQmx_SwitchScan_BreakMode ***
//*** Value set BreakMode ***
#define DAQmx_Val_NoAction                                                10227 // No Action
#define DAQmx_Val_BreakBeforeMake                                         10110 // Break Before Make

//*** Values for DAQmx_AI_Bridge_Cfg ***
//*** Value set BridgeConfiguration1 ***
#define DAQmx_Val_FullBridge                                              10182 // Full Bridge
#define DAQmx_Val_HalfBridge                                              10187 // Half Bridge
#define DAQmx_Val_QuarterBridge                                           10270 // Quarter Bridge
#define DAQmx_Val_NoBridge                                                10228 // No Bridge

//*** Values for DAQmx_Dev_BusType ***
//*** Value set BusType ***
#define DAQmx_Val_PCI                                                     12582 // PCI
#define DAQmx_Val_PCIe                                                    13612 // PCIe
#define DAQmx_Val_PXI                                                     12583 // PXI
#define DAQmx_Val_PXIe                                                    14706 // PXIe
#define DAQmx_Val_SCXI                                                    12584 // SCXI
#define DAQmx_Val_SCC                                                     14707 // SCC
#define DAQmx_Val_PCCard                                                  12585 // PCCard
#define DAQmx_Val_USB                                                     12586 // USB
#define DAQmx_Val_CompactDAQ                                              14637 // CompactDAQ
#define DAQmx_Val_TCPIP                                                   14828 // TCP/IP
#define DAQmx_Val_Unknown                                                 12588 // Unknown

//*** Values for DAQmx_CI_MeasType ***
//*** Value set CIMeasurementType ***
#define DAQmx_Val_CountEdges                                              10125 // Count Edges
#define DAQmx_Val_Freq                                                    10179 // Frequency
#define DAQmx_Val_Period                                                  10256 // Period
#define DAQmx_Val_PulseWidth                                              10359 // Pulse Width
#define DAQmx_Val_SemiPeriod                                              10289 // Semi Period
#define DAQmx_Val_Position_AngEncoder                                     10360 // Position:Angular Encoder
#define DAQmx_Val_Position_LinEncoder                                     10361 // Position:Linear Encoder
#define DAQmx_Val_TwoEdgeSep                                              10267 // Two Edge Separation
#define DAQmx_Val_GPS_Timestamp                                           10362 // GPS Timestamp

//*** Values for DAQmx_AI_Thrmcpl_CJCSrc ***
//*** Value set CJCSource1 ***
#define DAQmx_Val_BuiltIn                                                 10200 // Built-In
#define DAQmx_Val_ConstVal                                                10116 // Constant Value
#define DAQmx_Val_Chan                                                    10113 // Channel

//*** Values for DAQmx_CO_OutputType ***
//*** Value set COOutputType ***
#define DAQmx_Val_Pulse_Time                                              10269 // Pulse:Time
#define DAQmx_Val_Pulse_Freq                                              10119 // Pulse:Frequency
#define DAQmx_Val_Pulse_Ticks                                             10268 // Pulse:Ticks

//*** Values for DAQmx_ChanType ***
//*** Value set ChannelType ***
#define DAQmx_Val_AI                                                      10100 // Analog Input
#define DAQmx_Val_AO                                                      10102 // Analog Output
#define DAQmx_Val_DI                                                      10151 // Digital Input
#define DAQmx_Val_DO                                                      10153 // Digital Output
#define DAQmx_Val_CI                                                      10131 // Counter Input
#define DAQmx_Val_CO                                                      10132 // Counter Output

//*** Values for DAQmx_CO_ConstrainedGenMode ***
//*** Value set ConstrainedGenMode ***
#define DAQmx_Val_Unconstrained                                           14708 // Unconstrained
#define DAQmx_Val_FixedHighFreq                                           14709 // Fixed High Frequency
#define DAQmx_Val_FixedLowFreq                                            14710 // Fixed Low Frequency
#define DAQmx_Val_Fixed50PercentDutyCycle                                 14711 // Fixed 50 Percent Duty Cycle

//*** Values for DAQmx_CI_CountEdges_Dir ***
//*** Value set CountDirection1 ***
#define DAQmx_Val_CountUp                                                 10128 // Count Up
#define DAQmx_Val_CountDown                                               10124 // Count Down
#define DAQmx_Val_ExtControlled                                           10326 // Externally Controlled

//*** Values for DAQmx_CI_Freq_MeasMeth ***
//*** Values for DAQmx_CI_Period_MeasMeth ***
//*** Value set CounterFrequencyMethod ***
#define DAQmx_Val_LowFreq1Ctr                                             10105 // Low Frequency with 1 Counter
#define DAQmx_Val_HighFreq2Ctr                                            10157 // High Frequency with 2 Counters
#define DAQmx_Val_LargeRng2Ctr                                            10205 // Large Range with 2 Counters

//*** Values for DAQmx_AI_Coupling ***
//*** Value set Coupling1 ***
#define DAQmx_Val_AC                                                      10045 // AC
#define DAQmx_Val_DC                                                      10050 // DC
#define DAQmx_Val_GND                                                     10066 // GND

//*** Values for DAQmx_AnlgEdge_StartTrig_Coupling ***
//*** Values for DAQmx_AnlgWin_StartTrig_Coupling ***
//*** Values for DAQmx_AnlgEdge_RefTrig_Coupling ***
//*** Values for DAQmx_AnlgWin_RefTrig_Coupling ***
//*** Values for DAQmx_AnlgLvl_PauseTrig_Coupling ***
//*** Values for DAQmx_AnlgWin_PauseTrig_Coupling ***
//*** Value set Coupling2 ***
#define DAQmx_Val_AC                                                      10045 // AC
#define DAQmx_Val_DC                                                      10050 // DC

//*** Values for DAQmx_AI_CurrentShunt_Loc ***
//*** Value set CurrentShuntResistorLocation1 ***
#define DAQmx_Val_Internal                                                10200 // Internal
#define DAQmx_Val_External                                                10167 // External

//*** Values for DAQmx_AI_Current_Units ***
//*** Values for DAQmx_AI_Current_ACRMS_Units ***
//*** Values for DAQmx_AO_Current_Units ***
//*** Value set CurrentUnits1 ***
#define DAQmx_Val_Amps                                                    10342 // Amps
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale
#define DAQmx_Val_FromTEDS                                                12516 // From TEDS

//*** Value set CurrentUnits2 ***
#define DAQmx_Val_Amps                                                    10342 // Amps
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_AI_RawSampJustification ***
//*** Value set DataJustification1 ***
#define DAQmx_Val_RightJustified                                          10279 // Right-Justified
#define DAQmx_Val_LeftJustified                                           10209 // Left-Justified

//*** Values for DAQmx_AI_DataXferMech ***
//*** Values for DAQmx_AO_DataXferMech ***
//*** Values for DAQmx_DI_DataXferMech ***
//*** Values for DAQmx_DO_DataXferMech ***
//*** Values for DAQmx_CI_DataXferMech ***
//*** Value set DataTransferMechanism ***
#define DAQmx_Val_DMA                                                     10054 // DMA
#define DAQmx_Val_Interrupts                                              10204 // Interrupts
#define DAQmx_Val_ProgrammedIO                                            10264 // Programmed I/O
#define DAQmx_Val_USBbulk                                                 12590 // USB Bulk

//*** Values for DAQmx_Exported_RdyForXferEvent_DeassertCond ***
//*** Value set DeassertCondition ***
#define DAQmx_Val_OnbrdMemMoreThanHalfFull                                10237 // Onboard Memory More than Half Full
#define DAQmx_Val_OnbrdMemFull                                            10236 // Onboard Memory Full
#define DAQmx_Val_OnbrdMemCustomThreshold                                 12577 // Onboard Memory Custom Threshold

//*** Values for DAQmx_DO_OutputDriveType ***
//*** Value set DigitalDriveType ***
#define DAQmx_Val_ActiveDrive                                             12573 // Active Drive
#define DAQmx_Val_OpenCollector                                           12574 // Open Collector

//*** Values for DAQmx_DO_LineStates_StartState ***
//*** Values for DAQmx_DO_LineStates_PausedState ***
//*** Values for DAQmx_DO_LineStates_DoneState ***
//*** Values for DAQmx_Watchdog_DO_ExpirState ***
//*** Value set DigitalLineState ***
#define DAQmx_Val_High                                                    10192 // High
#define DAQmx_Val_Low                                                     10214 // Low
#define DAQmx_Val_Tristate                                                10310 // Tristate
#define DAQmx_Val_NoChange                                                10160 // No Change

//*** Values for DAQmx_DigPattern_StartTrig_When ***
//*** Values for DAQmx_DigPattern_RefTrig_When ***
//*** Values for DAQmx_DigPattern_PauseTrig_When ***
//*** Value set DigitalPatternCondition1 ***
#define DAQmx_Val_PatternMatches                                          10254 // Pattern Matches
#define DAQmx_Val_PatternDoesNotMatch                                     10253 // Pattern Does Not Match

//*** Values for DAQmx_StartTrig_DelayUnits ***
//*** Value set DigitalWidthUnits1 ***
#define DAQmx_Val_SampClkPeriods                                          10286 // Sample Clock Periods
#define DAQmx_Val_Seconds                                                 10364 // Seconds
#define DAQmx_Val_Ticks                                                   10304 // Ticks

//*** Values for DAQmx_DelayFromSampClk_DelayUnits ***
//*** Value set DigitalWidthUnits2 ***
#define DAQmx_Val_Seconds                                                 10364 // Seconds
#define DAQmx_Val_Ticks                                                   10304 // Ticks

//*** Values for DAQmx_Exported_AdvTrig_Pulse_WidthUnits ***
//*** Value set DigitalWidthUnits3 ***
#define DAQmx_Val_Seconds                                                 10364 // Seconds

//*** Values for DAQmx_CI_Freq_StartingEdge ***
//*** Values for DAQmx_CI_Period_StartingEdge ***
//*** Values for DAQmx_CI_CountEdges_ActiveEdge ***
//*** Values for DAQmx_CI_PulseWidth_StartingEdge ***
//*** Values for DAQmx_CI_TwoEdgeSep_FirstEdge ***
//*** Values for DAQmx_CI_TwoEdgeSep_SecondEdge ***
//*** Values for DAQmx_CI_SemiPeriod_StartingEdge ***
//*** Values for DAQmx_CI_CtrTimebaseActiveEdge ***
//*** Values for DAQmx_CO_CtrTimebaseActiveEdge ***
//*** Values for DAQmx_SampClk_ActiveEdge ***
//*** Values for DAQmx_SampClk_Timebase_ActiveEdge ***
//*** Values for DAQmx_AIConv_ActiveEdge ***
//*** Values for DAQmx_DigEdge_StartTrig_Edge ***
//*** Values for DAQmx_DigEdge_RefTrig_Edge ***
//*** Values for DAQmx_DigEdge_AdvTrig_Edge ***
//*** Values for DAQmx_DigEdge_ArmStartTrig_Edge ***
//*** Values for DAQmx_DigEdge_WatchdogExpirTrig_Edge ***
//*** Value set Edge1 ***
#define DAQmx_Val_Rising                                                  10280 // Rising
#define DAQmx_Val_Falling                                                 10171 // Falling

//*** Values for DAQmx_CI_Encoder_DecodingType ***
//*** Value set EncoderType2 ***
#define DAQmx_Val_X1                                                      10090 // X1
#define DAQmx_Val_X2                                                      10091 // X2
#define DAQmx_Val_X4                                                      10092 // X4
#define DAQmx_Val_TwoPulseCounting                                        10313 // Two Pulse Counting

//*** Values for DAQmx_CI_Encoder_ZIndexPhase ***
//*** Value set EncoderZIndexPhase1 ***
#define DAQmx_Val_AHighBHigh                                              10040 // A High B High
#define DAQmx_Val_AHighBLow                                               10041 // A High B Low
#define DAQmx_Val_ALowBHigh                                               10042 // A Low B High
#define DAQmx_Val_ALowBLow                                                10043 // A Low B Low

//*** Values for DAQmx_AI_Excit_DCorAC ***
//*** Value set ExcitationDCorAC ***
#define DAQmx_Val_DC                                                      10050 // DC
#define DAQmx_Val_AC                                                      10045 // AC

//*** Values for DAQmx_AI_Excit_Src ***
//*** Value set ExcitationSource ***
#define DAQmx_Val_Internal                                                10200 // Internal
#define DAQmx_Val_External                                                10167 // External
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_AI_Excit_VoltageOrCurrent ***
//*** Value set ExcitationVoltageOrCurrent ***
#define DAQmx_Val_Voltage                                                 10322 // Voltage
#define DAQmx_Val_Current                                                 10134 // Current

//*** Values for DAQmx_Exported_CtrOutEvent_OutputBehavior ***
//*** Value set ExportActions2 ***
#define DAQmx_Val_Pulse                                                   10265 // Pulse
#define DAQmx_Val_Toggle                                                  10307 // Toggle

//*** Values for DAQmx_Exported_SampClk_OutputBehavior ***
//*** Value set ExportActions3 ***
#define DAQmx_Val_Pulse                                                   10265 // Pulse
#define DAQmx_Val_Lvl                                                     10210 // Level

//*** Values for DAQmx_Exported_HshkEvent_OutputBehavior ***
//*** Value set ExportActions5 ***
#define DAQmx_Val_Interlocked                                             12549 // Interlocked
#define DAQmx_Val_Pulse                                                   10265 // Pulse

//*** Values for DAQmx_AI_Freq_Units ***
//*** Value set FrequencyUnits ***
#define DAQmx_Val_Hz                                                      10373 // Hz
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_CO_Pulse_Freq_Units ***
//*** Value set FrequencyUnits2 ***
#define DAQmx_Val_Hz                                                      10373 // Hz

//*** Values for DAQmx_CI_Freq_Units ***
//*** Value set FrequencyUnits3 ***
#define DAQmx_Val_Hz                                                      10373 // Hz
#define DAQmx_Val_Ticks                                                   10304 // Ticks
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_AO_FuncGen_Type ***
//*** Value set FuncGenType ***
#define DAQmx_Val_Sine                                                    14751 // Sine
#define DAQmx_Val_Triangle                                                14752 // Triangle
#define DAQmx_Val_Square                                                  14753 // Square
#define DAQmx_Val_Sawtooth                                                14754 // Sawtooth

//*** Values for DAQmx_CI_GPS_SyncMethod ***
//*** Value set GpsSignalType1 ***
#define DAQmx_Val_IRIGB                                                   10070 // IRIG-B
#define DAQmx_Val_PPS                                                     10080 // PPS
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_Hshk_StartCond ***
//*** Value set HandshakeStartCondition ***
#define DAQmx_Val_Immediate                                               10198 // Immediate
#define DAQmx_Val_WaitForHandshakeTriggerAssert                           12550 // Wait For Handshake Trigger Assert
#define DAQmx_Val_WaitForHandshakeTriggerDeassert                         12551 // Wait For Handshake Trigger Deassert


//*** Values for DAQmx_AI_DataXferReqCond ***
//*** Values for DAQmx_DI_DataXferReqCond ***
//*** Value set InputDataTransferCondition ***
#define DAQmx_Val_OnBrdMemMoreThanHalfFull                                10237 // Onboard Memory More than Half Full
#define DAQmx_Val_OnBrdMemNotEmpty                                        10241 // Onboard Memory Not Empty
#define DAQmx_Val_OnbrdMemCustomThreshold                                 12577 // Onboard Memory Custom Threshold
#define DAQmx_Val_WhenAcqComplete                                         12546 // When Acquisition Complete

//*** Values for DAQmx_AI_TermCfg ***
//*** Value set InputTermCfg ***
#define DAQmx_Val_RSE                                                     10083 // RSE
#define DAQmx_Val_NRSE                                                    10078 // NRSE
#define DAQmx_Val_Diff                                                    10106 // Differential
#define DAQmx_Val_PseudoDiff                                              12529 // Pseudodifferential

//*** Values for DAQmx_AI_LVDT_SensitivityUnits ***
//*** Value set LVDTSensitivityUnits1 ***
#define DAQmx_Val_mVoltsPerVoltPerMillimeter                              12506 // mVolts/Volt/mMeter
#define DAQmx_Val_mVoltsPerVoltPerMilliInch                               12505 // mVolts/Volt/0.001 Inch

//*** Values for DAQmx_AI_LVDT_Units ***
//*** Value set LengthUnits2 ***
#define DAQmx_Val_Meters                                                  10219 // Meters
#define DAQmx_Val_Inches                                                  10379 // Inches
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_CI_LinEncoder_Units ***
//*** Value set LengthUnits3 ***
#define DAQmx_Val_Meters                                                  10219 // Meters
#define DAQmx_Val_Inches                                                  10379 // Inches
#define DAQmx_Val_Ticks                                                   10304 // Ticks
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_CI_OutputState ***
//*** Values for DAQmx_CO_Pulse_IdleState ***
//*** Values for DAQmx_CO_OutputState ***
//*** Values for DAQmx_Exported_CtrOutEvent_Toggle_IdleState ***
//*** Values for DAQmx_Exported_HshkEvent_Interlocked_AssertedLvl ***
//*** Values for DAQmx_Interlocked_HshkTrig_AssertedLvl ***
//*** Values for DAQmx_DigLvl_PauseTrig_When ***
//*** Value set Level1 ***
#define DAQmx_Val_High                                                    10192 // High
#define DAQmx_Val_Low                                                     10214 // Low

//*** Values for DAQmx_DI_LogicFamily ***
//*** Values for DAQmx_DO_LogicFamily ***
//*** Value set LogicFamily ***
#define DAQmx_Val_2point5V                                                14620 // 2.5 V
#define DAQmx_Val_3point3V                                                14621 // 3.3 V
#define DAQmx_Val_5V                                                      14619 // 5.0 V

//*** Values for DAQmx_AIConv_Timebase_Src ***
//*** Value set MIOAIConvertTbSrc ***
#define DAQmx_Val_SameAsSampTimebase                                      10284 // Same as Sample Timebase
#define DAQmx_Val_SameAsMasterTimebase                                    10282 // Same as Master Timebase
#define DAQmx_Val_20MHzTimebase                                           12537 // 20MHz Timebase
#define DAQmx_Val_80MHzTimebase                                           14636 // 80MHz Timebase

//*** Values for DAQmx_AO_FuncGen_ModulationType ***
//*** Value set ModulationType ***
#define DAQmx_Val_AM                                                      14756 // AM
#define DAQmx_Val_FM                                                      14757 // FM
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_AO_DataXferReqCond ***
//*** Values for DAQmx_DO_DataXferReqCond ***
//*** Value set OutputDataTransferCondition ***
#define DAQmx_Val_OnBrdMemEmpty                                           10235 // Onboard Memory Empty
#define DAQmx_Val_OnBrdMemHalfFullOrLess                                  10239 // Onboard Memory Half Full or Less
#define DAQmx_Val_OnBrdMemNotFull                                         10242 // Onboard Memory Less than Full

//*** Values for DAQmx_AO_TermCfg ***
//*** Value set OutputTermCfg ***
#define DAQmx_Val_RSE                                                     10083 // RSE
#define DAQmx_Val_Diff                                                    10106 // Differential
#define DAQmx_Val_PseudoDiff                                              12529 // Pseudodifferential

//*** Values for DAQmx_Read_OverWrite ***
//*** Value set OverwriteMode1 ***
#define DAQmx_Val_OverwriteUnreadSamps                                    10252 // Overwrite Unread Samples
#define DAQmx_Val_DoNotOverwriteUnreadSamps                               10159 // Do Not Overwrite Unread Samples

//*** Values for DAQmx_Exported_AIConvClk_Pulse_Polarity ***
//*** Values for DAQmx_Exported_SampClk_Pulse_Polarity ***
//*** Values for DAQmx_Exported_AdvTrig_Pulse_Polarity ***
//*** Values for DAQmx_Exported_PauseTrig_Lvl_ActiveLvl ***
//*** Values for DAQmx_Exported_RefTrig_Pulse_Polarity ***
//*** Values for DAQmx_Exported_StartTrig_Pulse_Polarity ***
//*** Values for DAQmx_Exported_AdvCmpltEvent_Pulse_Polarity ***
//*** Values for DAQmx_Exported_AIHoldCmpltEvent_PulsePolarity ***
//*** Values for DAQmx_Exported_ChangeDetectEvent_Pulse_Polarity ***
//*** Values for DAQmx_Exported_CtrOutEvent_Pulse_Polarity ***
//*** Values for DAQmx_Exported_HshkEvent_Pulse_Polarity ***
//*** Values for DAQmx_Exported_RdyForXferEvent_Lvl_ActiveLvl ***
//*** Values for DAQmx_Exported_DataActiveEvent_Lvl_ActiveLvl ***
//*** Values for DAQmx_Exported_RdyForStartEvent_Lvl_ActiveLvl ***
//*** Value set Polarity2 ***
#define DAQmx_Val_ActiveHigh                                              10095 // Active High
#define DAQmx_Val_ActiveLow                                               10096 // Active Low

//*** Values for DAQmx_Dev_ProductCategory ***
//*** Value set ProductCategory ***
#define DAQmx_Val_MSeriesDAQ                                              14643 // M Series DAQ
#define DAQmx_Val_ESeriesDAQ                                              14642 // E Series DAQ
#define DAQmx_Val_SSeriesDAQ                                              14644 // S Series DAQ
#define DAQmx_Val_BSeriesDAQ                                              14662 // B Series DAQ
#define DAQmx_Val_SCSeriesDAQ                                             14645 // SC Series DAQ
#define DAQmx_Val_USBDAQ                                                  14646 // USB DAQ
#define DAQmx_Val_AOSeries                                                14647 // AO Series
#define DAQmx_Val_DigitalIO                                               14648 // Digital I/O
#define DAQmx_Val_TIOSeries                                               14661 // TIO Series
#define DAQmx_Val_DynamicSignalAcquisition                                14649 // Dynamic Signal Acquisition
#define DAQmx_Val_Switches                                                14650 // Switches
#define DAQmx_Val_CompactDAQChassis                                       14658 // CompactDAQ Chassis
#define DAQmx_Val_CSeriesModule                                           14659 // C Series Module
#define DAQmx_Val_SCXIModule                                              14660 // SCXI Module
#define DAQmx_Val_SCCConnectorBlock                                       14704 // SCC Connector Block
#define DAQmx_Val_SCCModule                                               14705 // SCC Module
#define DAQmx_Val_NIELVIS                                                 14755 // NI ELVIS
#define DAQmx_Val_NetworkDAQ                                              14829 // Network DAQ
#define DAQmx_Val_Unknown                                                 12588 // Unknown

//*** Values for DAQmx_AI_RTD_Type ***
//*** Value set RTDType1 ***
#define DAQmx_Val_Pt3750                                                  12481 // Pt3750
#define DAQmx_Val_Pt3851                                                  10071 // Pt3851
#define DAQmx_Val_Pt3911                                                  12482 // Pt3911
#define DAQmx_Val_Pt3916                                                  10069 // Pt3916
#define DAQmx_Val_Pt3920                                                  10053 // Pt3920
#define DAQmx_Val_Pt3928                                                  12483 // Pt3928
#define DAQmx_Val_Custom                                                  10137 // Custom

//*** Values for DAQmx_AI_RVDT_SensitivityUnits ***
//*** Value set RVDTSensitivityUnits1 ***
#define DAQmx_Val_mVoltsPerVoltPerDegree                                  12507 // mVolts/Volt/Degree
#define DAQmx_Val_mVoltsPerVoltPerRadian                                  12508 // mVolts/Volt/Radian

//*** Values for DAQmx_AI_RawDataCompressionType ***
//*** Value set RawDataCompressionType ***
#define DAQmx_Val_None                                                    10230 // None
#define DAQmx_Val_LosslessPacking                                         12555 // Lossless Packing
#define DAQmx_Val_LossyLSBRemoval                                         12556 // Lossy LSB Removal

//*** Values for DAQmx_Read_RelativeTo ***
//*** Value set ReadRelativeTo ***
#define DAQmx_Val_FirstSample                                             10424 // First Sample
#define DAQmx_Val_CurrReadPos                                             10425 // Current Read Position
#define DAQmx_Val_RefTrig                                                 10426 // Reference Trigger
#define DAQmx_Val_FirstPretrigSamp                                        10427 // First Pretrigger Sample
#define DAQmx_Val_MostRecentSamp                                          10428 // Most Recent Sample

//*** Values for DAQmx_Write_RegenMode ***
//*** Value set RegenerationMode1 ***
#define DAQmx_Val_AllowRegen                                              10097 // Allow Regeneration
#define DAQmx_Val_DoNotAllowRegen                                         10158 // Do Not Allow Regeneration

//*** Values for DAQmx_AI_ResistanceCfg ***
//*** Value set ResistanceConfiguration ***
#define DAQmx_Val_2Wire                                                       2 // 2-Wire
#define DAQmx_Val_3Wire                                                       3 // 3-Wire
#define DAQmx_Val_4Wire                                                       4 // 4-Wire

//*** Values for DAQmx_AI_Resistance_Units ***
//*** Value set ResistanceUnits1 ***
#define DAQmx_Val_Ohms                                                    10384 // Ohms
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale
#define DAQmx_Val_FromTEDS                                                12516 // From TEDS

//*** Value set ResistanceUnits2 ***
#define DAQmx_Val_Ohms                                                    10384 // Ohms
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_AI_ResolutionUnits ***
//*** Values for DAQmx_AO_ResolutionUnits ***
//*** Value set ResolutionType1 ***
#define DAQmx_Val_Bits                                                    10109 // Bits

//*** Value set SCXI1124Range ***
#define DAQmx_Val_SCXI1124Range0to1V                                      14629 // 0V to 1V
#define DAQmx_Val_SCXI1124Range0to5V                                      14630 // 0V to 5V
#define DAQmx_Val_SCXI1124Range0to10V                                     14631 // 0V to 10V
#define DAQmx_Val_SCXI1124RangeNeg1to1V                                   14632 // -1V to 1V
#define DAQmx_Val_SCXI1124RangeNeg5to5V                                   14633 // -5V to 5V
#define DAQmx_Val_SCXI1124RangeNeg10to10V                                 14634 // -10V to 10V
#define DAQmx_Val_SCXI1124Range0to20mA                                    14635 // 0mA to 20mA

//*** Values for DAQmx_DI_AcquireOn ***
//*** Values for DAQmx_DO_GenerateOn ***
//*** Value set SampleClockActiveOrInactiveEdgeSelection ***
#define DAQmx_Val_SampClkActiveEdge                                       14617 // Sample Clock Active Edge
#define DAQmx_Val_SampClkInactiveEdge                                     14618 // Sample Clock Inactive Edge

//*** Values for DAQmx_Hshk_SampleInputDataWhen ***
//*** Value set SampleInputDataWhen ***
#define DAQmx_Val_HandshakeTriggerAsserts                                 12552 // Handshake Trigger Asserts
#define DAQmx_Val_HandshakeTriggerDeasserts                               12553 // Handshake Trigger Deasserts

//*** Values for DAQmx_SampTimingType ***
//*** Value set SampleTimingType ***
#define DAQmx_Val_SampClk                                                 10388 // Sample Clock
#define DAQmx_Val_BurstHandshake                                          12548 // Burst Handshake
#define DAQmx_Val_Handshake                                               10389 // Handshake
#define DAQmx_Val_Implicit                                                10451 // Implicit
#define DAQmx_Val_OnDemand                                                10390 // On Demand
#define DAQmx_Val_ChangeDetection                                         12504 // Change Detection
#define DAQmx_Val_PipelinedSampClk                                        14668 // Pipelined Sample Clock

//*** Values for DAQmx_Scale_Type ***
//*** Value set ScaleType ***
#define DAQmx_Val_Linear                                                  10447 // Linear
#define DAQmx_Val_MapRanges                                               10448 // Map Ranges
#define DAQmx_Val_Polynomial                                              10449 // Polynomial
#define DAQmx_Val_Table                                                   10450 // Table

//*** Values for DAQmx_AI_Thrmcpl_ScaleType ***
//*** Value set ScaleType2 ***
#define DAQmx_Val_Polynomial                                              10449 // Polynomial
#define DAQmx_Val_Table                                                   10450 // Table

//*** Values for DAQmx_AI_ChanCal_ScaleType ***
//*** Value set ScaleType3 ***
#define DAQmx_Val_Polynomial                                              10449 // Polynomial
#define DAQmx_Val_Table                                                   10450 // Table
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_AI_Bridge_ShuntCal_Select ***
//*** Value set ShuntCalSelect ***
#define DAQmx_Val_A                                                       12513 // A
#define DAQmx_Val_B                                                       12514 // B
#define DAQmx_Val_AandB                                                   12515 // A and B

//*** Value set ShuntElementLocation ***
#define DAQmx_Val_R1                                                      12465 // R1
#define DAQmx_Val_R2                                                      12466 // R2
#define DAQmx_Val_R3                                                      12467 // R3
#define DAQmx_Val_R4                                                      14813 // R4
#define DAQmx_Val_None                                                    10230 // None

//*** Value set Signal ***
#define DAQmx_Val_AIConvertClock                                          12484 // AI Convert Clock
#define DAQmx_Val_10MHzRefClock                                           12536 // 10MHz Reference Clock
#define DAQmx_Val_20MHzTimebaseClock                                      12486 // 20MHz Timebase Clock
#define DAQmx_Val_SampleClock                                             12487 // Sample Clock
#define DAQmx_Val_AdvanceTrigger                                          12488 // Advance Trigger
#define DAQmx_Val_ReferenceTrigger                                        12490 // Reference Trigger
#define DAQmx_Val_StartTrigger                                            12491 // Start Trigger
#define DAQmx_Val_AdvCmpltEvent                                           12492 // Advance Complete Event
#define DAQmx_Val_AIHoldCmpltEvent                                        12493 // AI Hold Complete Event
#define DAQmx_Val_CounterOutputEvent                                      12494 // Counter Output Event
#define DAQmx_Val_ChangeDetectionEvent                                    12511 // Change Detection Event
#define DAQmx_Val_WDTExpiredEvent                                         12512 // Watchdog Timer Expired Event

//*** Value set Signal2 ***
#define DAQmx_Val_SampleCompleteEvent                                     12530 // Sample Complete Event
#define DAQmx_Val_CounterOutputEvent                                      12494 // Counter Output Event
#define DAQmx_Val_ChangeDetectionEvent                                    12511 // Change Detection Event
#define DAQmx_Val_SampleClock                                             12487 // Sample Clock

//*** Values for DAQmx_AnlgEdge_StartTrig_Slope ***
//*** Values for DAQmx_AnlgEdge_RefTrig_Slope ***
//*** Value set Slope1 ***
#define DAQmx_Val_RisingSlope                                             10280 // Rising
#define DAQmx_Val_FallingSlope                                            10171 // Falling

//*** Values for DAQmx_AI_SoundPressure_Units ***
//*** Value set SoundPressureUnits1 ***
#define DAQmx_Val_Pascals                                                 10081 // Pascals
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_AI_Lowpass_SwitchCap_ClkSrc ***
//*** Values for DAQmx_AO_DAC_Ref_Src ***
//*** Values for DAQmx_AO_DAC_Offset_Src ***
//*** Value set SourceSelection ***
#define DAQmx_Val_Internal                                                10200 // Internal
#define DAQmx_Val_External                                                10167 // External

//*** Values for DAQmx_AI_StrainGage_Cfg ***
//*** Value set StrainGageBridgeType1 ***
#define DAQmx_Val_FullBridgeI                                             10183 // Full Bridge I
#define DAQmx_Val_FullBridgeII                                            10184 // Full Bridge II
#define DAQmx_Val_FullBridgeIII                                           10185 // Full Bridge III
#define DAQmx_Val_HalfBridgeI                                             10188 // Half Bridge I
#define DAQmx_Val_HalfBridgeII                                            10189 // Half Bridge II
#define DAQmx_Val_QuarterBridgeI                                          10271 // Quarter Bridge I
#define DAQmx_Val_QuarterBridgeII                                         10272 // Quarter Bridge II

//*** Values for DAQmx_AI_Strain_Units ***
//*** Value set StrainUnits1 ***
#define DAQmx_Val_Strain                                                  10299 // Strain
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_SwitchScan_RepeatMode ***
//*** Value set SwitchScanRepeatMode ***
#define DAQmx_Val_Finite                                                  10172 // Finite
#define DAQmx_Val_Cont                                                    10117 // Continuous

//*** Values for DAQmx_SwitchChan_Usage ***
//*** Value set SwitchUsageTypes ***
#define DAQmx_Val_Source                                                  10439 // Source
#define DAQmx_Val_Load                                                    10440 // Load
#define DAQmx_Val_ReservedForRouting                                      10441 // Reserved for Routing

//*** Value set TEDSUnits ***
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale
#define DAQmx_Val_FromTEDS                                                12516 // From TEDS

//*** Values for DAQmx_AI_Temp_Units ***
//*** Value set TemperatureUnits1 ***
#define DAQmx_Val_DegC                                                    10143 // Deg C
#define DAQmx_Val_DegF                                                    10144 // Deg F
#define DAQmx_Val_Kelvins                                                 10325 // Kelvins
#define DAQmx_Val_DegR                                                    10145 // Deg R
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_AI_Thrmcpl_Type ***
//*** Value set ThermocoupleType1 ***
#define DAQmx_Val_J_Type_TC                                               10072 // J
#define DAQmx_Val_K_Type_TC                                               10073 // K
#define DAQmx_Val_N_Type_TC                                               10077 // N
#define DAQmx_Val_R_Type_TC                                               10082 // R
#define DAQmx_Val_S_Type_TC                                               10085 // S
#define DAQmx_Val_T_Type_TC                                               10086 // T
#define DAQmx_Val_B_Type_TC                                               10047 // B
#define DAQmx_Val_E_Type_TC                                               10055 // E

//*** Values for DAQmx_CI_Timestamp_Units ***
//*** Value set TimeUnits ***
#define DAQmx_Val_Seconds                                                 10364 // Seconds
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_CO_Pulse_Time_Units ***
//*** Value set TimeUnits2 ***
#define DAQmx_Val_Seconds                                                 10364 // Seconds

//*** Values for DAQmx_CI_Period_Units ***
//*** Values for DAQmx_CI_PulseWidth_Units ***
//*** Values for DAQmx_CI_TwoEdgeSep_Units ***
//*** Values for DAQmx_CI_SemiPeriod_Units ***
//*** Value set TimeUnits3 ***
#define DAQmx_Val_Seconds                                                 10364 // Seconds
#define DAQmx_Val_Ticks                                                   10304 // Ticks
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Value set TimingResponseMode ***
#define DAQmx_Val_SingleCycle                                             14613 // Single-cycle
#define DAQmx_Val_Multicycle                                              14614 // Multicycle

//*** Values for DAQmx_ArmStartTrig_Type ***
//*** Values for DAQmx_WatchdogExpirTrig_Type ***
//*** Value set TriggerType4 ***
#define DAQmx_Val_DigEdge                                                 10150 // Digital Edge
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_AdvTrig_Type ***
//*** Value set TriggerType5 ***
#define DAQmx_Val_DigEdge                                                 10150 // Digital Edge
#define DAQmx_Val_Software                                                10292 // Software
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_PauseTrig_Type ***
//*** Value set TriggerType6 ***
#define DAQmx_Val_AnlgLvl                                                 10101 // Analog Level
#define DAQmx_Val_AnlgWin                                                 10103 // Analog Window
#define DAQmx_Val_DigLvl                                                  10152 // Digital Level
#define DAQmx_Val_DigPattern                                              10398 // Digital Pattern
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_StartTrig_Type ***
//*** Values for DAQmx_RefTrig_Type ***
//*** Value set TriggerType8 ***
#define DAQmx_Val_AnlgEdge                                                10099 // Analog Edge
#define DAQmx_Val_DigEdge                                                 10150 // Digital Edge
#define DAQmx_Val_DigPattern                                              10398 // Digital Pattern
#define DAQmx_Val_AnlgWin                                                 10103 // Analog Window
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_HshkTrig_Type ***
//*** Value set TriggerType9 ***
#define DAQmx_Val_Interlocked                                             12549 // Interlocked
#define DAQmx_Val_None                                                    10230 // None

//*** Values for DAQmx_SampClk_UnderflowBehavior ***
//*** Value set UnderflowBehavior ***
#define DAQmx_Val_HaltOutputAndError                                      14615 // Halt Output and Error
#define DAQmx_Val_PauseUntilDataAvailable                                 14616 // Pause until Data Available

//*** Values for DAQmx_Scale_PreScaledUnits ***
//*** Value set UnitsPreScaled ***
#define DAQmx_Val_Volts                                                   10348 // Volts
#define DAQmx_Val_Amps                                                    10342 // Amps
#define DAQmx_Val_DegF                                                    10144 // Deg F
#define DAQmx_Val_DegC                                                    10143 // Deg C
#define DAQmx_Val_DegR                                                    10145 // Deg R
#define DAQmx_Val_Kelvins                                                 10325 // Kelvins
#define DAQmx_Val_Strain                                                  10299 // Strain
#define DAQmx_Val_Ohms                                                    10384 // Ohms
#define DAQmx_Val_Hz                                                      10373 // Hz
#define DAQmx_Val_Seconds                                                 10364 // Seconds
#define DAQmx_Val_Meters                                                  10219 // Meters
#define DAQmx_Val_Inches                                                  10379 // Inches
#define DAQmx_Val_Degrees                                                 10146 // Degrees
#define DAQmx_Val_Radians                                                 10273 // Radians
#define DAQmx_Val_g                                                       10186 // g
#define DAQmx_Val_MetersPerSecondSquared                                  12470 // m/s^2
#define DAQmx_Val_Pascals                                                 10081 // Pascals
#define DAQmx_Val_FromTEDS                                                12516 // From TEDS

//*** Values for DAQmx_AI_Voltage_Units ***
//*** Values for DAQmx_AI_Voltage_ACRMS_Units ***
//*** Value set VoltageUnits1 ***
#define DAQmx_Val_Volts                                                   10348 // Volts
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale
#define DAQmx_Val_FromTEDS                                                12516 // From TEDS

//*** Values for DAQmx_AO_Voltage_Units ***
//*** Value set VoltageUnits2 ***
#define DAQmx_Val_Volts                                                   10348 // Volts
#define DAQmx_Val_FromCustomScale                                         10065 // From Custom Scale

//*** Values for DAQmx_Read_WaitMode ***
//*** Value set WaitMode ***
#define DAQmx_Val_WaitForInterrupt                                        12523 // Wait For Interrupt
#define DAQmx_Val_Poll                                                    12524 // Poll
#define DAQmx_Val_Yield                                                   12525 // Yield
#define DAQmx_Val_Sleep                                                   12547 // Sleep

//*** Values for DAQmx_Write_WaitMode ***
//*** Value set WaitMode2 ***
#define DAQmx_Val_Poll                                                    12524 // Poll
#define DAQmx_Val_Yield                                                   12525 // Yield
#define DAQmx_Val_Sleep                                                   12547 // Sleep

//*** Values for DAQmx_RealTime_WaitForNextSampClkWaitMode ***
//*** Value set WaitMode3 ***
#define DAQmx_Val_WaitForInterrupt                                        12523 // Wait For Interrupt
#define DAQmx_Val_Poll                                                    12524 // Poll

//*** Values for DAQmx_RealTime_WriteRecoveryMode ***
//*** Value set WaitMode4 ***
#define DAQmx_Val_WaitForInterrupt                                        12523 // Wait For Interrupt
#define DAQmx_Val_Poll                                                    12524 // Poll

//*** Values for DAQmx_AnlgWin_StartTrig_When ***
//*** Values for DAQmx_AnlgWin_RefTrig_When ***
//*** Value set WindowTriggerCondition1 ***
#define DAQmx_Val_EnteringWin                                             10163 // Entering Window
#define DAQmx_Val_LeavingWin                                              10208 // Leaving Window

//*** Values for DAQmx_AnlgWin_PauseTrig_When ***
//*** Value set WindowTriggerCondition2 ***
#define DAQmx_Val_InsideWin                                               10199 // Inside Window
#define DAQmx_Val_OutsideWin                                              10251 // Outside Window

//*** Value set WriteBasicTEDSOptions ***
#define DAQmx_Val_WriteToEEPROM                                           12538 // Write To EEPROM
#define DAQmx_Val_WriteToPROM                                             12539 // Write To PROM Once
#define DAQmx_Val_DoNotWrite                                              12540 // Do Not Write

//*** Values for DAQmx_Write_RelativeTo ***
//*** Value set WriteRelativeTo ***
#define DAQmx_Val_FirstSample                                             10424 // First Sample
#define DAQmx_Val_CurrWritePos                                            10430 // Current Write Position


/******************************************************************************
 *** NI-DAQmx Function Declarations *******************************************
 ******************************************************************************/

/******************************************************/
/***         Task Configuration/Control             ***/
/******************************************************/


int32n __CFUNC     DAQmxLoadTask            (const char taskName[], TaskHandle *taskHandle);
int32n __CFUNC     DAQmxCreateTask          (const char taskName[], TaskHandle *taskHandle);
// Channel Names must be valid channels already available in MAX. They are not created.
int32n __CFUNC     DAQmxAddGlobalChansToTask(TaskHandle taskHandle, const char channelNames[]);

int32n __CFUNC     DAQmxStartTask           (TaskHandle taskHandle);
int32n __CFUNC     DAQmxStopTask            (TaskHandle taskHandle);

int32n __CFUNC     DAQmxClearTask           (TaskHandle taskHandle);

int32n __CFUNC     DAQmxWaitUntilTaskDone   (TaskHandle taskHandle, float64n timeToWait);
int32n __CFUNC     DAQmxIsTaskDone          (TaskHandle taskHandle, bool32n *isTaskDone);

int32n __CFUNC     DAQmxTaskControl         (TaskHandle taskHandle, int32n action);

int32n __CFUNC     DAQmxGetNthTaskChannel   (TaskHandle taskHandle, uint32n index, char buffer[], int32n bufferSize);

int32n __CFUNC     DAQmxGetNthTaskDevice    (TaskHandle taskHandle, uint32n index, char buffer[], int32n bufferSize);

int32n __CFUNC_C   DAQmxGetTaskAttribute    (TaskHandle taskHandle, int32n attribute, void *value, ...);

typedef int32n (CVICALLBACK *DAQmxEveryNSamplesEventCallbackPtr)(TaskHandle taskHandle, int32n everyNsamplesEventType, uint32n nSamples, void *callbackData);
typedef int32n (CVICALLBACK *DAQmxDoneEventCallbackPtr)(TaskHandle taskHandle, int32n status, void *callbackData);
typedef int32n (CVICALLBACK *DAQmxSignalEventCallbackPtr)(TaskHandle taskHandle, int32n signalID, void *callbackData);

int32n __CFUNC     DAQmxRegisterEveryNSamplesEvent (TaskHandle task, int32n everyNsamplesEventType, uint32n nSamples, uint32n options, DAQmxEveryNSamplesEventCallbackPtr callbackFunction, void *callbackData);
int32n __CFUNC     DAQmxRegisterDoneEvent          (TaskHandle task, uint32n options, DAQmxDoneEventCallbackPtr callbackFunction, void *callbackData);
int32n __CFUNC     DAQmxRegisterSignalEvent        (TaskHandle task, int32n signalID, uint32n options, DAQmxSignalEventCallbackPtr callbackFunction, void *callbackData);

/******************************************************/
/***        Channel Configuration/Creation          ***/
/******************************************************/


int32n __CFUNC     DAQmxCreateAIVoltageChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAICurrentChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, int32n shuntResistorLoc, float64n extShuntResistorVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAIVoltageRMSChan       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAICurrentRMSChan       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, int32n shuntResistorLoc, float64n extShuntResistorVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAIThrmcplChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n thermocoupleType, int32n cjcSource, float64n cjcVal, const char cjcChannel[]);
int32n __CFUNC     DAQmxCreateAIRTDChan              (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n rtdType, int32n resistanceConfig, int32n currentExcitSource, float64n currentExcitVal, float64n r0);
int32n __CFUNC     DAQmxCreateAIThrmstrChanIex       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n resistanceConfig, int32n currentExcitSource, float64n currentExcitVal, float64n a, float64n b, float64n c);
int32n __CFUNC     DAQmxCreateAIThrmstrChanVex       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n resistanceConfig, int32n voltageExcitSource, float64n voltageExcitVal, float64n a, float64n b, float64n c, float64n r1);
int32n __CFUNC     DAQmxCreateAIFreqVoltageChan      (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, float64n thresholdLevel, float64n hysteresis, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAIResistanceChan       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n resistanceConfig, int32n currentExcitSource, float64n currentExcitVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAIStrainGageChan       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n strainConfig, int32n voltageExcitSource, float64n voltageExcitVal, float64n gageFactor, float64n initialBridgeVoltage, float64n nominalGageResistance, float64n poissonRatio, float64n leadWireResistance, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAIVoltageChanWithExcit (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, int32n bridgeConfig, int32n voltageExcitSource, float64n voltageExcitVal, bool32n useExcitForScaling, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAITempBuiltInSensorChan(TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n units);
int32n __CFUNC     DAQmxCreateAIAccelChan            (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, float64n sensitivity, int32n sensitivityUnits, int32n currentExcitSource, float64n currentExcitVal, const char customScaleName[]);

int32n __CFUNC     DAQmxCreateAIMicrophoneChan       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, int32n units, float64n micSensitivity, float64n maxSndPressLevel, int32n currentExcitSource, float64n currentExcitVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAIPosLVDTChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, float64n sensitivity, int32n sensitivityUnits, int32n voltageExcitSource, float64n voltageExcitVal, float64n voltageExcitFreq, int32n ACExcitWireMode, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAIPosRVDTChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, float64n sensitivity, int32n sensitivityUnits, int32n voltageExcitSource, float64n voltageExcitVal, float64n voltageExcitFreq, int32n ACExcitWireMode, const char customScaleName[]);
// Function DAQmxCreateAIDeviceTempChan is obsolete and has been replaced by DAQmxCreateAITempBuiltInSensorChan
int32n __CFUNC     DAQmxCreateAIDeviceTempChan       (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n units);

int32n __CFUNC     DAQmxCreateTEDSAIVoltageChan      (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateTEDSAICurrentChan      (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, int32n shuntResistorLoc, float64n extShuntResistorVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateTEDSAIThrmcplChan      (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n cjcSource, float64n cjcVal, const char cjcChannel[]);
int32n __CFUNC     DAQmxCreateTEDSAIRTDChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n resistanceConfig, int32n currentExcitSource, float64n currentExcitVal);
int32n __CFUNC     DAQmxCreateTEDSAIThrmstrChanIex   (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n resistanceConfig, int32n currentExcitSource, float64n currentExcitVal);
int32n __CFUNC     DAQmxCreateTEDSAIThrmstrChanVex   (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n resistanceConfig, int32n voltageExcitSource, float64n voltageExcitVal, float64n r1);
int32n __CFUNC     DAQmxCreateTEDSAIResistanceChan   (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n resistanceConfig, int32n currentExcitSource, float64n currentExcitVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateTEDSAIStrainGageChan   (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n voltageExcitSource, float64n voltageExcitVal, float64n initialBridgeVoltage, float64n leadWireResistance, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateTEDSAIVoltageChanWithExcit (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, int32n voltageExcitSource, float64n voltageExcitVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateTEDSAIAccelChan        (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, float64n minVal, float64n maxVal, int32n units, int32n currentExcitSource, float64n currentExcitVal, const char customScaleName[]);

int32n __CFUNC     DAQmxCreateTEDSAIMicrophoneChan   (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n terminalConfig, int32n units, float64n maxSndPressLevel, int32n currentExcitSource, float64n currentExcitVal, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateTEDSAIPosLVDTChan      (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n voltageExcitSource, float64n voltageExcitVal, float64n voltageExcitFreq, int32n ACExcitWireMode, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateTEDSAIPosRVDTChan      (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n voltageExcitSource, float64n voltageExcitVal, float64n voltageExcitFreq, int32n ACExcitWireMode, const char customScaleName[]);

int32n __CFUNC     DAQmxCreateAOVoltageChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAOCurrentChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateAOFuncGenChan          (TaskHandle taskHandle, const char physicalChannel[], const char nameToAssignToChannel[], int32n type, float64n freq, float64n amplitude, float64n offset);

int32n __CFUNC     DAQmxCreateDIChan                 (TaskHandle taskHandle, const char lines[], const char nameToAssignToLines[], int32n lineGrouping);

int32n __CFUNC     DAQmxCreateDOChan                 (TaskHandle taskHandle, const char lines[], const char nameToAssignToLines[], int32n lineGrouping);

int32n __CFUNC     DAQmxCreateCIFreqChan             (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n edge, int32n measMethod, float64n measTime, uint32n divisor, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateCIPeriodChan           (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n edge, int32n measMethod, float64n measTime, uint32n divisor, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateCICountEdgesChan       (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], int32n edge, uint32n initialCount, int32n countDirection);
int32n __CFUNC     DAQmxCreateCIPulseWidthChan       (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n startingEdge, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateCISemiPeriodChan       (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateCITwoEdgeSepChan       (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], float64n minVal, float64n maxVal, int32n units, int32n firstEdge, int32n secondEdge, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateCILinEncoderChan       (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], int32n decodingType, bool32n ZidxEnable, float64n ZidxVal, int32n ZidxPhase, int32n units, float64n distPerPulse, float64n initialPos, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateCIAngEncoderChan       (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], int32n decodingType, bool32n ZidxEnable, float64n ZidxVal, int32n ZidxPhase, int32n units, uint32n pulsesPerRev, float64n initialAngle, const char customScaleName[]);
int32n __CFUNC     DAQmxCreateCIGPSTimestampChan     (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], int32n units, int32n syncMethod, const char customScaleName[]);

int32n __CFUNC     DAQmxCreateCOPulseChanFreq        (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], int32n units, int32n idleState, float64n initialDelay, float64n freq, float64n dutyCycle);
int32n __CFUNC     DAQmxCreateCOPulseChanTime        (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], int32n units, int32n idleState, float64n initialDelay, float64n lowTime, float64n highTime);
int32n __CFUNC     DAQmxCreateCOPulseChanTicks       (TaskHandle taskHandle, const char counter[], const char nameToAssignToChannel[], const char sourceTerminal[], int32n idleState, int32n initialDelay, int32n lowTicks, int32n highTicks);

int32n __CFUNC     DAQmxGetAIChanCalCalDate(TaskHandle taskHandle, const char channelName[], uint32n *year, uint32n *month, uint32n *day, uint32n *hour, uint32n *minute);
int32n __CFUNC     DAQmxSetAIChanCalCalDate(TaskHandle taskHandle, const char channelName[], uint32n year, uint32n month, uint32n day, uint32n hour, uint32n minute);
int32n __CFUNC     DAQmxGetAIChanCalExpDate(TaskHandle taskHandle, const char channelName[], uint32n *year, uint32n *month, uint32n *day, uint32n *hour, uint32n *minute);
int32n __CFUNC     DAQmxSetAIChanCalExpDate(TaskHandle taskHandle, const char channelName[], uint32n year, uint32n month, uint32n day, uint32n hour, uint32n minute);

int32n __CFUNC_C   DAQmxGetChanAttribute             (TaskHandle taskHandle, const char channel[], int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetChanAttribute             (TaskHandle taskHandle, const char channel[], int32n attribute, ...);
int32n __CFUNC     DAQmxResetChanAttribute           (TaskHandle taskHandle, const char channel[], int32n attribute);


/******************************************************/
/***                    Timing                      ***/
/******************************************************/


// (Analog/Counter Timing)
int32n __CFUNC     DAQmxCfgSampClkTiming          (TaskHandle taskHandle, const char source[], float64n rate, int32n activeEdge, int32n sampleMode, uInt64n sampsPerChan);
// (Digital Timing)
int32n __CFUNC     DAQmxCfgHandshakingTiming      (TaskHandle taskHandle, int32n sampleMode, uInt64n sampsPerChan);
// (Burst Import Clock Timing)
int32n __CFUNC     DAQmxCfgBurstHandshakingTimingImportClock(TaskHandle taskHandle, int32n sampleMode, uInt64n sampsPerChan, float64n sampleClkRate, const char sampleClkSrc[], int32n sampleClkActiveEdge, int32n pauseWhen, int32n readyEventActiveLevel);
// (Burst Export Clock Timing)
int32n __CFUNC     DAQmxCfgBurstHandshakingTimingExportClock(TaskHandle taskHandle, int32n sampleMode, uInt64n sampsPerChan, float64n sampleClkRate, const char sampleClkOutpTerm[], int32n sampleClkPulsePolarity, int32n pauseWhen, int32n readyEventActiveLevel);
int32n __CFUNC     DAQmxCfgChangeDetectionTiming  (TaskHandle taskHandle, const char risingEdgeChan[], const char fallingEdgeChan[], int32n sampleMode, uInt64n sampsPerChan);
// (Counter Timing)
int32n __CFUNC     DAQmxCfgImplicitTiming         (TaskHandle taskHandle, int32n sampleMode, uInt64n sampsPerChan);
// (Pipelined Sample Clock Timing)
int32n __CFUNC     DAQmxCfgPipelinedSampClkTiming (TaskHandle taskHandle, const char source[], float64n rate, int32n activeEdge, int32n sampleMode, uInt64n sampsPerChan);

int32n __CFUNC_C   DAQmxGetTimingAttribute        (TaskHandle taskHandle, int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetTimingAttribute        (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetTimingAttribute      (TaskHandle taskHandle, int32n attribute);

int32n __CFUNC_C   DAQmxGetTimingAttributeEx      (TaskHandle taskHandle, const char deviceNames[], int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetTimingAttributeEx      (TaskHandle taskHandle, const char deviceNames[], int32n attribute, ...);
int32n __CFUNC     DAQmxResetTimingAttributeEx    (TaskHandle taskHandle, const char deviceNames[], int32n attribute);


/******************************************************/
/***                  Triggering                    ***/
/******************************************************/


int32n __CFUNC     DAQmxDisableStartTrig      (TaskHandle taskHandle);
int32n __CFUNC     DAQmxCfgDigEdgeStartTrig   (TaskHandle taskHandle, const char triggerSource[], int32n triggerEdge);
int32n __CFUNC     DAQmxCfgAnlgEdgeStartTrig  (TaskHandle taskHandle, const char triggerSource[], int32n triggerSlope, float64n triggerLevel);
int32n __CFUNC     DAQmxCfgAnlgWindowStartTrig(TaskHandle taskHandle, const char triggerSource[], int32n triggerWhen, float64n windowTop, float64n windowBottom);
int32n __CFUNC     DAQmxCfgDigPatternStartTrig(TaskHandle taskHandle, const char triggerSource[], const char triggerPattern[], int32n triggerWhen);

int32n __CFUNC     DAQmxDisableRefTrig        (TaskHandle taskHandle);
int32n __CFUNC     DAQmxCfgDigEdgeRefTrig     (TaskHandle taskHandle, const char triggerSource[], int32n triggerEdge, uint32n pretriggerSamples);
int32n __CFUNC     DAQmxCfgAnlgEdgeRefTrig    (TaskHandle taskHandle, const char triggerSource[], int32n triggerSlope, float64n triggerLevel, uint32n pretriggerSamples);
int32n __CFUNC     DAQmxCfgAnlgWindowRefTrig  (TaskHandle taskHandle, const char triggerSource[], int32n triggerWhen, float64n windowTop, float64n windowBottom, uint32n pretriggerSamples);
int32n __CFUNC     DAQmxCfgDigPatternRefTrig  (TaskHandle taskHandle, const char triggerSource[], const char triggerPattern[], int32n triggerWhen, uint32n pretriggerSamples);

int32n __CFUNC     DAQmxDisableAdvTrig        (TaskHandle taskHandle);
int32n __CFUNC     DAQmxCfgDigEdgeAdvTrig     (TaskHandle taskHandle, const char triggerSource[], int32n triggerEdge);

int32n __CFUNC_C   DAQmxGetTrigAttribute      (TaskHandle taskHandle, int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetTrigAttribute      (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetTrigAttribute    (TaskHandle taskHandle, int32n attribute);

int32n __CFUNC     DAQmxSendSoftwareTrigger   (TaskHandle taskHandle, int32n triggerID);


/******************************************************/
/***                 Read Data                      ***/
/******************************************************/


int32n __CFUNC     DAQmxReadAnalogF64         (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, float64n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);
int32n __CFUNC     DAQmxReadAnalogScalarF64   (TaskHandle taskHandle, float64n timeout, float64n *value, bool32n *reserved);

int32n __CFUNC     DAQmxReadBinaryI16         (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, int16n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);

int32n __CFUNC     DAQmxReadBinaryU16         (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, uint16n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);

int32n __CFUNC     DAQmxReadBinaryI32         (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, int32n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);

int32n __CFUNC     DAQmxReadBinaryU32         (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, uint32n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);

int32n __CFUNC     DAQmxReadDigitalU8         (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, uInt8n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);
int32n __CFUNC     DAQmxReadDigitalU16        (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, uint16n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);
int32n __CFUNC     DAQmxReadDigitalU32        (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, uint32n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);
int32n __CFUNC     DAQmxReadDigitalScalarU32  (TaskHandle taskHandle, float64n timeout, uint32n *value, bool32n *reserved);
int32n __CFUNC	    DAQmxReadDigitalLines      (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, bool32n fillMode, uInt8n readArray[], uint32n arraySizeInBytes, int32n *sampsPerChanRead, int32n *numBytesPerSamp, bool32n *reserved);

int32n __CFUNC     DAQmxReadCounterF64        (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, float64n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);
int32n __CFUNC     DAQmxReadCounterU32        (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, uint32n readArray[], uint32n arraySizeInSamps, int32n *sampsPerChanRead, bool32n *reserved);
int32n __CFUNC     DAQmxReadCounterScalarF64  (TaskHandle taskHandle, float64n timeout, float64n *value, bool32n *reserved);
int32n __CFUNC     DAQmxReadCounterScalarU32  (TaskHandle taskHandle, float64n timeout, uint32n *value, bool32n *reserved);

int32n __CFUNC     DAQmxReadRaw               (TaskHandle taskHandle, int32n numSampsPerChan, float64n timeout, void *readArray, uint32n arraySizeInBytes, int32n *sampsRead, int32n *numBytesPerSamp, bool32n *reserved);

int32n __CFUNC     DAQmxGetNthTaskReadChannel (TaskHandle taskHandle, uint32n index, char buffer[], int32n bufferSize);

int32n __CFUNC_C   DAQmxGetReadAttribute      (TaskHandle taskHandle, int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetReadAttribute      (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetReadAttribute    (TaskHandle taskHandle, int32n attribute);


/******************************************************/
/***                 Write Data                     ***/
/******************************************************/


int32n __CFUNC     DAQmxWriteAnalogF64          (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const float64n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteAnalogScalarF64    (TaskHandle taskHandle, bool32n autoStart, float64n timeout, float64n value, bool32n *reserved);

int32n __CFUNC     DAQmxWriteBinaryI16          (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const int16n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteBinaryU16          (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const uint16n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteBinaryI32          (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const int32n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteBinaryU32          (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const uint32n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);

int32n __CFUNC     DAQmxWriteDigitalU8          (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const uInt8n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteDigitalU16         (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const uint16n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteDigitalU32         (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const uint32n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteDigitalScalarU32   (TaskHandle taskHandle, bool32n autoStart, float64n timeout, uint32n value, bool32n *reserved);
int32n __CFUNC     DAQmxWriteDigitalLines       (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const uInt8n writeArray[], int32n *sampsPerChanWritten, bool32n *reserved);

int32n __CFUNC     DAQmxWriteCtrFreq            (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const float64n frequency[], const float64n dutyCycle[], int32n *numSampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteCtrFreqScalar      (TaskHandle taskHandle, bool32n autoStart, float64n timeout, float64n frequency, float64n dutyCycle, bool32n *reserved);
int32n __CFUNC     DAQmxWriteCtrTime            (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const float64n highTime[], const float64n lowTime[], int32n *numSampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteCtrTimeScalar      (TaskHandle taskHandle, bool32n autoStart, float64n timeout, float64n highTime, float64n lowTime, bool32n *reserved);
int32n __CFUNC     DAQmxWriteCtrTicks           (TaskHandle taskHandle, int32n numSampsPerChan, bool32n autoStart, float64n timeout, bool32n dataLayout, const uint32n highTicks[], const uint32n lowTicks[], int32n *numSampsPerChanWritten, bool32n *reserved);
int32n __CFUNC     DAQmxWriteCtrTicksScalar     (TaskHandle taskHandle, bool32n autoStart, float64n timeout, uint32n highTicks, uint32n lowTicks, bool32n *reserved);

int32n __CFUNC     DAQmxWriteRaw                (TaskHandle taskHandle, int32n numSamps, bool32n autoStart, float64n timeout, const void *writeArray, int32n *sampsPerChanWritten, bool32n *reserved);

int32n __CFUNC_C   DAQmxGetWriteAttribute       (TaskHandle taskHandle, int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetWriteAttribute       (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetWriteAttribute     (TaskHandle taskHandle, int32n attribute);


/******************************************************/
/***               Events & Signals                 ***/
/******************************************************/

// Terminology:  For hardware, "signals" comprise "clocks," "triggers," and (output) "events".
// Software signals or events are not presently supported.

// For possible values for parameter signalID see value set Signal in Values section above.
int32n __CFUNC     DAQmxExportSignal                (TaskHandle taskHandle, int32n signalID, const char outputTerminal[]);

int32n __CFUNC_C   DAQmxGetExportedSignalAttribute  (TaskHandle taskHandle, int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetExportedSignalAttribute  (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetExportedSignalAttribute(TaskHandle taskHandle, int32n attribute);


/******************************************************/
/***              Scale Configurations              ***/
/******************************************************/


int32n __CFUNC     DAQmxCreateLinScale             (const char name[], float64n slope, float64n yIntercept, int32n preScaledUnits, const char scaledUnits[]);
int32n __CFUNC     DAQmxCreateMapScale             (const char name[], float64n prescaledMin, float64n prescaledMax, float64n scaledMin, float64n scaledMax, int32n preScaledUnits, const char scaledUnits[]);
int32n __CFUNC     DAQmxCreatePolynomialScale      (const char name[], const float64n forwardCoeffs[], uint32n numForwardCoeffsIn, const float64n reverseCoeffs[], uint32n numReverseCoeffsIn, int32n preScaledUnits, const char scaledUnits[]);
int32n __CFUNC     DAQmxCreateTableScale           (const char name[], const float64n prescaledVals[], uint32n numPrescaledValsIn, const float64n scaledVals[], uint32n numScaledValsIn, int32n preScaledUnits, const char scaledUnits[]);
int32n __CFUNC     DAQmxCalculateReversePolyCoeff  (const float64n forwardCoeffs[], uint32n numForwardCoeffsIn, float64n minValX, float64n maxValX, int32n numPointsToCompute, int32n reversePolyOrder, float64n reverseCoeffs[]);

int32n __CFUNC_C   DAQmxGetScaleAttribute          (const char scaleName[], int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetScaleAttribute          (const char scaleName[], int32n attribute, ...);


/******************************************************/
/***             Buffer Configurations              ***/
/******************************************************/


int32n __CFUNC     DAQmxCfgInputBuffer      (TaskHandle taskHandle, uint32n numSampsPerChan);
int32n __CFUNC     DAQmxCfgOutputBuffer     (TaskHandle taskHandle, uint32n numSampsPerChan);

int32n __CFUNC_C   DAQmxGetBufferAttribute  (TaskHandle taskHandle, int32n attribute, void *value);
int32n __CFUNC_C   DAQmxSetBufferAttribute  (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetBufferAttribute(TaskHandle taskHandle, int32n attribute);

/******************************************************/
/***                Switch Functions                ***/
/******************************************************/


int32n __CFUNC     DAQmxSwitchCreateScanList      (const char scanList[], TaskHandle *taskHandle);

int32n __CFUNC     DAQmxSwitchConnect             (const char switchChannel1[], const char switchChannel2[], bool32n waitForSettling);
int32n __CFUNC     DAQmxSwitchConnectMulti        (const char connectionList[], bool32n waitForSettling);
int32n __CFUNC     DAQmxSwitchDisconnect          (const char switchChannel1[], const char switchChannel2[], bool32n waitForSettling);
int32n __CFUNC     DAQmxSwitchDisconnectMulti     (const char connectionList[], bool32n waitForSettling);
int32n __CFUNC     DAQmxSwitchDisconnectAll       (const char deviceName[], bool32n waitForSettling);

// Switch Topologies
#define DAQmx_Val_Switch_Topology_1127_1_Wire_64x1_Mux            "1127/1-Wire 64x1 Mux"              // 1127/1-Wire 64x1 Mux
#define DAQmx_Val_Switch_Topology_1127_2_Wire_32x1_Mux            "1127/2-Wire 32x1 Mux"              // 1127/2-Wire 32x1 Mux
#define DAQmx_Val_Switch_Topology_1127_2_Wire_4x8_Matrix          "1127/2-Wire 4x8 Matrix"            // 1127/2-Wire 4x8 Matrix
#define DAQmx_Val_Switch_Topology_1127_4_Wire_16x1_Mux            "1127/4-Wire 16x1 Mux"              // 1127/4-Wire 16x1 Mux
#define DAQmx_Val_Switch_Topology_1127_Independent                "1127/Independent"                  // 1127/Independent
#define DAQmx_Val_Switch_Topology_1128_1_Wire_64x1_Mux            "1128/1-Wire 64x1 Mux"              // 1128/1-Wire 64x1 Mux
#define DAQmx_Val_Switch_Topology_1128_2_Wire_32x1_Mux            "1128/2-Wire 32x1 Mux"              // 1128/2-Wire 32x1 Mux
#define DAQmx_Val_Switch_Topology_1128_2_Wire_4x8_Matrix          "1128/2-Wire 4x8 Matrix"            // 1128/2-Wire 4x8 Matrix
#define DAQmx_Val_Switch_Topology_1128_4_Wire_16x1_Mux            "1128/4-Wire 16x1 Mux"              // 1128/4-Wire 16x1 Mux
#define DAQmx_Val_Switch_Topology_1128_Independent                "1128/Independent"                  // 1128/Independent
#define DAQmx_Val_Switch_Topology_1129_2_Wire_16x16_Matrix        "1129/2-Wire 16x16 Matrix"          // 1129/2-Wire 16x16 Matrix
#define DAQmx_Val_Switch_Topology_1129_2_Wire_8x32_Matrix         "1129/2-Wire 8x32 Matrix"           // 1129/2-Wire 8x32 Matrix
#define DAQmx_Val_Switch_Topology_1129_2_Wire_4x64_Matrix         "1129/2-Wire 4x64 Matrix"           // 1129/2-Wire 4x64 Matrix
#define DAQmx_Val_Switch_Topology_1129_2_Wire_Dual_8x16_Matrix    "1129/2-Wire Dual 8x16 Matrix"      // 1129/2-Wire Dual 8x16 Matrix
#define DAQmx_Val_Switch_Topology_1129_2_Wire_Dual_4x32_Matrix    "1129/2-Wire Dual 4x32 Matrix"      // 1129/2-Wire Dual 4x32 Matrix
#define DAQmx_Val_Switch_Topology_1129_2_Wire_Quad_4x16_Matrix    "1129/2-Wire Quad 4x16 Matrix"      // 1129/2-Wire Quad 4x16 Matrix
#define DAQmx_Val_Switch_Topology_1130_1_Wire_256x1_Mux           "1130/1-Wire 256x1 Mux"             // 1130/1-Wire 256x1 Mux
#define DAQmx_Val_Switch_Topology_1130_1_Wire_Dual_128x1_Mux      "1130/1-Wire Dual 128x1 Mux"        // 1130/1-Wire Dual 128x1 Mux
#define DAQmx_Val_Switch_Topology_1130_2_Wire_128x1_Mux           "1130/2-Wire 128x1 Mux"             // 1130/2-Wire 128x1 Mux
#define DAQmx_Val_Switch_Topology_1130_4_Wire_64x1_Mux            "1130/4-Wire 64x1 Mux"              // 1130/4-Wire 64x1 Mux
#define DAQmx_Val_Switch_Topology_1130_1_Wire_4x64_Matrix         "1130/1-Wire 4x64 Matrix"           // 1130/1-Wire 4x64 Matrix
#define DAQmx_Val_Switch_Topology_1130_1_Wire_8x32_Matrix         "1130/1-Wire 8x32 Matrix"           // 1130/1-Wire 8x32 Matrix
#define DAQmx_Val_Switch_Topology_1130_1_Wire_Octal_32x1_Mux      "1130/1-Wire Octal 32x1 Mux"        // 1130/1-Wire Octal 32x1 Mux
#define DAQmx_Val_Switch_Topology_1130_1_Wire_Quad_64x1_Mux       "1130/1-Wire Quad 64x1 Mux"         // 1130/1-Wire Quad 64x1 Mux
#define DAQmx_Val_Switch_Topology_1130_1_Wire_Sixteen_16x1_Mux    "1130/1-Wire Sixteen 16x1 Mux"      // 1130/1-Wire Sixteen 16x1 Mux
#define DAQmx_Val_Switch_Topology_1130_2_Wire_4x32_Matrix         "1130/2-Wire 4x32 Matrix"           // 1130/2-Wire 4x32 Matrix
#define DAQmx_Val_Switch_Topology_1130_2_Wire_Octal_16x1_Mux      "1130/2-Wire Octal 16x1 Mux"        // 1130/2-Wire Octal 16x1 Mux
#define DAQmx_Val_Switch_Topology_1130_2_Wire_Quad_32x1_Mux       "1130/2-Wire Quad 32x1 Mux"         // 1130/2-Wire Quad 32x1 Mux
#define DAQmx_Val_Switch_Topology_1130_4_Wire_Quad_16x1_Mux       "1130/4-Wire Quad 16x1 Mux"         // 1130/4-Wire Quad 16x1 Mux
#define DAQmx_Val_Switch_Topology_1130_Independent                "1130/Independent"                  // 1130/Independent
#define DAQmx_Val_Switch_Topology_1160_16_SPDT                    "1160/16-SPDT"                      // 1160/16-SPDT
#define DAQmx_Val_Switch_Topology_1161_8_SPDT                     "1161/8-SPDT"                       // 1161/8-SPDT
#define DAQmx_Val_Switch_Topology_1163R_Octal_4x1_Mux             "1163R/Octal 4x1 Mux"               // 1163R/Octal 4x1 Mux
#define DAQmx_Val_Switch_Topology_1166_32_SPDT                    "1166/32-SPDT"                      // 1166/32-SPDT
#define DAQmx_Val_Switch_Topology_1166_16_DPDT                    "1166/16-DPDT"                      // 1166/16-DPDT
#define DAQmx_Val_Switch_Topology_1167_Independent                "1167/Independent"                  // 1167/Independent
#define DAQmx_Val_Switch_Topology_1169_100_SPST                   "1169/100-SPST"                     // 1169/100-SPST
#define DAQmx_Val_Switch_Topology_1169_50_DPST                    "1169/50-DPST"                      // 1169/50-DPST
#define DAQmx_Val_Switch_Topology_1175_1_Wire_196x1_Mux           "1175/1-Wire 196x1 Mux"             // 1175/1-Wire 196x1 Mux
#define DAQmx_Val_Switch_Topology_1175_2_Wire_98x1_Mux            "1175/2-Wire 98x1 Mux"              // 1175/2-Wire 98x1 Mux
#define DAQmx_Val_Switch_Topology_1175_2_Wire_95x1_Mux            "1175/2-Wire 95x1 Mux"              // 1175/2-Wire 95x1 Mux
#define DAQmx_Val_Switch_Topology_1190_Quad_4x1_Mux               "1190/Quad 4x1 Mux"                 // 1190/Quad 4x1 Mux
#define DAQmx_Val_Switch_Topology_1191_Quad_4x1_Mux               "1191/Quad 4x1 Mux"                 // 1191/Quad 4x1 Mux
#define DAQmx_Val_Switch_Topology_1192_8_SPDT                     "1192/8-SPDT"                       // 1192/8-SPDT
#define DAQmx_Val_Switch_Topology_1193_32x1_Mux                   "1193/32x1 Mux"                     // 1193/32x1 Mux
#define DAQmx_Val_Switch_Topology_1193_Dual_16x1_Mux              "1193/Dual 16x1 Mux"                // 1193/Dual 16x1 Mux
#define DAQmx_Val_Switch_Topology_1193_Quad_8x1_Mux               "1193/Quad 8x1 Mux"                 // 1193/Quad 8x1 Mux
#define DAQmx_Val_Switch_Topology_1193_16x1_Terminated_Mux        "1193/16x1 Terminated Mux"          // 1193/16x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_1193_Dual_8x1_Terminated_Mux    "1193/Dual 8x1 Terminated Mux"      // 1193/Dual 8x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_1193_Quad_4x1_Terminated_Mux    "1193/Quad 4x1 Terminated Mux"      // 1193/Quad 4x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_1193_Independent                "1193/Independent"                  // 1193/Independent
#define DAQmx_Val_Switch_Topology_1194_Quad_4x1_Mux               "1194/Quad 4x1 Mux"                 // 1194/Quad 4x1 Mux
#define DAQmx_Val_Switch_Topology_1195_Quad_4x1_Mux               "1195/Quad 4x1 Mux"                 // 1195/Quad 4x1 Mux
#define DAQmx_Val_Switch_Topology_2501_1_Wire_48x1_Mux            "2501/1-Wire 48x1 Mux"              // 2501/1-Wire 48x1 Mux
#define DAQmx_Val_Switch_Topology_2501_1_Wire_48x1_Amplified_Mux  "2501/1-Wire 48x1 Amplified Mux"    // 2501/1-Wire 48x1 Amplified Mux
#define DAQmx_Val_Switch_Topology_2501_2_Wire_24x1_Mux            "2501/2-Wire 24x1 Mux"              // 2501/2-Wire 24x1 Mux
#define DAQmx_Val_Switch_Topology_2501_2_Wire_24x1_Amplified_Mux  "2501/2-Wire 24x1 Amplified Mux"    // 2501/2-Wire 24x1 Amplified Mux
#define DAQmx_Val_Switch_Topology_2501_2_Wire_Dual_12x1_Mux       "2501/2-Wire Dual 12x1 Mux"         // 2501/2-Wire Dual 12x1 Mux
#define DAQmx_Val_Switch_Topology_2501_2_Wire_Quad_6x1_Mux        "2501/2-Wire Quad 6x1 Mux"          // 2501/2-Wire Quad 6x1 Mux
#define DAQmx_Val_Switch_Topology_2501_2_Wire_4x6_Matrix          "2501/2-Wire 4x6 Matrix"            // 2501/2-Wire 4x6 Matrix
#define DAQmx_Val_Switch_Topology_2501_4_Wire_12x1_Mux            "2501/4-Wire 12x1 Mux"              // 2501/4-Wire 12x1 Mux
#define DAQmx_Val_Switch_Topology_2503_1_Wire_48x1_Mux            "2503/1-Wire 48x1 Mux"              // 2503/1-Wire 48x1 Mux
#define DAQmx_Val_Switch_Topology_2503_2_Wire_24x1_Mux            "2503/2-Wire 24x1 Mux"              // 2503/2-Wire 24x1 Mux
#define DAQmx_Val_Switch_Topology_2503_2_Wire_Dual_12x1_Mux       "2503/2-Wire Dual 12x1 Mux"         // 2503/2-Wire Dual 12x1 Mux
#define DAQmx_Val_Switch_Topology_2503_2_Wire_Quad_6x1_Mux        "2503/2-Wire Quad 6x1 Mux"          // 2503/2-Wire Quad 6x1 Mux
#define DAQmx_Val_Switch_Topology_2503_2_Wire_4x6_Matrix          "2503/2-Wire 4x6 Matrix"            // 2503/2-Wire 4x6 Matrix
#define DAQmx_Val_Switch_Topology_2503_4_Wire_12x1_Mux            "2503/4-Wire 12x1 Mux"              // 2503/4-Wire 12x1 Mux
#define DAQmx_Val_Switch_Topology_2527_1_Wire_64x1_Mux            "2527/1-Wire 64x1 Mux"              // 2527/1-Wire 64x1 Mux
#define DAQmx_Val_Switch_Topology_2527_1_Wire_Dual_32x1_Mux       "2527/1-Wire Dual 32x1 Mux"         // 2527/1-Wire Dual 32x1 Mux
#define DAQmx_Val_Switch_Topology_2527_2_Wire_32x1_Mux            "2527/2-Wire 32x1 Mux"              // 2527/2-Wire 32x1 Mux
#define DAQmx_Val_Switch_Topology_2527_2_Wire_Dual_16x1_Mux       "2527/2-Wire Dual 16x1 Mux"         // 2527/2-Wire Dual 16x1 Mux
#define DAQmx_Val_Switch_Topology_2527_4_Wire_16x1_Mux            "2527/4-Wire 16x1 Mux"              // 2527/4-Wire 16x1 Mux
#define DAQmx_Val_Switch_Topology_2527_Independent                "2527/Independent"                  // 2527/Independent
#define DAQmx_Val_Switch_Topology_2529_2_Wire_8x16_Matrix         "2529/2-Wire 8x16 Matrix"           // 2529/2-Wire 8x16 Matrix
#define DAQmx_Val_Switch_Topology_2529_2_Wire_4x32_Matrix         "2529/2-Wire 4x32 Matrix"           // 2529/2-Wire 4x32 Matrix
#define DAQmx_Val_Switch_Topology_2529_2_Wire_Dual_4x16_Matrix    "2529/2-Wire Dual 4x16 Matrix"      // 2529/2-Wire Dual 4x16 Matrix
#define DAQmx_Val_Switch_Topology_2530_1_Wire_128x1_Mux           "2530/1-Wire 128x1 Mux"             // 2530/1-Wire 128x1 Mux
#define DAQmx_Val_Switch_Topology_2530_1_Wire_Dual_64x1_Mux       "2530/1-Wire Dual 64x1 Mux"         // 2530/1-Wire Dual 64x1 Mux
#define DAQmx_Val_Switch_Topology_2530_2_Wire_64x1_Mux            "2530/2-Wire 64x1 Mux"              // 2530/2-Wire 64x1 Mux
#define DAQmx_Val_Switch_Topology_2530_4_Wire_32x1_Mux            "2530/4-Wire 32x1 Mux"              // 2530/4-Wire 32x1 Mux
#define DAQmx_Val_Switch_Topology_2530_1_Wire_4x32_Matrix         "2530/1-Wire 4x32 Matrix"           // 2530/1-Wire 4x32 Matrix
#define DAQmx_Val_Switch_Topology_2530_1_Wire_8x16_Matrix         "2530/1-Wire 8x16 Matrix"           // 2530/1-Wire 8x16 Matrix
#define DAQmx_Val_Switch_Topology_2530_1_Wire_Octal_16x1_Mux      "2530/1-Wire Octal 16x1 Mux"        // 2530/1-Wire Octal 16x1 Mux
#define DAQmx_Val_Switch_Topology_2530_1_Wire_Quad_32x1_Mux       "2530/1-Wire Quad 32x1 Mux"         // 2530/1-Wire Quad 32x1 Mux
#define DAQmx_Val_Switch_Topology_2530_2_Wire_4x16_Matrix         "2530/2-Wire 4x16 Matrix"           // 2530/2-Wire 4x16 Matrix
#define DAQmx_Val_Switch_Topology_2530_2_Wire_Dual_32x1_Mux       "2530/2-Wire Dual 32x1 Mux"         // 2530/2-Wire Dual 32x1 Mux
#define DAQmx_Val_Switch_Topology_2530_2_Wire_Quad_16x1_Mux       "2530/2-Wire Quad 16x1 Mux"         // 2530/2-Wire Quad 16x1 Mux
#define DAQmx_Val_Switch_Topology_2530_4_Wire_Dual_16x1_Mux       "2530/4-Wire Dual 16x1 Mux"         // 2530/4-Wire Dual 16x1 Mux
#define DAQmx_Val_Switch_Topology_2530_Independent                "2530/Independent"                  // 2530/Independent
#define DAQmx_Val_Switch_Topology_2532_1_Wire_16x32_Matrix        "2532/1-Wire 16x32 Matrix"          // 2532/1-Wire 16x32 Matrix
#define DAQmx_Val_Switch_Topology_2532_1_Wire_4x128_Matrix        "2532/1-Wire 4x128 Matrix"          // 2532/1-Wire 4x128 Matrix
#define DAQmx_Val_Switch_Topology_2532_1_Wire_8x64_Matrix         "2532/1-Wire 8x64 Matrix"           // 2532/1-Wire 8x64 Matrix
#define DAQmx_Val_Switch_Topology_2532_1_Wire_Dual_16x16_Matrix   "2532/1-Wire Dual 16x16 Matrix"     // 2532/1-Wire Dual 16x16 Matrix
#define DAQmx_Val_Switch_Topology_2532_1_Wire_Dual_4x64_Matrix    "2532/1-Wire Dual 4x64 Matrix"      // 2532/1-Wire Dual 4x64 Matrix
#define DAQmx_Val_Switch_Topology_2532_1_Wire_Dual_8x32_Matrix    "2532/1-Wire Dual 8x32 Matrix"      // 2532/1-Wire Dual 8x32 Matrix
#define DAQmx_Val_Switch_Topology_2532_1_Wire_Sixteen_2x16_Matrix "2532/1-Wire Sixteen 2x16 Matrix"   // 2532/1-Wire Sixteen 2x16 Matrix
#define DAQmx_Val_Switch_Topology_2532_2_Wire_16x16_Matrix        "2532/2-Wire 16x16 Matrix"          // 2532/2-Wire 16x16 Matrix
#define DAQmx_Val_Switch_Topology_2532_2_Wire_4x64_Matrix         "2532/2-Wire 4x64 Matrix"           // 2532/2-Wire 4x64 Matrix
#define DAQmx_Val_Switch_Topology_2532_2_Wire_8x32_Matrix         "2532/2-Wire 8x32 Matrix"           // 2532/2-Wire 8x32 Matrix
#define DAQmx_Val_Switch_Topology_2533_1_Wire_4x64_Matrix         "2533/1-Wire 4x64 Matrix"           // 2533/1-Wire 4x64 Matrix
#define DAQmx_Val_Switch_Topology_2534_1_Wire_8x32_Matrix         "2534/1-Wire 8x32 Matrix"           // 2534/1-Wire 8x32 Matrix
#define DAQmx_Val_Switch_Topology_2535_1_Wire_4x136_Matrix        "2535/1-Wire 4x136 Matrix"          // 2535/1-Wire 4x136 Matrix
#define DAQmx_Val_Switch_Topology_2536_1_Wire_8x68_Matrix         "2536/1-Wire 8x68 Matrix"           // 2536/1-Wire 8x68 Matrix
#define DAQmx_Val_Switch_Topology_2545_4x1_Terminated_Mux         "2545/4x1 Terminated Mux"           // 2545/4x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_2546_Dual_4x1_Mux               "2546/Dual 4x1 Mux"                 // 2546/Dual 4x1 Mux
#define DAQmx_Val_Switch_Topology_2547_8x1_Mux                    "2547/8x1 Mux"                      // 2547/8x1 Mux
#define DAQmx_Val_Switch_Topology_2548_4_SPDT                     "2548/4-SPDT"                       // 2548/4-SPDT
#define DAQmx_Val_Switch_Topology_2549_Terminated_2_SPDT          "2549/Terminated 2-SPDT"            // 2549/Terminated 2-SPDT
#define DAQmx_Val_Switch_Topology_2554_4x1_Mux                    "2554/4x1 Mux"                      // 2554/4x1 Mux
#define DAQmx_Val_Switch_Topology_2555_4x1_Terminated_Mux         "2555/4x1 Terminated Mux"           // 2555/4x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_2556_Dual_4x1_Mux               "2556/Dual 4x1 Mux"                 // 2556/Dual 4x1 Mux
#define DAQmx_Val_Switch_Topology_2557_8x1_Mux                    "2557/8x1 Mux"                      // 2557/8x1 Mux
#define DAQmx_Val_Switch_Topology_2558_4_SPDT                     "2558/4-SPDT"                       // 2558/4-SPDT
#define DAQmx_Val_Switch_Topology_2559_Terminated_2_SPDT          "2559/Terminated 2-SPDT"            // 2559/Terminated 2-SPDT
#define DAQmx_Val_Switch_Topology_2564_16_SPST                    "2564/16-SPST"                      // 2564/16-SPST
#define DAQmx_Val_Switch_Topology_2564_8_DPST                     "2564/8-DPST"                       // 2564/8-DPST
#define DAQmx_Val_Switch_Topology_2565_16_SPST                    "2565/16-SPST"                      // 2565/16-SPST
#define DAQmx_Val_Switch_Topology_2566_16_SPDT                    "2566/16-SPDT"                      // 2566/16-SPDT
#define DAQmx_Val_Switch_Topology_2566_8_DPDT                     "2566/8-DPDT"                       // 2566/8-DPDT
#define DAQmx_Val_Switch_Topology_2567_Independent                "2567/Independent"                  // 2567/Independent
#define DAQmx_Val_Switch_Topology_2568_31_SPST                    "2568/31-SPST"                      // 2568/31-SPST
#define DAQmx_Val_Switch_Topology_2568_15_DPST                    "2568/15-DPST"                      // 2568/15-DPST
#define DAQmx_Val_Switch_Topology_2569_100_SPST                   "2569/100-SPST"                     // 2569/100-SPST
#define DAQmx_Val_Switch_Topology_2569_50_DPST                    "2569/50-DPST"                      // 2569/50-DPST
#define DAQmx_Val_Switch_Topology_2570_40_SPDT                    "2570/40-SPDT"                      // 2570/40-SPDT
#define DAQmx_Val_Switch_Topology_2570_20_DPDT                    "2570/20-DPDT"                      // 2570/20-DPDT
#define DAQmx_Val_Switch_Topology_2575_1_Wire_196x1_Mux           "2575/1-Wire 196x1 Mux"             // 2575/1-Wire 196x1 Mux
#define DAQmx_Val_Switch_Topology_2575_2_Wire_98x1_Mux            "2575/2-Wire 98x1 Mux"              // 2575/2-Wire 98x1 Mux
#define DAQmx_Val_Switch_Topology_2575_2_Wire_95x1_Mux            "2575/2-Wire 95x1 Mux"              // 2575/2-Wire 95x1 Mux
#define DAQmx_Val_Switch_Topology_2576_2_Wire_64x1_Mux            "2576/2-Wire 64x1 Mux"              // 2576/2-Wire 64x1 Mux
#define DAQmx_Val_Switch_Topology_2576_2_Wire_Dual_32x1_Mux       "2576/2-Wire Dual 32x1 Mux"         // 2576/2-Wire Dual 32x1 Mux
#define DAQmx_Val_Switch_Topology_2576_2_Wire_Octal_8x1_Mux       "2576/2-Wire Octal 8x1 Mux"         // 2576/2-Wire Octal 8x1 Mux
#define DAQmx_Val_Switch_Topology_2576_2_Wire_Quad_16x1_Mux       "2576/2-Wire Quad 16x1 Mux"         // 2576/2-Wire Quad 16x1 Mux
#define DAQmx_Val_Switch_Topology_2576_2_Wire_Sixteen_4x1_Mux     "2576/2-Wire Sixteen 4x1 Mux"       // 2576/2-Wire Sixteen 4x1 Mux
#define DAQmx_Val_Switch_Topology_2576_Independent                "2576/Independent"                  // 2576/Independent
#define DAQmx_Val_Switch_Topology_2584_1_Wire_12x1_Mux            "2584/1-Wire 12x1 Mux"              // 2584/1-Wire 12x1 Mux
#define DAQmx_Val_Switch_Topology_2584_1_Wire_Dual_6x1_Mux        "2584/1-Wire Dual 6x1 Mux"          // 2584/1-Wire Dual 6x1 Mux
#define DAQmx_Val_Switch_Topology_2584_2_Wire_6x1_Mux             "2584/2-Wire 6x1 Mux"               // 2584/2-Wire 6x1 Mux
#define DAQmx_Val_Switch_Topology_2584_Independent                "2584/Independent"                  // 2584/Independent
#define DAQmx_Val_Switch_Topology_2585_1_Wire_10x1_Mux            "2585/1-Wire 10x1 Mux"              // 2585/1-Wire 10x1 Mux
#define DAQmx_Val_Switch_Topology_2586_10_SPST                    "2586/10-SPST"                      // 2586/10-SPST
#define DAQmx_Val_Switch_Topology_2586_5_DPST                     "2586/5-DPST"                       // 2586/5-DPST
#define DAQmx_Val_Switch_Topology_2590_4x1_Mux                    "2590/4x1 Mux"                      // 2590/4x1 Mux
#define DAQmx_Val_Switch_Topology_2591_4x1_Mux                    "2591/4x1 Mux"                      // 2591/4x1 Mux
#define DAQmx_Val_Switch_Topology_2593_16x1_Mux                   "2593/16x1 Mux"                     // 2593/16x1 Mux
#define DAQmx_Val_Switch_Topology_2593_Dual_8x1_Mux               "2593/Dual 8x1 Mux"                 // 2593/Dual 8x1 Mux
#define DAQmx_Val_Switch_Topology_2593_8x1_Terminated_Mux         "2593/8x1 Terminated Mux"           // 2593/8x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_2593_Dual_4x1_Terminated_Mux    "2593/Dual 4x1 Terminated Mux"      // 2593/Dual 4x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_2593_Independent                "2593/Independent"                  // 2593/Independent
#define DAQmx_Val_Switch_Topology_2594_4x1_Mux                    "2594/4x1 Mux"                      // 2594/4x1 Mux
#define DAQmx_Val_Switch_Topology_2595_4x1_Mux                    "2595/4x1 Mux"                      // 2595/4x1 Mux
#define DAQmx_Val_Switch_Topology_2596_Dual_6x1_Mux               "2596/Dual 6x1 Mux"                 // 2596/Dual 6x1 Mux
#define DAQmx_Val_Switch_Topology_2597_6x1_Terminated_Mux         "2597/6x1 Terminated Mux"           // 2597/6x1 Terminated Mux
#define DAQmx_Val_Switch_Topology_2598_Dual_Transfer              "2598/Dual Transfer"                // 2598/Dual Transfer
#define DAQmx_Val_Switch_Topology_2599_2_SPDT                     "2599/2-SPDT"                       // 2599/2-SPDT

int32n __CFUNC     DAQmxSwitchSetTopologyAndReset (const char deviceName[], const char newTopology[]);

// For possible values of the output parameter pathStatus see value set SwitchPathType in Values section above.
int32n __CFUNC     DAQmxSwitchFindPath            (const char switchChannel1[], const char switchChannel2[], char path[], uint32n pathBufferSize, int32n *pathStatus);

int32n __CFUNC     DAQmxSwitchOpenRelays          (const char relayList[], bool32n waitForSettling);
int32n __CFUNC     DAQmxSwitchCloseRelays         (const char relayList[], bool32n waitForSettling);

int32n __CFUNC     DAQmxSwitchGetSingleRelayCount (const char relayName[], uint32n *count);
int32n __CFUNC     DAQmxSwitchGetMultiRelayCount  (const char relayList[], uint32n count[], uint32n countArraySize, uint32n *numRelayCountsRead);
// For possible values of the output parameter relayPos see value set RelayPos in Values section above.
int32n __CFUNC     DAQmxSwitchGetSingleRelayPos   (const char relayName[], uint32n *relayPos);
// For possible values in the output array relayPos see value set RelayPos in Values section above.
int32n __CFUNC     DAQmxSwitchGetMultiRelayPos    (const char relayList[], uint32n relayPos[], uint32n relayPosArraySize, uint32n *numRelayPossRead);

int32n __CFUNC     DAQmxSwitchWaitForSettling     (const char deviceName[]);

int32n __CFUNC_C   DAQmxGetSwitchChanAttribute    (const char switchChannelName[], int32n attribute, void *value);
int32n __CFUNC_C   DAQmxSetSwitchChanAttribute    (const char switchChannelName[], int32n attribute, ...);

int32n __CFUNC_C   DAQmxGetSwitchDeviceAttribute  (const char deviceName[], int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetSwitchDeviceAttribute  (const char deviceName[], int32n attribute, ...);

int32n __CFUNC_C   DAQmxGetSwitchScanAttribute    (TaskHandle taskHandle, int32n attribute, void *value);
int32n __CFUNC_C   DAQmxSetSwitchScanAttribute    (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetSwitchScanAttribute  (TaskHandle taskHandle, int32n attribute);


/******************************************************/
/***                Signal Routing                  ***/
/******************************************************/


int32n __CFUNC     DAQmxConnectTerms         (const char sourceTerminal[], const char destinationTerminal[], int32n signalModifiers);
int32n __CFUNC     DAQmxDisconnectTerms      (const char sourceTerminal[], const char destinationTerminal[]);
int32n __CFUNC     DAQmxTristateOutputTerm   (const char outputTerminal[]);


/******************************************************/
/***                Device Control                  ***/
/******************************************************/


int32n __CFUNC     DAQmxResetDevice              (const char deviceName[]);

int32n __CFUNC_C   DAQmxGetDeviceAttribute       (const char deviceName[], int32n attribute, void *value, ...);

/******************************************************/
/***              Watchdog Timer                    ***/
/******************************************************/


int32n __CFUNC_C   DAQmxCreateWatchdogTimerTask    (const char deviceName[], const char taskName[], TaskHandle *taskHandle, float64n timeout, const char lines[], int32n expState, ...);
int32n __CFUNC     DAQmxControlWatchdogTask        (TaskHandle taskHandle, int32n action);

int32n __CFUNC_C  DAQmxGetWatchdogAttribute        (TaskHandle taskHandle, const char lines[], int32n attribute, void *value, ...);
int32n __CFUNC_C  DAQmxSetWatchdogAttribute        (TaskHandle taskHandle, const char lines[], int32n attribute, ...);
int32n __CFUNC    DAQmxResetWatchdogAttribute      (TaskHandle taskHandle, const char lines[], int32n attribute);


/******************************************************/
/***                 Calibration                    ***/
/******************************************************/


int32n __CFUNC     DAQmxSelfCal                    (const char deviceName[]);
int32n __CFUNC     DAQmxPerformBridgeOffsetNullingCal(TaskHandle taskHandle, const char channel[]);
int32n __CFUNC     DAQmxPerformBridgeOffsetNullingCalEx(TaskHandle taskHandle, const char channel[], bool32n skipUnsupportedChannels);
int32n __CFUNC     DAQmxPerformStrainShuntCal      (TaskHandle taskHandle, const char channel[], float64n shuntResistorValue, int32n shuntResistorLocation, bool32n skipUnsupportedChannels);
int32n __CFUNC     DAQmxPerformBridgeShuntCal      (TaskHandle taskHandle, const char channel[], float64n shuntResistorValue, int32n shuntResistorLocation, float64n bridgeResistance, bool32n skipUnsupportedChannels);
int32n __CFUNC     DAQmxGetSelfCalLastDateAndTime  (const char deviceName[], uint32n *year, uint32n *month, uint32n *day, uint32n *hour, uint32n *minute);
int32n __CFUNC     DAQmxGetExtCalLastDateAndTime   (const char deviceName[], uint32n *year, uint32n *month, uint32n *day, uint32n *hour, uint32n *minute);
int32n __CFUNC     DAQmxRestoreLastExtCalConst     (const char deviceName[]);

int32n __CFUNC     DAQmxESeriesCalAdjust           (CalHandle calHandle, float64n referenceVoltage);
int32n __CFUNC     DAQmxMSeriesCalAdjust           (CalHandle calHandle, float64n referenceVoltage);
int32n __CFUNC     DAQmxSSeriesCalAdjust           (CalHandle calHandle, float64n referenceVoltage);
int32n __CFUNC     DAQmxSCBaseboardCalAdjust       (CalHandle calHandle, float64n referenceVoltage);
int32n __CFUNC     DAQmxAOSeriesCalAdjust          (CalHandle calHandle, float64n referenceVoltage);

int32n __CFUNC     DAQmxDeviceSupportsCal          (const char deviceName[], bool32n *calSupported);

int32n __CFUNC_C   DAQmxGetCalInfoAttribute        (const char deviceName[], int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetCalInfoAttribute        (const char deviceName[], int32n attribute, ...);

int32n __CFUNC     DAQmxInitExtCal                 (const char deviceName[], const char password[], CalHandle *calHandle);
int32n __CFUNC     DAQmxCloseExtCal                (CalHandle calHandle, int32n action);
int32n __CFUNC     DAQmxChangeExtCalPassword       (const char deviceName[], const char password[], const char newPassword[]);

int32n __CFUNC     DAQmxAdjustDSAAICal             (CalHandle calHandle, float64n referenceVoltage);
int32n __CFUNC     DAQmxAdjustDSAAOCal             (CalHandle calHandle, uint32n channel, float64n requestedLowVoltage, float64n actualLowVoltage, float64n requestedHighVoltage, float64n actualHighVoltage, float64n gainSetting);
int32n __CFUNC     DAQmxAdjustDSATimebaseCal       (CalHandle calHandle, float64n referenceFrequency);

int32n __CFUNC     DAQmxAdjust4204Cal              (CalHandle calHandle, const char channelNames[], float64n lowPassFreq, bool32n trackHoldEnabled, float64n inputVal);
int32n __CFUNC     DAQmxAdjust4220Cal              (CalHandle calHandle, const char channelNames[], float64n gain, float64n inputVal);
int32n __CFUNC     DAQmxAdjust4224Cal              (CalHandle calHandle, const char channelNames[], float64n gain, float64n inputVal);
// Note: This function is obsolete and now always returns zero.
int32n __CFUNC     DAQmxAdjust4225Cal              (CalHandle calHandle, const char channelNames[], float64n gain, float64n inputVal);

int32n __CFUNC     DAQmxSetup1102Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1102Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1104Cal               (CalHandle calHandle, const char channelName[]);
int32n __CFUNC     DAQmxAdjust1104Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1112Cal               (CalHandle calHandle, const char channelName[]);
int32n __CFUNC     DAQmxAdjust1112Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1122Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1122Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1124Cal               (CalHandle calHandle, const char channelName[], int32n range, uint32n dacValue);
int32n __CFUNC     DAQmxAdjust1124Cal              (CalHandle calHandle, float64n measOutput);

int32n __CFUNC     DAQmxSetup1125Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1125Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1126Cal               (CalHandle calHandle, const char channelName[], float64n upperFreqLimit);
int32n __CFUNC     DAQmxAdjust1126Cal              (CalHandle calHandle, float64n refFreq, float64n measOutput);

int32n __CFUNC     DAQmxSetup1141Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1141Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);
int32n __CFUNC     DAQmxSetup1142Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1142Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);
int32n __CFUNC     DAQmxSetup1143Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1143Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);


int32n __CFUNC     DAQmxSetup1502Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1502Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1503Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1503Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);
int32n __CFUNC     DAQmxAdjust1503CurrentCal       (CalHandle calHandle, const char channelName[], float64n measCurrent);

int32n __CFUNC     DAQmxSetup1520Cal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust1520Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1521Cal               (CalHandle calHandle, const char channelName[]);
int32n __CFUNC     DAQmxAdjust1521Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup153xCal               (CalHandle calHandle, const char channelName[], float64n gain);
int32n __CFUNC     DAQmxAdjust153xCal              (CalHandle calHandle, float64n refVoltage, float64n measOutput);

int32n __CFUNC     DAQmxSetup1540Cal               (CalHandle calHandle, const char channelName[], float64n excitationVoltage, float64n excitationFreq);
int32n __CFUNC     DAQmxAdjust1540Cal              (CalHandle calHandle, float64n refVoltage, float64n measOutput, int32n inputCalSource);


/******************************************************/
/***                     TEDS                       ***/
/******************************************************/

int32n __CFUNC     DAQmxConfigureTEDS              (const char physicalChannel[], const char filePath[]);
int32n __CFUNC     DAQmxClearTEDS                  (const char physicalChannel[]);
int32n __CFUNC     DAQmxWriteToTEDSFromArray       (const char physicalChannel[], const uInt8n bitStream[], uint32n arraySize, int32n basicTEDSOptions);
int32n __CFUNC     DAQmxWriteToTEDSFromFile        (const char physicalChannel[], const char filePath[], int32n basicTEDSOptions);
int32n __CFUNC_C   DAQmxGetPhysicalChanAttribute   (const char physicalChannel[], int32n attribute, void *value, ...);


/******************************************************/
/***                  Real-Time                     ***/
/******************************************************/

int32n __CFUNC     DAQmxWaitForNextSampleClock(TaskHandle taskHandle, float64n timeout, bool32n *isLate);
int32n __CFUNC_C   DAQmxGetRealTimeAttribute  (TaskHandle taskHandle, int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetRealTimeAttribute  (TaskHandle taskHandle, int32n attribute, ...);
int32n __CFUNC     DAQmxResetRealTimeAttribute(TaskHandle taskHandle, int32n attribute);

// Note: This function is obsolete and now always returns zero.
bool32n __CFUNC    DAQmxIsReadOrWriteLate     (int32n errorCode);


/******************************************************/
/***                   Storage                      ***/
/******************************************************/

int32n __CFUNC     DAQmxSaveTask                    (TaskHandle taskHandle, const char saveAs[], const char author[], uint32n options);
int32n __CFUNC     DAQmxSaveGlobalChan              (TaskHandle taskHandle, const char channelName[], const char saveAs[], const char author[], uint32n options);
int32n __CFUNC     DAQmxSaveScale                   (const char scaleName[], const char saveAs[], const char author[], uint32n options);
int32n __CFUNC     DAQmxDeleteSavedTask             (const char taskName[]);
int32n __CFUNC     DAQmxDeleteSavedGlobalChan       (const char channelName[]);
int32n __CFUNC     DAQmxDeleteSavedScale            (const char scaleName[]);

int32n __CFUNC_C   DAQmxGetPersistedTaskAttribute   (const char taskName[], int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxGetPersistedChanAttribute   (const char channel[], int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxGetPersistedScaleAttribute  (const char scaleName[], int32n attribute, void *value, ...);


/******************************************************/
/***              System Configuration              ***/
/******************************************************/

int32n __CFUNC_C   DAQmxGetSystemInfoAttribute (int32n attribute, void *value, ...);
int32n __CFUNC_C   DAQmxSetDigitalPowerUpStates(const char deviceName[], const char channelNames[], int32n state, ...);
int32n __CFUNC_C   DAQmxSetAnalogPowerUpStates(const char deviceName[], const char channelNames[], float64n state, int32n channelType, ...);
int32n __CFUNC     DAQmxSetDigitalLogicFamilyPowerUpState(const char deviceName[], int32n logicFamily);

/******************************************************/
/***                 Error Handling                 ***/
/******************************************************/


int32n __CFUNC     DAQmxGetErrorString       (int32n errorCode, char errorString[], uint32n bufferSize);
int32n __CFUNC     DAQmxGetExtendedErrorInfo (char errorString[], uint32n bufferSize);


/******************************************************************************
 *** NI-DAQmx Specific Attribute Get/Set/Reset Function Declarations **********
 ******************************************************************************/

//********** Buffer **********
//*** Set/Get functions for DAQmx_Buf_Input_BufSize ***
int32n __CFUNC DAQmxGetBufInputBufSize(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetBufInputBufSize(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetBufInputBufSize(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Buf_Input_OnbrdBufSize ***
int32n __CFUNC DAQmxGetBufInputOnbrdBufSize(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Buf_Output_BufSize ***
int32n __CFUNC DAQmxGetBufOutputBufSize(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetBufOutputBufSize(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetBufOutputBufSize(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Buf_Output_OnbrdBufSize ***
int32n __CFUNC DAQmxGetBufOutputOnbrdBufSize(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetBufOutputOnbrdBufSize(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetBufOutputOnbrdBufSize(TaskHandle taskHandle);

//********** Calibration Info **********
//*** Set/Get functions for DAQmx_SelfCal_Supported ***
int32n __CFUNC DAQmxGetSelfCalSupported(const char deviceName[], bool32n *data);
//*** Set/Get functions for DAQmx_SelfCal_LastTemp ***
int32n __CFUNC DAQmxGetSelfCalLastTemp(const char deviceName[], float64n *data);
//*** Set/Get functions for DAQmx_ExtCal_RecommendedInterval ***
int32n __CFUNC DAQmxGetExtCalRecommendedInterval(const char deviceName[], uint32n *data);
//*** Set/Get functions for DAQmx_ExtCal_LastTemp ***
int32n __CFUNC DAQmxGetExtCalLastTemp(const char deviceName[], float64n *data);
//*** Set/Get functions for DAQmx_Cal_UserDefinedInfo ***
int32n __CFUNC DAQmxGetCalUserDefinedInfo(const char deviceName[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCalUserDefinedInfo(const char deviceName[], const char *data);
//*** Set/Get functions for DAQmx_Cal_UserDefinedInfo_MaxSize ***
int32n __CFUNC DAQmxGetCalUserDefinedInfoMaxSize(const char deviceName[], uint32n *data);
//*** Set/Get functions for DAQmx_Cal_DevTemp ***
int32n __CFUNC DAQmxGetCalDevTemp(const char deviceName[], float64n *data);

//********** Channel **********
//*** Set/Get functions for DAQmx_AI_Max ***
int32n __CFUNC DAQmxGetAIMax(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIMax(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIMax(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Min ***
int32n __CFUNC DAQmxGetAIMin(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIMin(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIMin(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_CustomScaleName ***
int32n __CFUNC DAQmxGetAICustomScaleName(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAICustomScaleName(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetAICustomScaleName(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_MeasType ***
// Uses value set AIMeasurementType
int32n __CFUNC DAQmxGetAIMeasType(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_AI_Voltage_Units ***
// Uses value set VoltageUnits1
int32n __CFUNC DAQmxGetAIVoltageUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIVoltageUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIVoltageUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Voltage_dBRef ***
int32n __CFUNC DAQmxGetAIVoltagedBRef(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIVoltagedBRef(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIVoltagedBRef(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Voltage_ACRMS_Units ***
// Uses value set VoltageUnits1
int32n __CFUNC DAQmxGetAIVoltageACRMSUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIVoltageACRMSUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIVoltageACRMSUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Temp_Units ***
// Uses value set TemperatureUnits1
int32n __CFUNC DAQmxGetAITempUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAITempUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAITempUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmcpl_Type ***
// Uses value set ThermocoupleType1
int32n __CFUNC DAQmxGetAIThrmcplType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIThrmcplType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIThrmcplType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmcpl_ScaleType ***
// Uses value set ScaleType2
int32n __CFUNC DAQmxGetAIThrmcplScaleType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIThrmcplScaleType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIThrmcplScaleType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmcpl_CJCSrc ***
// Uses value set CJCSource1
int32n __CFUNC DAQmxGetAIThrmcplCJCSrc(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_AI_Thrmcpl_CJCVal ***
int32n __CFUNC DAQmxGetAIThrmcplCJCVal(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIThrmcplCJCVal(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIThrmcplCJCVal(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmcpl_CJCChan ***
int32n __CFUNC DAQmxGetAIThrmcplCJCChan(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_AI_RTD_Type ***
// Uses value set RTDType1
int32n __CFUNC DAQmxGetAIRTDType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIRTDType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIRTDType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RTD_R0 ***
int32n __CFUNC DAQmxGetAIRTDR0(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIRTDR0(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIRTDR0(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RTD_A ***
int32n __CFUNC DAQmxGetAIRTDA(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIRTDA(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIRTDA(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RTD_B ***
int32n __CFUNC DAQmxGetAIRTDB(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIRTDB(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIRTDB(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RTD_C ***
int32n __CFUNC DAQmxGetAIRTDC(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIRTDC(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIRTDC(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmstr_A ***
int32n __CFUNC DAQmxGetAIThrmstrA(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIThrmstrA(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIThrmstrA(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmstr_B ***
int32n __CFUNC DAQmxGetAIThrmstrB(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIThrmstrB(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIThrmstrB(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmstr_C ***
int32n __CFUNC DAQmxGetAIThrmstrC(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIThrmstrC(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIThrmstrC(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Thrmstr_R1 ***
int32n __CFUNC DAQmxGetAIThrmstrR1(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIThrmstrR1(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIThrmstrR1(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ForceReadFromChan ***
int32n __CFUNC DAQmxGetAIForceReadFromChan(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIForceReadFromChan(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIForceReadFromChan(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Current_Units ***
// Uses value set CurrentUnits1
int32n __CFUNC DAQmxGetAICurrentUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAICurrentUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAICurrentUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Current_ACRMS_Units ***
// Uses value set CurrentUnits1
int32n __CFUNC DAQmxGetAICurrentACRMSUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAICurrentACRMSUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAICurrentACRMSUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Strain_Units ***
// Uses value set StrainUnits1
int32n __CFUNC DAQmxGetAIStrainUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIStrainUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIStrainUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_StrainGage_GageFactor ***
int32n __CFUNC DAQmxGetAIStrainGageGageFactor(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIStrainGageGageFactor(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIStrainGageGageFactor(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_StrainGage_PoissonRatio ***
int32n __CFUNC DAQmxGetAIStrainGagePoissonRatio(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIStrainGagePoissonRatio(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIStrainGagePoissonRatio(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_StrainGage_Cfg ***
// Uses value set StrainGageBridgeType1
int32n __CFUNC DAQmxGetAIStrainGageCfg(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIStrainGageCfg(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIStrainGageCfg(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Resistance_Units ***
// Uses value set ResistanceUnits1
int32n __CFUNC DAQmxGetAIResistanceUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIResistanceUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIResistanceUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Freq_Units ***
// Uses value set FrequencyUnits
int32n __CFUNC DAQmxGetAIFreqUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIFreqUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIFreqUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Freq_ThreshVoltage ***
int32n __CFUNC DAQmxGetAIFreqThreshVoltage(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIFreqThreshVoltage(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIFreqThreshVoltage(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Freq_Hyst ***
int32n __CFUNC DAQmxGetAIFreqHyst(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIFreqHyst(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIFreqHyst(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_LVDT_Units ***
// Uses value set LengthUnits2
int32n __CFUNC DAQmxGetAILVDTUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAILVDTUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAILVDTUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_LVDT_Sensitivity ***
int32n __CFUNC DAQmxGetAILVDTSensitivity(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAILVDTSensitivity(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAILVDTSensitivity(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_LVDT_SensitivityUnits ***
// Uses value set LVDTSensitivityUnits1
int32n __CFUNC DAQmxGetAILVDTSensitivityUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAILVDTSensitivityUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAILVDTSensitivityUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RVDT_Units ***
// Uses value set AngleUnits1
int32n __CFUNC DAQmxGetAIRVDTUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIRVDTUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIRVDTUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RVDT_Sensitivity ***
int32n __CFUNC DAQmxGetAIRVDTSensitivity(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIRVDTSensitivity(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIRVDTSensitivity(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RVDT_SensitivityUnits ***
// Uses value set RVDTSensitivityUnits1
int32n __CFUNC DAQmxGetAIRVDTSensitivityUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIRVDTSensitivityUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIRVDTSensitivityUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_SoundPressure_MaxSoundPressureLvl ***
int32n __CFUNC DAQmxGetAISoundPressureMaxSoundPressureLvl(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAISoundPressureMaxSoundPressureLvl(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAISoundPressureMaxSoundPressureLvl(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_SoundPressure_Units ***
// Uses value set SoundPressureUnits1
int32n __CFUNC DAQmxGetAISoundPressureUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAISoundPressureUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAISoundPressureUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_SoundPressure_dBRef ***
int32n __CFUNC DAQmxGetAISoundPressuredBRef(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAISoundPressuredBRef(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAISoundPressuredBRef(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Microphone_Sensitivity ***
int32n __CFUNC DAQmxGetAIMicrophoneSensitivity(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIMicrophoneSensitivity(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIMicrophoneSensitivity(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Accel_Units ***
// Uses value set AccelUnits2
int32n __CFUNC DAQmxGetAIAccelUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIAccelUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIAccelUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Accel_dBRef ***
int32n __CFUNC DAQmxGetAIAcceldBRef(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIAcceldBRef(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIAcceldBRef(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Accel_Sensitivity ***
int32n __CFUNC DAQmxGetAIAccelSensitivity(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIAccelSensitivity(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIAccelSensitivity(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Accel_SensitivityUnits ***
// Uses value set AccelSensitivityUnits1
int32n __CFUNC DAQmxGetAIAccelSensitivityUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIAccelSensitivityUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIAccelSensitivityUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Is_TEDS ***
int32n __CFUNC DAQmxGetAIIsTEDS(TaskHandle taskHandle, const char channel[], bool32n *data);
//*** Set/Get functions for DAQmx_AI_TEDS_Units ***
int32n __CFUNC DAQmxGetAITEDSUnits(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_AI_Coupling ***
// Uses value set Coupling1
int32n __CFUNC DAQmxGetAICoupling(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAICoupling(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAICoupling(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Impedance ***
// Uses value set Impedance1
int32n __CFUNC DAQmxGetAIImpedance(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIImpedance(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIImpedance(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_TermCfg ***
// Uses value set InputTermCfg
int32n __CFUNC DAQmxGetAITermCfg(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAITermCfg(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAITermCfg(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_InputSrc ***
int32n __CFUNC DAQmxGetAIInputSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAIInputSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetAIInputSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ResistanceCfg ***
// Uses value set ResistanceConfiguration
int32n __CFUNC DAQmxGetAIResistanceCfg(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIResistanceCfg(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIResistanceCfg(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_LeadWireResistance ***
int32n __CFUNC DAQmxGetAILeadWireResistance(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAILeadWireResistance(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAILeadWireResistance(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_Cfg ***
// Uses value set BridgeConfiguration1
int32n __CFUNC DAQmxGetAIBridgeCfg(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIBridgeCfg(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIBridgeCfg(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_NomResistance ***
int32n __CFUNC DAQmxGetAIBridgeNomResistance(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIBridgeNomResistance(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIBridgeNomResistance(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_InitialVoltage ***
int32n __CFUNC DAQmxGetAIBridgeInitialVoltage(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIBridgeInitialVoltage(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIBridgeInitialVoltage(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_ShuntCal_Enable ***
int32n __CFUNC DAQmxGetAIBridgeShuntCalEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIBridgeShuntCalEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIBridgeShuntCalEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_ShuntCal_Select ***
// Uses value set ShuntCalSelect
int32n __CFUNC DAQmxGetAIBridgeShuntCalSelect(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIBridgeShuntCalSelect(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIBridgeShuntCalSelect(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_ShuntCal_GainAdjust ***
int32n __CFUNC DAQmxGetAIBridgeShuntCalGainAdjust(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIBridgeShuntCalGainAdjust(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIBridgeShuntCalGainAdjust(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_Balance_CoarsePot ***
int32n __CFUNC DAQmxGetAIBridgeBalanceCoarsePot(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIBridgeBalanceCoarsePot(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIBridgeBalanceCoarsePot(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Bridge_Balance_FinePot ***
int32n __CFUNC DAQmxGetAIBridgeBalanceFinePot(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIBridgeBalanceFinePot(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIBridgeBalanceFinePot(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_CurrentShunt_Loc ***
// Uses value set CurrentShuntResistorLocation1
int32n __CFUNC DAQmxGetAICurrentShuntLoc(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAICurrentShuntLoc(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAICurrentShuntLoc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_CurrentShunt_Resistance ***
int32n __CFUNC DAQmxGetAICurrentShuntResistance(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAICurrentShuntResistance(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAICurrentShuntResistance(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Excit_Src ***
// Uses value set ExcitationSource
int32n __CFUNC DAQmxGetAIExcitSrc(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIExcitSrc(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIExcitSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Excit_Val ***
int32n __CFUNC DAQmxGetAIExcitVal(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIExcitVal(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIExcitVal(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Excit_UseForScaling ***
int32n __CFUNC DAQmxGetAIExcitUseForScaling(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIExcitUseForScaling(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIExcitUseForScaling(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Excit_UseMultiplexed ***
int32n __CFUNC DAQmxGetAIExcitUseMultiplexed(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIExcitUseMultiplexed(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIExcitUseMultiplexed(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Excit_ActualVal ***
int32n __CFUNC DAQmxGetAIExcitActualVal(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIExcitActualVal(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIExcitActualVal(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Excit_DCorAC ***
// Uses value set ExcitationDCorAC
int32n __CFUNC DAQmxGetAIExcitDCorAC(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIExcitDCorAC(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIExcitDCorAC(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Excit_VoltageOrCurrent ***
// Uses value set ExcitationVoltageOrCurrent
int32n __CFUNC DAQmxGetAIExcitVoltageOrCurrent(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIExcitVoltageOrCurrent(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIExcitVoltageOrCurrent(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ACExcit_Freq ***
int32n __CFUNC DAQmxGetAIACExcitFreq(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIACExcitFreq(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIACExcitFreq(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ACExcit_SyncEnable ***
int32n __CFUNC DAQmxGetAIACExcitSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIACExcitSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIACExcitSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ACExcit_WireMode ***
// Uses value set ACExcitWireMode
int32n __CFUNC DAQmxGetAIACExcitWireMode(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIACExcitWireMode(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIACExcitWireMode(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Atten ***
int32n __CFUNC DAQmxGetAIAtten(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIAtten(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIAtten(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ProbeAtten ***
int32n __CFUNC DAQmxGetAIProbeAtten(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIProbeAtten(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIProbeAtten(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Lowpass_Enable ***
int32n __CFUNC DAQmxGetAILowpassEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAILowpassEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAILowpassEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Lowpass_CutoffFreq ***
int32n __CFUNC DAQmxGetAILowpassCutoffFreq(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAILowpassCutoffFreq(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAILowpassCutoffFreq(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Lowpass_SwitchCap_ClkSrc ***
// Uses value set SourceSelection
int32n __CFUNC DAQmxGetAILowpassSwitchCapClkSrc(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAILowpassSwitchCapClkSrc(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAILowpassSwitchCapClkSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Lowpass_SwitchCap_ExtClkFreq ***
int32n __CFUNC DAQmxGetAILowpassSwitchCapExtClkFreq(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAILowpassSwitchCapExtClkFreq(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAILowpassSwitchCapExtClkFreq(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Lowpass_SwitchCap_ExtClkDiv ***
int32n __CFUNC DAQmxGetAILowpassSwitchCapExtClkDiv(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetAILowpassSwitchCapExtClkDiv(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetAILowpassSwitchCapExtClkDiv(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Lowpass_SwitchCap_OutClkDiv ***
int32n __CFUNC DAQmxGetAILowpassSwitchCapOutClkDiv(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetAILowpassSwitchCapOutClkDiv(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetAILowpassSwitchCapOutClkDiv(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ResolutionUnits ***
// Uses value set ResolutionType1
int32n __CFUNC DAQmxGetAIResolutionUnits(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_AI_Resolution ***
int32n __CFUNC DAQmxGetAIResolution(TaskHandle taskHandle, const char channel[], float64n *data);
//*** Set/Get functions for DAQmx_AI_RawSampSize ***
int32n __CFUNC DAQmxGetAIRawSampSize(TaskHandle taskHandle, const char channel[], uint32n *data);
//*** Set/Get functions for DAQmx_AI_RawSampJustification ***
// Uses value set DataJustification1
int32n __CFUNC DAQmxGetAIRawSampJustification(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_AI_ADCTimingMode ***
// Uses value set ADCTimingMode
int32n __CFUNC DAQmxGetAIADCTimingMode(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIADCTimingMode(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIADCTimingMode(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Dither_Enable ***
int32n __CFUNC DAQmxGetAIDitherEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIDitherEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIDitherEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_HasValidCalInfo ***
int32n __CFUNC DAQmxGetAIChanCalHasValidCalInfo(TaskHandle taskHandle, const char channel[], bool32n *data);
//*** Set/Get functions for DAQmx_AI_ChanCal_EnableCal ***
int32n __CFUNC DAQmxGetAIChanCalEnableCal(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIChanCalEnableCal(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIChanCalEnableCal(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_ApplyCalIfExp ***
int32n __CFUNC DAQmxGetAIChanCalApplyCalIfExp(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIChanCalApplyCalIfExp(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIChanCalApplyCalIfExp(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_ScaleType ***
// Uses value set ScaleType3
int32n __CFUNC DAQmxGetAIChanCalScaleType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIChanCalScaleType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIChanCalScaleType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_Table_PreScaledVals ***
int32n __CFUNC DAQmxGetAIChanCalTablePreScaledVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetAIChanCalTablePreScaledVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxResetAIChanCalTablePreScaledVals(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_Table_ScaledVals ***
int32n __CFUNC DAQmxGetAIChanCalTableScaledVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetAIChanCalTableScaledVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxResetAIChanCalTableScaledVals(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_Poly_ForwardCoeff ***
int32n __CFUNC DAQmxGetAIChanCalPolyForwardCoeff(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetAIChanCalPolyForwardCoeff(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxResetAIChanCalPolyForwardCoeff(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_Poly_ReverseCoeff ***
int32n __CFUNC DAQmxGetAIChanCalPolyReverseCoeff(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetAIChanCalPolyReverseCoeff(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxResetAIChanCalPolyReverseCoeff(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_OperatorName ***
int32n __CFUNC DAQmxGetAIChanCalOperatorName(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAIChanCalOperatorName(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetAIChanCalOperatorName(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_Desc ***
int32n __CFUNC DAQmxGetAIChanCalDesc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAIChanCalDesc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetAIChanCalDesc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_Verif_RefVals ***
int32n __CFUNC DAQmxGetAIChanCalVerifRefVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetAIChanCalVerifRefVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxResetAIChanCalVerifRefVals(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_ChanCal_Verif_AcqVals ***
int32n __CFUNC DAQmxGetAIChanCalVerifAcqVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetAIChanCalVerifAcqVals(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxResetAIChanCalVerifAcqVals(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Rng_High ***
int32n __CFUNC DAQmxGetAIRngHigh(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIRngHigh(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIRngHigh(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Rng_Low ***
int32n __CFUNC DAQmxGetAIRngLow(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIRngLow(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIRngLow(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_DCOffset ***
int32n __CFUNC DAQmxGetAIDCOffset(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIDCOffset(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIDCOffset(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_Gain ***
int32n __CFUNC DAQmxGetAIGain(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAIGain(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAIGain(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_SampAndHold_Enable ***
int32n __CFUNC DAQmxGetAISampAndHoldEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAISampAndHoldEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAISampAndHoldEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_AutoZeroMode ***
// Uses value set AutoZeroType1
int32n __CFUNC DAQmxGetAIAutoZeroMode(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIAutoZeroMode(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIAutoZeroMode(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_DataXferMech ***
// Uses value set DataTransferMechanism
int32n __CFUNC DAQmxGetAIDataXferMech(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIDataXferMech(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIDataXferMech(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_DataXferReqCond ***
// Uses value set InputDataTransferCondition
int32n __CFUNC DAQmxGetAIDataXferReqCond(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIDataXferReqCond(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIDataXferReqCond(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_DataXferCustomThreshold ***
int32n __CFUNC DAQmxGetAIDataXferCustomThreshold(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetAIDataXferCustomThreshold(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetAIDataXferCustomThreshold(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_UsbXferReqSize ***
int32n __CFUNC DAQmxGetAIUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetAIUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetAIUsbXferReqSize(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_MemMapEnable ***
int32n __CFUNC DAQmxGetAIMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIMemMapEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_RawDataCompressionType ***
// Uses value set RawDataCompressionType
int32n __CFUNC DAQmxGetAIRawDataCompressionType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAIRawDataCompressionType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAIRawDataCompressionType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_LossyLSBRemoval_CompressedSampSize ***
int32n __CFUNC DAQmxGetAILossyLSBRemovalCompressedSampSize(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetAILossyLSBRemovalCompressedSampSize(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetAILossyLSBRemovalCompressedSampSize(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AI_DevScalingCoeff ***
int32n __CFUNC DAQmxGetAIDevScalingCoeff(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_AI_EnhancedAliasRejectionEnable ***
int32n __CFUNC DAQmxGetAIEnhancedAliasRejectionEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAIEnhancedAliasRejectionEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAIEnhancedAliasRejectionEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_Max ***
int32n __CFUNC DAQmxGetAOMax(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOMax(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOMax(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_Min ***
int32n __CFUNC DAQmxGetAOMin(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOMin(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOMin(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_CustomScaleName ***
int32n __CFUNC DAQmxGetAOCustomScaleName(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAOCustomScaleName(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetAOCustomScaleName(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_OutputType ***
// Uses value set AOOutputChannelType
int32n __CFUNC DAQmxGetAOOutputType(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_AO_Voltage_Units ***
// Uses value set VoltageUnits2
int32n __CFUNC DAQmxGetAOVoltageUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAOVoltageUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAOVoltageUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_Voltage_CurrentLimit ***
int32n __CFUNC DAQmxGetAOVoltageCurrentLimit(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOVoltageCurrentLimit(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOVoltageCurrentLimit(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_Current_Units ***
// Uses value set CurrentUnits1
int32n __CFUNC DAQmxGetAOCurrentUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAOCurrentUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAOCurrentUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_FuncGen_Type ***
// Uses value set FuncGenType
int32n __CFUNC DAQmxGetAOFuncGenType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAOFuncGenType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAOFuncGenType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_FuncGen_Freq ***
int32n __CFUNC DAQmxGetAOFuncGenFreq(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOFuncGenFreq(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOFuncGenFreq(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_FuncGen_Amplitude ***
int32n __CFUNC DAQmxGetAOFuncGenAmplitude(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOFuncGenAmplitude(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOFuncGenAmplitude(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_FuncGen_Offset ***
int32n __CFUNC DAQmxGetAOFuncGenOffset(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOFuncGenOffset(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOFuncGenOffset(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_FuncGen_Square_DutyCycle ***
int32n __CFUNC DAQmxGetAOFuncGenSquareDutyCycle(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOFuncGenSquareDutyCycle(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOFuncGenSquareDutyCycle(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_FuncGen_ModulationType ***
// Uses value set ModulationType
int32n __CFUNC DAQmxGetAOFuncGenModulationType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAOFuncGenModulationType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAOFuncGenModulationType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_FuncGen_FMDeviation ***
int32n __CFUNC DAQmxGetAOFuncGenFMDeviation(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOFuncGenFMDeviation(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOFuncGenFMDeviation(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_OutputImpedance ***
int32n __CFUNC DAQmxGetAOOutputImpedance(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOOutputImpedance(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOOutputImpedance(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_LoadImpedance ***
int32n __CFUNC DAQmxGetAOLoadImpedance(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOLoadImpedance(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOLoadImpedance(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_IdleOutputBehavior ***
// Uses value set AOIdleOutputBehavior
int32n __CFUNC DAQmxGetAOIdleOutputBehavior(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAOIdleOutputBehavior(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAOIdleOutputBehavior(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_TermCfg ***
// Uses value set OutputTermCfg
int32n __CFUNC DAQmxGetAOTermCfg(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAOTermCfg(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAOTermCfg(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_ResolutionUnits ***
// Uses value set ResolutionType1
int32n __CFUNC DAQmxGetAOResolutionUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAOResolutionUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAOResolutionUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_Resolution ***
int32n __CFUNC DAQmxGetAOResolution(TaskHandle taskHandle, const char channel[], float64n *data);
//*** Set/Get functions for DAQmx_AO_DAC_Rng_High ***
int32n __CFUNC DAQmxGetAODACRngHigh(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAODACRngHigh(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAODACRngHigh(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Rng_Low ***
int32n __CFUNC DAQmxGetAODACRngLow(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAODACRngLow(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAODACRngLow(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Ref_ConnToGnd ***
int32n __CFUNC DAQmxGetAODACRefConnToGnd(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAODACRefConnToGnd(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAODACRefConnToGnd(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Ref_AllowConnToGnd ***
int32n __CFUNC DAQmxGetAODACRefAllowConnToGnd(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAODACRefAllowConnToGnd(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAODACRefAllowConnToGnd(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Ref_Src ***
// Uses value set SourceSelection
int32n __CFUNC DAQmxGetAODACRefSrc(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAODACRefSrc(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAODACRefSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Ref_ExtSrc ***
int32n __CFUNC DAQmxGetAODACRefExtSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAODACRefExtSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetAODACRefExtSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Ref_Val ***
int32n __CFUNC DAQmxGetAODACRefVal(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAODACRefVal(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAODACRefVal(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Offset_Src ***
// Uses value set SourceSelection
int32n __CFUNC DAQmxGetAODACOffsetSrc(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAODACOffsetSrc(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAODACOffsetSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Offset_ExtSrc ***
int32n __CFUNC DAQmxGetAODACOffsetExtSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAODACOffsetExtSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetAODACOffsetExtSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DAC_Offset_Val ***
int32n __CFUNC DAQmxGetAODACOffsetVal(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAODACOffsetVal(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAODACOffsetVal(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_ReglitchEnable ***
int32n __CFUNC DAQmxGetAOReglitchEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAOReglitchEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAOReglitchEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_Gain ***
int32n __CFUNC DAQmxGetAOGain(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetAOGain(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetAOGain(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_UseOnlyOnBrdMem ***
int32n __CFUNC DAQmxGetAOUseOnlyOnBrdMem(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAOUseOnlyOnBrdMem(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAOUseOnlyOnBrdMem(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DataXferMech ***
// Uses value set DataTransferMechanism
int32n __CFUNC DAQmxGetAODataXferMech(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAODataXferMech(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAODataXferMech(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DataXferReqCond ***
// Uses value set OutputDataTransferCondition
int32n __CFUNC DAQmxGetAODataXferReqCond(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetAODataXferReqCond(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetAODataXferReqCond(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_UsbXferReqSize ***
int32n __CFUNC DAQmxGetAOUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetAOUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetAOUsbXferReqSize(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_MemMapEnable ***
int32n __CFUNC DAQmxGetAOMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAOMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAOMemMapEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_AO_DevScalingCoeff ***
int32n __CFUNC DAQmxGetAODevScalingCoeff(TaskHandle taskHandle, const char channel[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_AO_EnhancedImageRejectionEnable ***
int32n __CFUNC DAQmxGetAOEnhancedImageRejectionEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetAOEnhancedImageRejectionEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetAOEnhancedImageRejectionEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_InvertLines ***
int32n __CFUNC DAQmxGetDIInvertLines(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDIInvertLines(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDIInvertLines(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_NumLines ***
int32n __CFUNC DAQmxGetDINumLines(TaskHandle taskHandle, const char channel[], uint32n *data);
//*** Set/Get functions for DAQmx_DI_DigFltr_Enable ***
int32n __CFUNC DAQmxGetDIDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDIDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDIDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetDIDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetDIDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetDIDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_Tristate ***
int32n __CFUNC DAQmxGetDITristate(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDITristate(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDITristate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_LogicFamily ***
// Uses value set LogicFamily
int32n __CFUNC DAQmxGetDILogicFamily(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDILogicFamily(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDILogicFamily(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_DataXferMech ***
// Uses value set DataTransferMechanism
int32n __CFUNC DAQmxGetDIDataXferMech(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDIDataXferMech(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDIDataXferMech(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_DataXferReqCond ***
// Uses value set InputDataTransferCondition
int32n __CFUNC DAQmxGetDIDataXferReqCond(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDIDataXferReqCond(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDIDataXferReqCond(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_UsbXferReqSize ***
int32n __CFUNC DAQmxGetDIUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetDIUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetDIUsbXferReqSize(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_MemMapEnable ***
int32n __CFUNC DAQmxGetDIMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDIMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDIMemMapEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DI_AcquireOn ***
// Uses value set SampleClockActiveOrInactiveEdgeSelection
int32n __CFUNC DAQmxGetDIAcquireOn(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDIAcquireOn(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDIAcquireOn(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_OutputDriveType ***
// Uses value set DigitalDriveType
int32n __CFUNC DAQmxGetDOOutputDriveType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDOOutputDriveType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDOOutputDriveType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_InvertLines ***
int32n __CFUNC DAQmxGetDOInvertLines(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDOInvertLines(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDOInvertLines(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_NumLines ***
int32n __CFUNC DAQmxGetDONumLines(TaskHandle taskHandle, const char channel[], uint32n *data);
//*** Set/Get functions for DAQmx_DO_Tristate ***
int32n __CFUNC DAQmxGetDOTristate(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDOTristate(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDOTristate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_LineStates_StartState ***
// Uses value set DigitalLineState
int32n __CFUNC DAQmxGetDOLineStatesStartState(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDOLineStatesStartState(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDOLineStatesStartState(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_LineStates_PausedState ***
// Uses value set DigitalLineState
int32n __CFUNC DAQmxGetDOLineStatesPausedState(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDOLineStatesPausedState(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDOLineStatesPausedState(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_LineStates_DoneState ***
// Uses value set DigitalLineState
int32n __CFUNC DAQmxGetDOLineStatesDoneState(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDOLineStatesDoneState(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDOLineStatesDoneState(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_LogicFamily ***
// Uses value set LogicFamily
int32n __CFUNC DAQmxGetDOLogicFamily(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDOLogicFamily(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDOLogicFamily(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_Overcurrent_Limit ***
int32n __CFUNC DAQmxGetDOOvercurrentLimit(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetDOOvercurrentLimit(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetDOOvercurrentLimit(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_Overcurrent_AutoReenable ***
int32n __CFUNC DAQmxGetDOOvercurrentAutoReenable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDOOvercurrentAutoReenable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDOOvercurrentAutoReenable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_Overcurrent_ReenablePeriod ***
int32n __CFUNC DAQmxGetDOOvercurrentReenablePeriod(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetDOOvercurrentReenablePeriod(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetDOOvercurrentReenablePeriod(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_UseOnlyOnBrdMem ***
int32n __CFUNC DAQmxGetDOUseOnlyOnBrdMem(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDOUseOnlyOnBrdMem(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDOUseOnlyOnBrdMem(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_DataXferMech ***
// Uses value set DataTransferMechanism
int32n __CFUNC DAQmxGetDODataXferMech(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDODataXferMech(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDODataXferMech(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_DataXferReqCond ***
// Uses value set OutputDataTransferCondition
int32n __CFUNC DAQmxGetDODataXferReqCond(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDODataXferReqCond(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDODataXferReqCond(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_UsbXferReqSize ***
int32n __CFUNC DAQmxGetDOUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetDOUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetDOUsbXferReqSize(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_MemMapEnable ***
int32n __CFUNC DAQmxGetDOMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetDOMemMapEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetDOMemMapEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_DO_GenerateOn ***
// Uses value set SampleClockActiveOrInactiveEdgeSelection
int32n __CFUNC DAQmxGetDOGenerateOn(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetDOGenerateOn(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetDOGenerateOn(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Max ***
int32n __CFUNC DAQmxGetCIMax(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIMax(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIMax(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Min ***
int32n __CFUNC DAQmxGetCIMin(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIMin(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIMin(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CustomScaleName ***
int32n __CFUNC DAQmxGetCICustomScaleName(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCICustomScaleName(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCICustomScaleName(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_MeasType ***
// Uses value set CIMeasurementType
int32n __CFUNC DAQmxGetCIMeasType(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_CI_Freq_Units ***
// Uses value set FrequencyUnits3
int32n __CFUNC DAQmxGetCIFreqUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIFreqUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIFreqUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_Term ***
int32n __CFUNC DAQmxGetCIFreqTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIFreqTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIFreqTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_StartingEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCIFreqStartingEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIFreqStartingEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIFreqStartingEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_MeasMeth ***
// Uses value set CounterFrequencyMethod
int32n __CFUNC DAQmxGetCIFreqMeasMeth(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIFreqMeasMeth(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIFreqMeasMeth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_MeasTime ***
int32n __CFUNC DAQmxGetCIFreqMeasTime(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIFreqMeasTime(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIFreqMeasTime(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_Div ***
int32n __CFUNC DAQmxGetCIFreqDiv(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCIFreqDiv(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCIFreqDiv(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCIFreqDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIFreqDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIFreqDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCIFreqDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIFreqDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIFreqDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCIFreqDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIFreqDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIFreqDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCIFreqDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIFreqDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIFreqDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Freq_DigSync_Enable ***
int32n __CFUNC DAQmxGetCIFreqDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIFreqDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIFreqDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_Units ***
// Uses value set TimeUnits3
int32n __CFUNC DAQmxGetCIPeriodUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIPeriodUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIPeriodUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_Term ***
int32n __CFUNC DAQmxGetCIPeriodTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIPeriodTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIPeriodTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_StartingEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCIPeriodStartingEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIPeriodStartingEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIPeriodStartingEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_MeasMeth ***
// Uses value set CounterFrequencyMethod
int32n __CFUNC DAQmxGetCIPeriodMeasMeth(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIPeriodMeasMeth(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIPeriodMeasMeth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_MeasTime ***
int32n __CFUNC DAQmxGetCIPeriodMeasTime(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIPeriodMeasTime(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIPeriodMeasTime(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_Div ***
int32n __CFUNC DAQmxGetCIPeriodDiv(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCIPeriodDiv(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCIPeriodDiv(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCIPeriodDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIPeriodDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIPeriodDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCIPeriodDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIPeriodDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIPeriodDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCIPeriodDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIPeriodDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIPeriodDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCIPeriodDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIPeriodDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIPeriodDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Period_DigSync_Enable ***
int32n __CFUNC DAQmxGetCIPeriodDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIPeriodDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIPeriodDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_Term ***
int32n __CFUNC DAQmxGetCICountEdgesTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCICountEdgesTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCICountEdgesTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_Dir ***
// Uses value set CountDirection1
int32n __CFUNC DAQmxGetCICountEdgesDir(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCICountEdgesDir(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCICountEdgesDir(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_DirTerm ***
int32n __CFUNC DAQmxGetCICountEdgesDirTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCICountEdgesDirTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCICountEdgesDirTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_CountDir_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCICountEdgesCountDirDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCICountEdgesCountDirDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCICountEdgesCountDirDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_CountDir_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCICountEdgesCountDirDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCICountEdgesCountDirDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCICountEdgesCountDirDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_CountDir_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCICountEdgesCountDirDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCICountEdgesCountDirDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCICountEdgesCountDirDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_CountDir_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCICountEdgesCountDirDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCICountEdgesCountDirDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCICountEdgesCountDirDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_CountDir_DigSync_Enable ***
int32n __CFUNC DAQmxGetCICountEdgesCountDirDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCICountEdgesCountDirDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCICountEdgesCountDirDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_InitialCnt ***
int32n __CFUNC DAQmxGetCICountEdgesInitialCnt(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCICountEdgesInitialCnt(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCICountEdgesInitialCnt(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_ActiveEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCICountEdgesActiveEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCICountEdgesActiveEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCICountEdgesActiveEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCICountEdgesDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCICountEdgesDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCICountEdgesDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCICountEdgesDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCICountEdgesDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCICountEdgesDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCICountEdgesDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCICountEdgesDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCICountEdgesDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCICountEdgesDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCICountEdgesDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCICountEdgesDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CountEdges_DigSync_Enable ***
int32n __CFUNC DAQmxGetCICountEdgesDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCICountEdgesDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCICountEdgesDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_AngEncoder_Units ***
// Uses value set AngleUnits2
int32n __CFUNC DAQmxGetCIAngEncoderUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIAngEncoderUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIAngEncoderUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_AngEncoder_PulsesPerRev ***
int32n __CFUNC DAQmxGetCIAngEncoderPulsesPerRev(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCIAngEncoderPulsesPerRev(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCIAngEncoderPulsesPerRev(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_AngEncoder_InitialAngle ***
int32n __CFUNC DAQmxGetCIAngEncoderInitialAngle(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIAngEncoderInitialAngle(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIAngEncoderInitialAngle(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_LinEncoder_Units ***
// Uses value set LengthUnits3
int32n __CFUNC DAQmxGetCILinEncoderUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCILinEncoderUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCILinEncoderUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_LinEncoder_DistPerPulse ***
int32n __CFUNC DAQmxGetCILinEncoderDistPerPulse(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCILinEncoderDistPerPulse(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCILinEncoderDistPerPulse(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_LinEncoder_InitialPos ***
int32n __CFUNC DAQmxGetCILinEncoderInitialPos(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCILinEncoderInitialPos(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCILinEncoderInitialPos(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_DecodingType ***
// Uses value set EncoderType2
int32n __CFUNC DAQmxGetCIEncoderDecodingType(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIEncoderDecodingType(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIEncoderDecodingType(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_AInputTerm ***
int32n __CFUNC DAQmxGetCIEncoderAInputTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIEncoderAInputTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIEncoderAInputTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_AInput_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCIEncoderAInputDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIEncoderAInputDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIEncoderAInputDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_AInput_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCIEncoderAInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIEncoderAInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIEncoderAInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_AInput_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCIEncoderAInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIEncoderAInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIEncoderAInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_AInput_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCIEncoderAInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIEncoderAInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIEncoderAInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_AInput_DigSync_Enable ***
int32n __CFUNC DAQmxGetCIEncoderAInputDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIEncoderAInputDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIEncoderAInputDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_BInputTerm ***
int32n __CFUNC DAQmxGetCIEncoderBInputTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIEncoderBInputTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIEncoderBInputTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_BInput_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCIEncoderBInputDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIEncoderBInputDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIEncoderBInputDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_BInput_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCIEncoderBInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIEncoderBInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIEncoderBInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_BInput_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCIEncoderBInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIEncoderBInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIEncoderBInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_BInput_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCIEncoderBInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIEncoderBInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIEncoderBInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_BInput_DigSync_Enable ***
int32n __CFUNC DAQmxGetCIEncoderBInputDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIEncoderBInputDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIEncoderBInputDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZInputTerm ***
int32n __CFUNC DAQmxGetCIEncoderZInputTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIEncoderZInputTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIEncoderZInputTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZInput_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCIEncoderZInputDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIEncoderZInputDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIEncoderZInputDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZInput_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCIEncoderZInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIEncoderZInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIEncoderZInputDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZInput_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCIEncoderZInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIEncoderZInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIEncoderZInputDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZInput_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCIEncoderZInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIEncoderZInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIEncoderZInputDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZInput_DigSync_Enable ***
int32n __CFUNC DAQmxGetCIEncoderZInputDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIEncoderZInputDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIEncoderZInputDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZIndexEnable ***
int32n __CFUNC DAQmxGetCIEncoderZIndexEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIEncoderZIndexEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIEncoderZIndexEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZIndexVal ***
int32n __CFUNC DAQmxGetCIEncoderZIndexVal(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIEncoderZIndexVal(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIEncoderZIndexVal(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Encoder_ZIndexPhase ***
// Uses value set EncoderZIndexPhase1
int32n __CFUNC DAQmxGetCIEncoderZIndexPhase(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIEncoderZIndexPhase(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIEncoderZIndexPhase(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_Units ***
// Uses value set TimeUnits3
int32n __CFUNC DAQmxGetCIPulseWidthUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIPulseWidthUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIPulseWidthUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_Term ***
int32n __CFUNC DAQmxGetCIPulseWidthTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIPulseWidthTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIPulseWidthTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_StartingEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCIPulseWidthStartingEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIPulseWidthStartingEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIPulseWidthStartingEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCIPulseWidthDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIPulseWidthDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIPulseWidthDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCIPulseWidthDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIPulseWidthDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIPulseWidthDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCIPulseWidthDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIPulseWidthDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIPulseWidthDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCIPulseWidthDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCIPulseWidthDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCIPulseWidthDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_PulseWidth_DigSync_Enable ***
int32n __CFUNC DAQmxGetCIPulseWidthDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIPulseWidthDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIPulseWidthDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_Units ***
// Uses value set TimeUnits3
int32n __CFUNC DAQmxGetCITwoEdgeSepUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_FirstTerm ***
int32n __CFUNC DAQmxGetCITwoEdgeSepFirstTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCITwoEdgeSepFirstTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCITwoEdgeSepFirstTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_FirstEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCITwoEdgeSepFirstEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepFirstEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepFirstEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_First_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCITwoEdgeSepFirstDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepFirstDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepFirstDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_First_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCITwoEdgeSepFirstDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepFirstDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepFirstDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_First_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCITwoEdgeSepFirstDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCITwoEdgeSepFirstDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCITwoEdgeSepFirstDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_First_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCITwoEdgeSepFirstDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepFirstDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepFirstDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_First_DigSync_Enable ***
int32n __CFUNC DAQmxGetCITwoEdgeSepFirstDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepFirstDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepFirstDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_SecondTerm ***
int32n __CFUNC DAQmxGetCITwoEdgeSepSecondTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCITwoEdgeSepSecondTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCITwoEdgeSepSecondTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_SecondEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCITwoEdgeSepSecondEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepSecondEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepSecondEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_Second_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCITwoEdgeSepSecondDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepSecondDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepSecondDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_Second_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCITwoEdgeSepSecondDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepSecondDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepSecondDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_Second_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCITwoEdgeSepSecondDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCITwoEdgeSepSecondDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCITwoEdgeSepSecondDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_Second_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCITwoEdgeSepSecondDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepSecondDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepSecondDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_TwoEdgeSep_Second_DigSync_Enable ***
int32n __CFUNC DAQmxGetCITwoEdgeSepSecondDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCITwoEdgeSepSecondDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCITwoEdgeSepSecondDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_Units ***
// Uses value set TimeUnits3
int32n __CFUNC DAQmxGetCISemiPeriodUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCISemiPeriodUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCISemiPeriodUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_Term ***
int32n __CFUNC DAQmxGetCISemiPeriodTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCISemiPeriodTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCISemiPeriodTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_StartingEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCISemiPeriodStartingEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCISemiPeriodStartingEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCISemiPeriodStartingEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCISemiPeriodDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCISemiPeriodDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCISemiPeriodDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCISemiPeriodDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCISemiPeriodDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCISemiPeriodDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCISemiPeriodDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCISemiPeriodDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCISemiPeriodDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCISemiPeriodDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCISemiPeriodDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCISemiPeriodDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_SemiPeriod_DigSync_Enable ***
int32n __CFUNC DAQmxGetCISemiPeriodDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCISemiPeriodDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCISemiPeriodDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Timestamp_Units ***
// Uses value set TimeUnits
int32n __CFUNC DAQmxGetCITimestampUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCITimestampUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCITimestampUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Timestamp_InitialSeconds ***
int32n __CFUNC DAQmxGetCITimestampInitialSeconds(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCITimestampInitialSeconds(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCITimestampInitialSeconds(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_GPS_SyncMethod ***
// Uses value set GpsSignalType1
int32n __CFUNC DAQmxGetCIGPSSyncMethod(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIGPSSyncMethod(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIGPSSyncMethod(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_GPS_SyncSrc ***
int32n __CFUNC DAQmxGetCIGPSSyncSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCIGPSSyncSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCIGPSSyncSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebaseSrc ***
int32n __CFUNC DAQmxGetCICtrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCICtrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCICtrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebaseRate ***
int32n __CFUNC DAQmxGetCICtrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCICtrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCICtrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebaseActiveEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCICtrTimebaseActiveEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCICtrTimebaseActiveEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCICtrTimebaseActiveEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebase_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCICtrTimebaseDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCICtrTimebaseDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCICtrTimebaseDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebase_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCICtrTimebaseDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCICtrTimebaseDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCICtrTimebaseDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebase_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCICtrTimebaseDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCICtrTimebaseDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCICtrTimebaseDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebase_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCICtrTimebaseDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCICtrTimebaseDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCICtrTimebaseDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_CtrTimebase_DigSync_Enable ***
int32n __CFUNC DAQmxGetCICtrTimebaseDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCICtrTimebaseDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCICtrTimebaseDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Count ***
int32n __CFUNC DAQmxGetCICount(TaskHandle taskHandle, const char channel[], uint32n *data);
//*** Set/Get functions for DAQmx_CI_OutputState ***
// Uses value set Level1
int32n __CFUNC DAQmxGetCIOutputState(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_CI_TCReached ***
int32n __CFUNC DAQmxGetCITCReached(TaskHandle taskHandle, const char channel[], bool32n *data);
//*** Set/Get functions for DAQmx_CI_CtrTimebaseMasterTimebaseDiv ***
int32n __CFUNC DAQmxGetCICtrTimebaseMasterTimebaseDiv(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCICtrTimebaseMasterTimebaseDiv(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCICtrTimebaseMasterTimebaseDiv(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_DataXferMech ***
// Uses value set DataTransferMechanism
int32n __CFUNC DAQmxGetCIDataXferMech(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCIDataXferMech(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCIDataXferMech(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_UsbXferReqSize ***
int32n __CFUNC DAQmxGetCIUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCIUsbXferReqSize(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCIUsbXferReqSize(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_NumPossiblyInvalidSamps ***
int32n __CFUNC DAQmxGetCINumPossiblyInvalidSamps(TaskHandle taskHandle, const char channel[], uint32n *data);
//*** Set/Get functions for DAQmx_CI_DupCountPrevent ***
int32n __CFUNC DAQmxGetCIDupCountPrevent(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCIDupCountPrevent(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCIDupCountPrevent(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CI_Prescaler ***
int32n __CFUNC DAQmxGetCIPrescaler(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCIPrescaler(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCIPrescaler(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_OutputType ***
// Uses value set COOutputType
int32n __CFUNC DAQmxGetCOOutputType(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_CO_Pulse_IdleState ***
// Uses value set Level1
int32n __CFUNC DAQmxGetCOPulseIdleState(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCOPulseIdleState(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCOPulseIdleState(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_Term ***
int32n __CFUNC DAQmxGetCOPulseTerm(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCOPulseTerm(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCOPulseTerm(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_Time_Units ***
// Uses value set TimeUnits2
int32n __CFUNC DAQmxGetCOPulseTimeUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCOPulseTimeUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCOPulseTimeUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_HighTime ***
int32n __CFUNC DAQmxGetCOPulseHighTime(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOPulseHighTime(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOPulseHighTime(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_LowTime ***
int32n __CFUNC DAQmxGetCOPulseLowTime(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOPulseLowTime(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOPulseLowTime(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_Time_InitialDelay ***
int32n __CFUNC DAQmxGetCOPulseTimeInitialDelay(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOPulseTimeInitialDelay(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOPulseTimeInitialDelay(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_DutyCyc ***
int32n __CFUNC DAQmxGetCOPulseDutyCyc(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOPulseDutyCyc(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOPulseDutyCyc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_Freq_Units ***
// Uses value set FrequencyUnits2
int32n __CFUNC DAQmxGetCOPulseFreqUnits(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCOPulseFreqUnits(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCOPulseFreqUnits(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_Freq ***
int32n __CFUNC DAQmxGetCOPulseFreq(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOPulseFreq(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOPulseFreq(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_Freq_InitialDelay ***
int32n __CFUNC DAQmxGetCOPulseFreqInitialDelay(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOPulseFreqInitialDelay(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOPulseFreqInitialDelay(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_HighTicks ***
int32n __CFUNC DAQmxGetCOPulseHighTicks(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCOPulseHighTicks(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCOPulseHighTicks(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_LowTicks ***
int32n __CFUNC DAQmxGetCOPulseLowTicks(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCOPulseLowTicks(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCOPulseLowTicks(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Pulse_Ticks_InitialDelay ***
int32n __CFUNC DAQmxGetCOPulseTicksInitialDelay(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCOPulseTicksInitialDelay(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCOPulseTicksInitialDelay(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebaseSrc ***
int32n __CFUNC DAQmxGetCOCtrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCOCtrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCOCtrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebaseRate ***
int32n __CFUNC DAQmxGetCOCtrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOCtrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOCtrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebaseActiveEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetCOCtrTimebaseActiveEdge(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCOCtrTimebaseActiveEdge(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCOCtrTimebaseActiveEdge(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebase_DigFltr_Enable ***
int32n __CFUNC DAQmxGetCOCtrTimebaseDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCOCtrTimebaseDigFltrEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCOCtrTimebaseDigFltrEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebase_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetCOCtrTimebaseDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOCtrTimebaseDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOCtrTimebaseDigFltrMinPulseWidth(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebase_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetCOCtrTimebaseDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetCOCtrTimebaseDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetCOCtrTimebaseDigFltrTimebaseSrc(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebase_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetCOCtrTimebaseDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n *data);
int32n __CFUNC DAQmxSetCOCtrTimebaseDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[], float64n data);
int32n __CFUNC DAQmxResetCOCtrTimebaseDigFltrTimebaseRate(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebase_DigSync_Enable ***
int32n __CFUNC DAQmxGetCOCtrTimebaseDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n *data);
int32n __CFUNC DAQmxSetCOCtrTimebaseDigSyncEnable(TaskHandle taskHandle, const char channel[], bool32n data);
int32n __CFUNC DAQmxResetCOCtrTimebaseDigSyncEnable(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Count ***
int32n __CFUNC DAQmxGetCOCount(TaskHandle taskHandle, const char channel[], uint32n *data);
//*** Set/Get functions for DAQmx_CO_OutputState ***
// Uses value set Level1
int32n __CFUNC DAQmxGetCOOutputState(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_CO_AutoIncrCnt ***
int32n __CFUNC DAQmxGetCOAutoIncrCnt(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCOAutoIncrCnt(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCOAutoIncrCnt(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_CtrTimebaseMasterTimebaseDiv ***
int32n __CFUNC DAQmxGetCOCtrTimebaseMasterTimebaseDiv(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCOCtrTimebaseMasterTimebaseDiv(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCOCtrTimebaseMasterTimebaseDiv(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_PulseDone ***
int32n __CFUNC DAQmxGetCOPulseDone(TaskHandle taskHandle, const char channel[], bool32n *data);
//*** Set/Get functions for DAQmx_CO_ConstrainedGenMode ***
// Uses value set ConstrainedGenMode
int32n __CFUNC DAQmxGetCOConstrainedGenMode(TaskHandle taskHandle, const char channel[], int32n *data);
int32n __CFUNC DAQmxSetCOConstrainedGenMode(TaskHandle taskHandle, const char channel[], int32n data);
int32n __CFUNC DAQmxResetCOConstrainedGenMode(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_Prescaler ***
int32n __CFUNC DAQmxGetCOPrescaler(TaskHandle taskHandle, const char channel[], uint32n *data);
int32n __CFUNC DAQmxSetCOPrescaler(TaskHandle taskHandle, const char channel[], uint32n data);
int32n __CFUNC DAQmxResetCOPrescaler(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_CO_RdyForNewVal ***
int32n __CFUNC DAQmxGetCORdyForNewVal(TaskHandle taskHandle, const char channel[], bool32n *data);
//*** Set/Get functions for DAQmx_ChanType ***
// Uses value set ChannelType
int32n __CFUNC DAQmxGetChanType(TaskHandle taskHandle, const char channel[], int32n *data);
//*** Set/Get functions for DAQmx_PhysicalChanName ***
int32n __CFUNC DAQmxGetPhysicalChanName(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetPhysicalChanName(TaskHandle taskHandle, const char channel[], const char *data);
//*** Set/Get functions for DAQmx_ChanDescr ***
int32n __CFUNC DAQmxGetChanDescr(TaskHandle taskHandle, const char channel[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetChanDescr(TaskHandle taskHandle, const char channel[], const char *data);
int32n __CFUNC DAQmxResetChanDescr(TaskHandle taskHandle, const char channel[]);
//*** Set/Get functions for DAQmx_ChanIsGlobal ***
int32n __CFUNC DAQmxGetChanIsGlobal(TaskHandle taskHandle, const char channel[], bool32n *data);

//********** Export Signal **********
//*** Set/Get functions for DAQmx_Exported_AIConvClk_OutputTerm ***
int32n __CFUNC DAQmxGetExportedAIConvClkOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedAIConvClkOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedAIConvClkOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AIConvClk_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedAIConvClkPulsePolarity(TaskHandle taskHandle, int32n *data);
//*** Set/Get functions for DAQmx_Exported_10MHzRefClk_OutputTerm ***
int32n __CFUNC DAQmxGetExported10MHzRefClkOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExported10MHzRefClkOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExported10MHzRefClkOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_20MHzTimebase_OutputTerm ***
int32n __CFUNC DAQmxGetExported20MHzTimebaseOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExported20MHzTimebaseOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExported20MHzTimebaseOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_SampClk_OutputBehavior ***
// Uses value set ExportActions3
int32n __CFUNC DAQmxGetExportedSampClkOutputBehavior(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedSampClkOutputBehavior(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedSampClkOutputBehavior(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_SampClk_OutputTerm ***
int32n __CFUNC DAQmxGetExportedSampClkOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedSampClkOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedSampClkOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_SampClk_DelayOffset ***
int32n __CFUNC DAQmxGetExportedSampClkDelayOffset(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetExportedSampClkDelayOffset(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetExportedSampClkDelayOffset(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_SampClk_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedSampClkPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedSampClkPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedSampClkPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_SampClkTimebase_OutputTerm ***
int32n __CFUNC DAQmxGetExportedSampClkTimebaseOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedSampClkTimebaseOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedSampClkTimebaseOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_DividedSampClkTimebase_OutputTerm ***
int32n __CFUNC DAQmxGetExportedDividedSampClkTimebaseOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedDividedSampClkTimebaseOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedDividedSampClkTimebaseOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AdvTrig_OutputTerm ***
int32n __CFUNC DAQmxGetExportedAdvTrigOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedAdvTrigOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedAdvTrigOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AdvTrig_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedAdvTrigPulsePolarity(TaskHandle taskHandle, int32n *data);
//*** Set/Get functions for DAQmx_Exported_AdvTrig_Pulse_WidthUnits ***
// Uses value set DigitalWidthUnits3
int32n __CFUNC DAQmxGetExportedAdvTrigPulseWidthUnits(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedAdvTrigPulseWidthUnits(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedAdvTrigPulseWidthUnits(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AdvTrig_Pulse_Width ***
int32n __CFUNC DAQmxGetExportedAdvTrigPulseWidth(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetExportedAdvTrigPulseWidth(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetExportedAdvTrigPulseWidth(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_PauseTrig_OutputTerm ***
int32n __CFUNC DAQmxGetExportedPauseTrigOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedPauseTrigOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedPauseTrigOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_PauseTrig_Lvl_ActiveLvl ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedPauseTrigLvlActiveLvl(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedPauseTrigLvlActiveLvl(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedPauseTrigLvlActiveLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RefTrig_OutputTerm ***
int32n __CFUNC DAQmxGetExportedRefTrigOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedRefTrigOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedRefTrigOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RefTrig_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedRefTrigPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedRefTrigPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedRefTrigPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_StartTrig_OutputTerm ***
int32n __CFUNC DAQmxGetExportedStartTrigOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedStartTrigOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedStartTrigOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_StartTrig_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedStartTrigPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedStartTrigPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedStartTrigPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AdvCmpltEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedAdvCmpltEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedAdvCmpltEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedAdvCmpltEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AdvCmpltEvent_Delay ***
int32n __CFUNC DAQmxGetExportedAdvCmpltEventDelay(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetExportedAdvCmpltEventDelay(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetExportedAdvCmpltEventDelay(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AdvCmpltEvent_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedAdvCmpltEventPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedAdvCmpltEventPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedAdvCmpltEventPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AdvCmpltEvent_Pulse_Width ***
int32n __CFUNC DAQmxGetExportedAdvCmpltEventPulseWidth(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetExportedAdvCmpltEventPulseWidth(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetExportedAdvCmpltEventPulseWidth(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AIHoldCmpltEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedAIHoldCmpltEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedAIHoldCmpltEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedAIHoldCmpltEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_AIHoldCmpltEvent_PulsePolarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedAIHoldCmpltEventPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedAIHoldCmpltEventPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedAIHoldCmpltEventPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_ChangeDetectEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedChangeDetectEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedChangeDetectEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedChangeDetectEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_ChangeDetectEvent_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedChangeDetectEventPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedChangeDetectEventPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedChangeDetectEventPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_CtrOutEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedCtrOutEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedCtrOutEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedCtrOutEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_CtrOutEvent_OutputBehavior ***
// Uses value set ExportActions2
int32n __CFUNC DAQmxGetExportedCtrOutEventOutputBehavior(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedCtrOutEventOutputBehavior(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedCtrOutEventOutputBehavior(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_CtrOutEvent_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedCtrOutEventPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedCtrOutEventPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedCtrOutEventPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_CtrOutEvent_Toggle_IdleState ***
// Uses value set Level1
int32n __CFUNC DAQmxGetExportedCtrOutEventToggleIdleState(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedCtrOutEventToggleIdleState(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedCtrOutEventToggleIdleState(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedHshkEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedHshkEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedHshkEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_OutputBehavior ***
// Uses value set ExportActions5
int32n __CFUNC DAQmxGetExportedHshkEventOutputBehavior(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedHshkEventOutputBehavior(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedHshkEventOutputBehavior(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_Delay ***
int32n __CFUNC DAQmxGetExportedHshkEventDelay(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetExportedHshkEventDelay(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetExportedHshkEventDelay(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_Interlocked_AssertedLvl ***
// Uses value set Level1
int32n __CFUNC DAQmxGetExportedHshkEventInterlockedAssertedLvl(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedHshkEventInterlockedAssertedLvl(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedHshkEventInterlockedAssertedLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_Interlocked_AssertOnStart ***
int32n __CFUNC DAQmxGetExportedHshkEventInterlockedAssertOnStart(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetExportedHshkEventInterlockedAssertOnStart(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetExportedHshkEventInterlockedAssertOnStart(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_Interlocked_DeassertDelay ***
int32n __CFUNC DAQmxGetExportedHshkEventInterlockedDeassertDelay(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetExportedHshkEventInterlockedDeassertDelay(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetExportedHshkEventInterlockedDeassertDelay(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_Pulse_Polarity ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedHshkEventPulsePolarity(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedHshkEventPulsePolarity(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedHshkEventPulsePolarity(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_HshkEvent_Pulse_Width ***
int32n __CFUNC DAQmxGetExportedHshkEventPulseWidth(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetExportedHshkEventPulseWidth(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetExportedHshkEventPulseWidth(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RdyForXferEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedRdyForXferEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedRdyForXferEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedRdyForXferEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RdyForXferEvent_Lvl_ActiveLvl ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedRdyForXferEventLvlActiveLvl(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedRdyForXferEventLvlActiveLvl(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedRdyForXferEventLvlActiveLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RdyForXferEvent_DeassertCond ***
// Uses value set DeassertCondition
int32n __CFUNC DAQmxGetExportedRdyForXferEventDeassertCond(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedRdyForXferEventDeassertCond(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedRdyForXferEventDeassertCond(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RdyForXferEvent_DeassertCondCustomThreshold ***
int32n __CFUNC DAQmxGetExportedRdyForXferEventDeassertCondCustomThreshold(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetExportedRdyForXferEventDeassertCondCustomThreshold(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetExportedRdyForXferEventDeassertCondCustomThreshold(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_DataActiveEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedDataActiveEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedDataActiveEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedDataActiveEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_DataActiveEvent_Lvl_ActiveLvl ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedDataActiveEventLvlActiveLvl(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedDataActiveEventLvlActiveLvl(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedDataActiveEventLvlActiveLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RdyForStartEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedRdyForStartEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedRdyForStartEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedRdyForStartEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_RdyForStartEvent_Lvl_ActiveLvl ***
// Uses value set Polarity2
int32n __CFUNC DAQmxGetExportedRdyForStartEventLvlActiveLvl(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetExportedRdyForStartEventLvlActiveLvl(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetExportedRdyForStartEventLvlActiveLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_SyncPulseEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedSyncPulseEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedSyncPulseEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedSyncPulseEventOutputTerm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Exported_WatchdogExpiredEvent_OutputTerm ***
int32n __CFUNC DAQmxGetExportedWatchdogExpiredEventOutputTerm(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetExportedWatchdogExpiredEventOutputTerm(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetExportedWatchdogExpiredEventOutputTerm(TaskHandle taskHandle);

//********** Device **********
//*** Set/Get functions for DAQmx_Dev_IsSimulated ***
int32n __CFUNC DAQmxGetDevIsSimulated(const char device[], bool32n *data);
//*** Set/Get functions for DAQmx_Dev_ProductCategory ***
// Uses value set ProductCategory
int32n __CFUNC DAQmxGetDevProductCategory(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_ProductType ***
int32n __CFUNC DAQmxGetDevProductType(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_ProductNum ***
int32n __CFUNC DAQmxGetDevProductNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_SerialNum ***
int32n __CFUNC DAQmxGetDevSerialNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Carrier_SerialNum ***
int32n __CFUNC DAQmxGetCarrierSerialNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_Chassis_ModuleDevNames ***
int32n __CFUNC DAQmxGetDevChassisModuleDevNames(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_AnlgTrigSupported ***
int32n __CFUNC DAQmxGetDevAnlgTrigSupported(const char device[], bool32n *data);
//*** Set/Get functions for DAQmx_Dev_DigTrigSupported ***
int32n __CFUNC DAQmxGetDevDigTrigSupported(const char device[], bool32n *data);
//*** Set/Get functions for DAQmx_Dev_AI_PhysicalChans ***
int32n __CFUNC DAQmxGetDevAIPhysicalChans(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_AI_MaxSingleChanRate ***
int32n __CFUNC DAQmxGetDevAIMaxSingleChanRate(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_AI_MaxMultiChanRate ***
int32n __CFUNC DAQmxGetDevAIMaxMultiChanRate(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_AI_MinRate ***
int32n __CFUNC DAQmxGetDevAIMinRate(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_AI_SimultaneousSamplingSupported ***
int32n __CFUNC DAQmxGetDevAISimultaneousSamplingSupported(const char device[], bool32n *data);
//*** Set/Get functions for DAQmx_Dev_AI_TrigUsage ***
// Uses bits from enum TriggerUsageTypeBits
int32n __CFUNC DAQmxGetDevAITrigUsage(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_AI_VoltageRngs ***
int32n __CFUNC DAQmxGetDevAIVoltageRngs(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_VoltageIntExcitDiscreteVals ***
int32n __CFUNC DAQmxGetDevAIVoltageIntExcitDiscreteVals(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_VoltageIntExcitRangeVals ***
int32n __CFUNC DAQmxGetDevAIVoltageIntExcitRangeVals(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_CurrentRngs ***
int32n __CFUNC DAQmxGetDevAICurrentRngs(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_CurrentIntExcitDiscreteVals ***
int32n __CFUNC DAQmxGetDevAICurrentIntExcitDiscreteVals(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_FreqRngs ***
int32n __CFUNC DAQmxGetDevAIFreqRngs(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_Gains ***
int32n __CFUNC DAQmxGetDevAIGains(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_Couplings ***
// Uses bits from enum CouplingTypeBits
int32n __CFUNC DAQmxGetDevAICouplings(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_AI_LowpassCutoffFreqDiscreteVals ***
int32n __CFUNC DAQmxGetDevAILowpassCutoffFreqDiscreteVals(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AI_LowpassCutoffFreqRangeVals ***
int32n __CFUNC DAQmxGetDevAILowpassCutoffFreqRangeVals(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AO_PhysicalChans ***
int32n __CFUNC DAQmxGetDevAOPhysicalChans(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_AO_SampClkSupported ***
int32n __CFUNC DAQmxGetDevAOSampClkSupported(const char device[], bool32n *data);
//*** Set/Get functions for DAQmx_Dev_AO_MaxRate ***
int32n __CFUNC DAQmxGetDevAOMaxRate(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_AO_MinRate ***
int32n __CFUNC DAQmxGetDevAOMinRate(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_AO_TrigUsage ***
// Uses bits from enum TriggerUsageTypeBits
int32n __CFUNC DAQmxGetDevAOTrigUsage(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_AO_VoltageRngs ***
int32n __CFUNC DAQmxGetDevAOVoltageRngs(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AO_CurrentRngs ***
int32n __CFUNC DAQmxGetDevAOCurrentRngs(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_AO_Gains ***
int32n __CFUNC DAQmxGetDevAOGains(const char device[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Dev_DI_Lines ***
int32n __CFUNC DAQmxGetDevDILines(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_DI_Ports ***
int32n __CFUNC DAQmxGetDevDIPorts(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_DI_MaxRate ***
int32n __CFUNC DAQmxGetDevDIMaxRate(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_DI_TrigUsage ***
// Uses bits from enum TriggerUsageTypeBits
int32n __CFUNC DAQmxGetDevDITrigUsage(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_DO_Lines ***
int32n __CFUNC DAQmxGetDevDOLines(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_DO_Ports ***
int32n __CFUNC DAQmxGetDevDOPorts(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_DO_MaxRate ***
int32n __CFUNC DAQmxGetDevDOMaxRate(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_DO_TrigUsage ***
// Uses bits from enum TriggerUsageTypeBits
int32n __CFUNC DAQmxGetDevDOTrigUsage(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_CI_PhysicalChans ***
int32n __CFUNC DAQmxGetDevCIPhysicalChans(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_CI_TrigUsage ***
// Uses bits from enum TriggerUsageTypeBits
int32n __CFUNC DAQmxGetDevCITrigUsage(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_CI_SampClkSupported ***
int32n __CFUNC DAQmxGetDevCISampClkSupported(const char device[], bool32n *data);
//*** Set/Get functions for DAQmx_Dev_CI_MaxSize ***
int32n __CFUNC DAQmxGetDevCIMaxSize(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_CI_MaxTimebase ***
int32n __CFUNC DAQmxGetDevCIMaxTimebase(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_CO_PhysicalChans ***
int32n __CFUNC DAQmxGetDevCOPhysicalChans(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_CO_TrigUsage ***
// Uses bits from enum TriggerUsageTypeBits
int32n __CFUNC DAQmxGetDevCOTrigUsage(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_CO_MaxSize ***
int32n __CFUNC DAQmxGetDevCOMaxSize(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_CO_MaxTimebase ***
int32n __CFUNC DAQmxGetDevCOMaxTimebase(const char device[], float64n *data);
//*** Set/Get functions for DAQmx_Dev_NumDMAChans ***
int32n __CFUNC DAQmxGetDevNumDMAChans(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_BusType ***
// Uses value set BusType
int32n __CFUNC DAQmxGetDevBusType(const char device[], int32n *data);
//*** Set/Get functions for DAQmx_Dev_PCI_BusNum ***
int32n __CFUNC DAQmxGetDevPCIBusNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_PCI_DevNum ***
int32n __CFUNC DAQmxGetDevPCIDevNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_PXI_ChassisNum ***
int32n __CFUNC DAQmxGetDevPXIChassisNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_PXI_SlotNum ***
int32n __CFUNC DAQmxGetDevPXISlotNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_CompactDAQ_ChassisDevName ***
int32n __CFUNC DAQmxGetDevCompactDAQChassisDevName(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_CompactDAQ_SlotNum ***
int32n __CFUNC DAQmxGetDevCompactDAQSlotNum(const char device[], uint32n *data);
//*** Set/Get functions for DAQmx_Dev_TCPIP_Hostname ***
int32n __CFUNC DAQmxGetDevTCPIPHostname(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_TCPIP_EthernetIP ***
int32n __CFUNC DAQmxGetDevTCPIPEthernetIP(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_TCPIP_WirelessIP ***
int32n __CFUNC DAQmxGetDevTCPIPWirelessIP(const char device[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Dev_Terminals ***
int32n __CFUNC DAQmxGetDevTerminals(const char device[], char *data, uint32n bufferSize);

//********** Read **********
//*** Set/Get functions for DAQmx_Read_RelativeTo ***
// Uses value set ReadRelativeTo
int32n __CFUNC DAQmxGetReadRelativeTo(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetReadRelativeTo(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetReadRelativeTo(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Read_Offset ***
int32n __CFUNC DAQmxGetReadOffset(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetReadOffset(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetReadOffset(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Read_ChannelsToRead ***
int32n __CFUNC DAQmxGetReadChannelsToRead(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetReadChannelsToRead(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetReadChannelsToRead(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Read_ReadAllAvailSamp ***
int32n __CFUNC DAQmxGetReadReadAllAvailSamp(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetReadReadAllAvailSamp(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetReadReadAllAvailSamp(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Read_AutoStart ***
int32n __CFUNC DAQmxGetReadAutoStart(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetReadAutoStart(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetReadAutoStart(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Read_OverWrite ***
// Uses value set OverwriteMode1
int32n __CFUNC DAQmxGetReadOverWrite(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetReadOverWrite(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetReadOverWrite(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Read_CurrReadPos ***
int32n __CFUNC DAQmxGetReadCurrReadPos(TaskHandle taskHandle, uInt64n *data);
//*** Set/Get functions for DAQmx_Read_AvailSampPerChan ***
int32n __CFUNC DAQmxGetReadAvailSampPerChan(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Read_TotalSampPerChanAcquired ***
int32n __CFUNC DAQmxGetReadTotalSampPerChanAcquired(TaskHandle taskHandle, uInt64n *data);
//*** Set/Get functions for DAQmx_Read_CommonModeRangeErrorChansExist ***
int32n __CFUNC DAQmxGetReadCommonModeRangeErrorChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Read_CommonModeRangeErrorChans ***
int32n __CFUNC DAQmxGetReadCommonModeRangeErrorChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Read_OvercurrentChansExist ***
int32n __CFUNC DAQmxGetReadOvercurrentChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Read_OvercurrentChans ***
int32n __CFUNC DAQmxGetReadOvercurrentChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Read_OpenCurrentLoopChansExist ***
int32n __CFUNC DAQmxGetReadOpenCurrentLoopChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Read_OpenCurrentLoopChans ***
int32n __CFUNC DAQmxGetReadOpenCurrentLoopChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Read_OpenThrmcplChansExist ***
int32n __CFUNC DAQmxGetReadOpenThrmcplChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Read_OpenThrmcplChans ***
int32n __CFUNC DAQmxGetReadOpenThrmcplChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Read_OverloadedChansExist ***
int32n __CFUNC DAQmxGetReadOverloadedChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Read_OverloadedChans ***
int32n __CFUNC DAQmxGetReadOverloadedChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Read_ChangeDetect_HasOverflowed ***
int32n __CFUNC DAQmxGetReadChangeDetectHasOverflowed(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Read_RawDataWidth ***
int32n __CFUNC DAQmxGetReadRawDataWidth(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Read_NumChans ***
int32n __CFUNC DAQmxGetReadNumChans(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Read_DigitalLines_BytesPerChan ***
int32n __CFUNC DAQmxGetReadDigitalLinesBytesPerChan(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Read_WaitMode ***
// Uses value set WaitMode
int32n __CFUNC DAQmxGetReadWaitMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetReadWaitMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetReadWaitMode(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Read_SleepTime ***
int32n __CFUNC DAQmxGetReadSleepTime(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetReadSleepTime(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetReadSleepTime(TaskHandle taskHandle);

//********** Real-Time **********
//*** Set/Get functions for DAQmx_RealTime_ConvLateErrorsToWarnings ***
int32n __CFUNC DAQmxGetRealTimeConvLateErrorsToWarnings(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetRealTimeConvLateErrorsToWarnings(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetRealTimeConvLateErrorsToWarnings(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RealTime_NumOfWarmupIters ***
int32n __CFUNC DAQmxGetRealTimeNumOfWarmupIters(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetRealTimeNumOfWarmupIters(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetRealTimeNumOfWarmupIters(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RealTime_WaitForNextSampClkWaitMode ***
// Uses value set WaitMode3
int32n __CFUNC DAQmxGetRealTimeWaitForNextSampClkWaitMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetRealTimeWaitForNextSampClkWaitMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetRealTimeWaitForNextSampClkWaitMode(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RealTime_ReportMissedSamp ***
int32n __CFUNC DAQmxGetRealTimeReportMissedSamp(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetRealTimeReportMissedSamp(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetRealTimeReportMissedSamp(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RealTime_WriteRecoveryMode ***
// Uses value set WaitMode4
int32n __CFUNC DAQmxGetRealTimeWriteRecoveryMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetRealTimeWriteRecoveryMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetRealTimeWriteRecoveryMode(TaskHandle taskHandle);

//********** Switch Channel **********
//*** Set/Get functions for DAQmx_SwitchChan_Usage ***
// Uses value set SwitchUsageTypes
int32n __CFUNC DAQmxGetSwitchChanUsage(const char switchChannelName[], int32n *data);
int32n __CFUNC DAQmxSetSwitchChanUsage(const char switchChannelName[], int32n data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxACCarryCurrent ***
int32n __CFUNC DAQmxGetSwitchChanMaxACCarryCurrent(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxACSwitchCurrent ***
int32n __CFUNC DAQmxGetSwitchChanMaxACSwitchCurrent(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxACCarryPwr ***
int32n __CFUNC DAQmxGetSwitchChanMaxACCarryPwr(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxACSwitchPwr ***
int32n __CFUNC DAQmxGetSwitchChanMaxACSwitchPwr(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxDCCarryCurrent ***
int32n __CFUNC DAQmxGetSwitchChanMaxDCCarryCurrent(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxDCSwitchCurrent ***
int32n __CFUNC DAQmxGetSwitchChanMaxDCSwitchCurrent(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxDCCarryPwr ***
int32n __CFUNC DAQmxGetSwitchChanMaxDCCarryPwr(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxDCSwitchPwr ***
int32n __CFUNC DAQmxGetSwitchChanMaxDCSwitchPwr(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxACVoltage ***
int32n __CFUNC DAQmxGetSwitchChanMaxACVoltage(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_MaxDCVoltage ***
int32n __CFUNC DAQmxGetSwitchChanMaxDCVoltage(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_WireMode ***
int32n __CFUNC DAQmxGetSwitchChanWireMode(const char switchChannelName[], uint32n *data);
//*** Set/Get functions for DAQmx_SwitchChan_Bandwidth ***
int32n __CFUNC DAQmxGetSwitchChanBandwidth(const char switchChannelName[], float64n *data);
//*** Set/Get functions for DAQmx_SwitchChan_Impedance ***
int32n __CFUNC DAQmxGetSwitchChanImpedance(const char switchChannelName[], float64n *data);

//********** Switch Device **********
//*** Set/Get functions for DAQmx_SwitchDev_SettlingTime ***
int32n __CFUNC DAQmxGetSwitchDevSettlingTime(const char deviceName[], float64n *data);
int32n __CFUNC DAQmxSetSwitchDevSettlingTime(const char deviceName[], float64n data);
//*** Set/Get functions for DAQmx_SwitchDev_AutoConnAnlgBus ***
int32n __CFUNC DAQmxGetSwitchDevAutoConnAnlgBus(const char deviceName[], bool32n *data);
int32n __CFUNC DAQmxSetSwitchDevAutoConnAnlgBus(const char deviceName[], bool32n data);
//*** Set/Get functions for DAQmx_SwitchDev_PwrDownLatchRelaysAfterSettling ***
int32n __CFUNC DAQmxGetSwitchDevPwrDownLatchRelaysAfterSettling(const char deviceName[], bool32n *data);
int32n __CFUNC DAQmxSetSwitchDevPwrDownLatchRelaysAfterSettling(const char deviceName[], bool32n data);
//*** Set/Get functions for DAQmx_SwitchDev_Settled ***
int32n __CFUNC DAQmxGetSwitchDevSettled(const char deviceName[], bool32n *data);
//*** Set/Get functions for DAQmx_SwitchDev_RelayList ***
int32n __CFUNC DAQmxGetSwitchDevRelayList(const char deviceName[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_SwitchDev_NumRelays ***
int32n __CFUNC DAQmxGetSwitchDevNumRelays(const char deviceName[], uint32n *data);
//*** Set/Get functions for DAQmx_SwitchDev_SwitchChanList ***
int32n __CFUNC DAQmxGetSwitchDevSwitchChanList(const char deviceName[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_SwitchDev_NumSwitchChans ***
int32n __CFUNC DAQmxGetSwitchDevNumSwitchChans(const char deviceName[], uint32n *data);
//*** Set/Get functions for DAQmx_SwitchDev_NumRows ***
int32n __CFUNC DAQmxGetSwitchDevNumRows(const char deviceName[], uint32n *data);
//*** Set/Get functions for DAQmx_SwitchDev_NumColumns ***
int32n __CFUNC DAQmxGetSwitchDevNumColumns(const char deviceName[], uint32n *data);
//*** Set/Get functions for DAQmx_SwitchDev_Topology ***
int32n __CFUNC DAQmxGetSwitchDevTopology(const char deviceName[], char *data, uint32n bufferSize);

//********** Switch Scan **********
//*** Set/Get functions for DAQmx_SwitchScan_BreakMode ***
// Uses value set BreakMode
int32n __CFUNC DAQmxGetSwitchScanBreakMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSwitchScanBreakMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSwitchScanBreakMode(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SwitchScan_RepeatMode ***
// Uses value set SwitchScanRepeatMode
int32n __CFUNC DAQmxGetSwitchScanRepeatMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSwitchScanRepeatMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSwitchScanRepeatMode(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SwitchScan_WaitingForAdv ***
int32n __CFUNC DAQmxGetSwitchScanWaitingForAdv(TaskHandle taskHandle, bool32n *data);

//********** Scale **********
//*** Set/Get functions for DAQmx_Scale_Descr ***
int32n __CFUNC DAQmxGetScaleDescr(const char scaleName[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetScaleDescr(const char scaleName[], const char *data);
//*** Set/Get functions for DAQmx_Scale_ScaledUnits ***
int32n __CFUNC DAQmxGetScaleScaledUnits(const char scaleName[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetScaleScaledUnits(const char scaleName[], const char *data);
//*** Set/Get functions for DAQmx_Scale_PreScaledUnits ***
// Uses value set UnitsPreScaled
int32n __CFUNC DAQmxGetScalePreScaledUnits(const char scaleName[], int32n *data);
int32n __CFUNC DAQmxSetScalePreScaledUnits(const char scaleName[], int32n data);
//*** Set/Get functions for DAQmx_Scale_Type ***
// Uses value set ScaleType
int32n __CFUNC DAQmxGetScaleType(const char scaleName[], int32n *data);
//*** Set/Get functions for DAQmx_Scale_Lin_Slope ***
int32n __CFUNC DAQmxGetScaleLinSlope(const char scaleName[], float64n *data);
int32n __CFUNC DAQmxSetScaleLinSlope(const char scaleName[], float64n data);
//*** Set/Get functions for DAQmx_Scale_Lin_YIntercept ***
int32n __CFUNC DAQmxGetScaleLinYIntercept(const char scaleName[], float64n *data);
int32n __CFUNC DAQmxSetScaleLinYIntercept(const char scaleName[], float64n data);
//*** Set/Get functions for DAQmx_Scale_Map_ScaledMax ***
int32n __CFUNC DAQmxGetScaleMapScaledMax(const char scaleName[], float64n *data);
int32n __CFUNC DAQmxSetScaleMapScaledMax(const char scaleName[], float64n data);
//*** Set/Get functions for DAQmx_Scale_Map_PreScaledMax ***
int32n __CFUNC DAQmxGetScaleMapPreScaledMax(const char scaleName[], float64n *data);
int32n __CFUNC DAQmxSetScaleMapPreScaledMax(const char scaleName[], float64n data);
//*** Set/Get functions for DAQmx_Scale_Map_ScaledMin ***
int32n __CFUNC DAQmxGetScaleMapScaledMin(const char scaleName[], float64n *data);
int32n __CFUNC DAQmxSetScaleMapScaledMin(const char scaleName[], float64n data);
//*** Set/Get functions for DAQmx_Scale_Map_PreScaledMin ***
int32n __CFUNC DAQmxGetScaleMapPreScaledMin(const char scaleName[], float64n *data);
int32n __CFUNC DAQmxSetScaleMapPreScaledMin(const char scaleName[], float64n data);
//*** Set/Get functions for DAQmx_Scale_Poly_ForwardCoeff ***
int32n __CFUNC DAQmxGetScalePolyForwardCoeff(const char scaleName[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetScalePolyForwardCoeff(const char scaleName[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Scale_Poly_ReverseCoeff ***
int32n __CFUNC DAQmxGetScalePolyReverseCoeff(const char scaleName[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetScalePolyReverseCoeff(const char scaleName[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Scale_Table_ScaledVals ***
int32n __CFUNC DAQmxGetScaleTableScaledVals(const char scaleName[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetScaleTableScaledVals(const char scaleName[], float64n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_Scale_Table_PreScaledVals ***
int32n __CFUNC DAQmxGetScaleTablePreScaledVals(const char scaleName[], float64n *data, uint32n arraySizeInSamples);
int32n __CFUNC DAQmxSetScaleTablePreScaledVals(const char scaleName[], float64n *data, uint32n arraySizeInSamples);

//********** System **********
//*** Set/Get functions for DAQmx_Sys_GlobalChans ***
int32n __CFUNC DAQmxGetSysGlobalChans(char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Sys_Scales ***
int32n __CFUNC DAQmxGetSysScales(char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Sys_Tasks ***
int32n __CFUNC DAQmxGetSysTasks(char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Sys_DevNames ***
int32n __CFUNC DAQmxGetSysDevNames(char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Sys_NIDAQMajorVersion ***
int32n __CFUNC DAQmxGetSysNIDAQMajorVersion(uint32n *data);
//*** Set/Get functions for DAQmx_Sys_NIDAQMinorVersion ***
int32n __CFUNC DAQmxGetSysNIDAQMinorVersion(uint32n *data);

//********** Task **********
//*** Set/Get functions for DAQmx_Task_Name ***
int32n __CFUNC DAQmxGetTaskName(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Task_Channels ***
int32n __CFUNC DAQmxGetTaskChannels(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Task_NumChans ***
int32n __CFUNC DAQmxGetTaskNumChans(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Task_Devices ***
int32n __CFUNC DAQmxGetTaskDevices(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Task_NumDevices ***
int32n __CFUNC DAQmxGetTaskNumDevices(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Task_Complete ***
int32n __CFUNC DAQmxGetTaskComplete(TaskHandle taskHandle, bool32n *data);

//********** Timing **********
//*** Set/Get functions for DAQmx_SampQuant_SampMode ***
// Uses value set AcquisitionType
int32n __CFUNC DAQmxGetSampQuantSampMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSampQuantSampMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSampQuantSampMode(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampQuant_SampPerChan ***
int32n __CFUNC DAQmxGetSampQuantSampPerChan(TaskHandle taskHandle, uInt64n *data);
int32n __CFUNC DAQmxSetSampQuantSampPerChan(TaskHandle taskHandle, uInt64n data);
int32n __CFUNC DAQmxResetSampQuantSampPerChan(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampTimingType ***
// Uses value set SampleTimingType
int32n __CFUNC DAQmxGetSampTimingType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSampTimingType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSampTimingType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_Rate ***
int32n __CFUNC DAQmxGetSampClkRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetSampClkRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetSampClkRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_MaxRate ***
int32n __CFUNC DAQmxGetSampClkMaxRate(TaskHandle taskHandle, float64n *data);
//*** Set/Get functions for DAQmx_SampClk_Src ***
int32n __CFUNC DAQmxGetSampClkSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetSampClkSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetSampClkSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_ActiveEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetSampClkActiveEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSampClkActiveEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSampClkActiveEdge(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_UnderflowBehavior ***
// Uses value set UnderflowBehavior
int32n __CFUNC DAQmxGetSampClkUnderflowBehavior(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSampClkUnderflowBehavior(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSampClkUnderflowBehavior(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_TimebaseDiv ***
int32n __CFUNC DAQmxGetSampClkTimebaseDiv(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetSampClkTimebaseDiv(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetSampClkTimebaseDiv(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_Timebase_Rate ***
int32n __CFUNC DAQmxGetSampClkTimebaseRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetSampClkTimebaseRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetSampClkTimebaseRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_Timebase_Src ***
int32n __CFUNC DAQmxGetSampClkTimebaseSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetSampClkTimebaseSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetSampClkTimebaseSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_Timebase_ActiveEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetSampClkTimebaseActiveEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSampClkTimebaseActiveEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSampClkTimebaseActiveEdge(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_Timebase_MasterTimebaseDiv ***
int32n __CFUNC DAQmxGetSampClkTimebaseMasterTimebaseDiv(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetSampClkTimebaseMasterTimebaseDiv(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetSampClkTimebaseMasterTimebaseDiv(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_DigFltr_Enable ***
int32n __CFUNC DAQmxGetSampClkDigFltrEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetSampClkDigFltrEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetSampClkDigFltrEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetSampClkDigFltrMinPulseWidth(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetSampClkDigFltrMinPulseWidth(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetSampClkDigFltrMinPulseWidth(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetSampClkDigFltrTimebaseSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetSampClkDigFltrTimebaseSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetSampClkDigFltrTimebaseSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetSampClkDigFltrTimebaseRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetSampClkDigFltrTimebaseRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetSampClkDigFltrTimebaseRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampClk_DigSync_Enable ***
int32n __CFUNC DAQmxGetSampClkDigSyncEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetSampClkDigSyncEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetSampClkDigSyncEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Hshk_DelayAfterXfer ***
int32n __CFUNC DAQmxGetHshkDelayAfterXfer(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetHshkDelayAfterXfer(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetHshkDelayAfterXfer(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Hshk_StartCond ***
// Uses value set HandshakeStartCondition
int32n __CFUNC DAQmxGetHshkStartCond(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetHshkStartCond(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetHshkStartCond(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Hshk_SampleInputDataWhen ***
// Uses value set SampleInputDataWhen
int32n __CFUNC DAQmxGetHshkSampleInputDataWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetHshkSampleInputDataWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetHshkSampleInputDataWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_ChangeDetect_DI_RisingEdgePhysicalChans ***
int32n __CFUNC DAQmxGetChangeDetectDIRisingEdgePhysicalChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetChangeDetectDIRisingEdgePhysicalChans(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetChangeDetectDIRisingEdgePhysicalChans(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_ChangeDetect_DI_FallingEdgePhysicalChans ***
int32n __CFUNC DAQmxGetChangeDetectDIFallingEdgePhysicalChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetChangeDetectDIFallingEdgePhysicalChans(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetChangeDetectDIFallingEdgePhysicalChans(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_OnDemand_SimultaneousAOEnable ***
int32n __CFUNC DAQmxGetOnDemandSimultaneousAOEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetOnDemandSimultaneousAOEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetOnDemandSimultaneousAOEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AIConv_Rate ***
int32n __CFUNC DAQmxGetAIConvRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAIConvRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAIConvRate(TaskHandle taskHandle);
int32n __CFUNC DAQmxGetAIConvRateEx(TaskHandle taskHandle, const char deviceNames[], float64n *data);
int32n __CFUNC DAQmxSetAIConvRateEx(TaskHandle taskHandle, const char deviceNames[], float64n data);
int32n __CFUNC DAQmxResetAIConvRateEx(TaskHandle taskHandle, const char deviceNames[]);
//*** Set/Get functions for DAQmx_AIConv_MaxRate ***
int32n __CFUNC DAQmxGetAIConvMaxRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxGetAIConvMaxRateEx(TaskHandle taskHandle, const char deviceNames[], float64n *data);
//*** Set/Get functions for DAQmx_AIConv_Src ***
int32n __CFUNC DAQmxGetAIConvSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAIConvSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetAIConvSrc(TaskHandle taskHandle);
int32n __CFUNC DAQmxGetAIConvSrcEx(TaskHandle taskHandle, const char deviceNames[], char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAIConvSrcEx(TaskHandle taskHandle, const char deviceNames[], const char *data);
int32n __CFUNC DAQmxResetAIConvSrcEx(TaskHandle taskHandle, const char deviceNames[]);
//*** Set/Get functions for DAQmx_AIConv_ActiveEdge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetAIConvActiveEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAIConvActiveEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAIConvActiveEdge(TaskHandle taskHandle);
int32n __CFUNC DAQmxGetAIConvActiveEdgeEx(TaskHandle taskHandle, const char deviceNames[], int32n *data);
int32n __CFUNC DAQmxSetAIConvActiveEdgeEx(TaskHandle taskHandle, const char deviceNames[], int32n data);
int32n __CFUNC DAQmxResetAIConvActiveEdgeEx(TaskHandle taskHandle, const char deviceNames[]);
//*** Set/Get functions for DAQmx_AIConv_TimebaseDiv ***
int32n __CFUNC DAQmxGetAIConvTimebaseDiv(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetAIConvTimebaseDiv(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetAIConvTimebaseDiv(TaskHandle taskHandle);
int32n __CFUNC DAQmxGetAIConvTimebaseDivEx(TaskHandle taskHandle, const char deviceNames[], uint32n *data);
int32n __CFUNC DAQmxSetAIConvTimebaseDivEx(TaskHandle taskHandle, const char deviceNames[], uint32n data);
int32n __CFUNC DAQmxResetAIConvTimebaseDivEx(TaskHandle taskHandle, const char deviceNames[]);
//*** Set/Get functions for DAQmx_AIConv_Timebase_Src ***
// Uses value set MIOAIConvertTbSrc
int32n __CFUNC DAQmxGetAIConvTimebaseSrc(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAIConvTimebaseSrc(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAIConvTimebaseSrc(TaskHandle taskHandle);
int32n __CFUNC DAQmxGetAIConvTimebaseSrcEx(TaskHandle taskHandle, const char deviceNames[], int32n *data);
int32n __CFUNC DAQmxSetAIConvTimebaseSrcEx(TaskHandle taskHandle, const char deviceNames[], int32n data);
int32n __CFUNC DAQmxResetAIConvTimebaseSrcEx(TaskHandle taskHandle, const char deviceNames[]);
//*** Set/Get functions for DAQmx_DelayFromSampClk_DelayUnits ***
// Uses value set DigitalWidthUnits2
int32n __CFUNC DAQmxGetDelayFromSampClkDelayUnits(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDelayFromSampClkDelayUnits(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDelayFromSampClkDelayUnits(TaskHandle taskHandle);
int32n __CFUNC DAQmxGetDelayFromSampClkDelayUnitsEx(TaskHandle taskHandle, const char deviceNames[], int32n *data);
int32n __CFUNC DAQmxSetDelayFromSampClkDelayUnitsEx(TaskHandle taskHandle, const char deviceNames[], int32n data);
int32n __CFUNC DAQmxResetDelayFromSampClkDelayUnitsEx(TaskHandle taskHandle, const char deviceNames[]);
//*** Set/Get functions for DAQmx_DelayFromSampClk_Delay ***
int32n __CFUNC DAQmxGetDelayFromSampClkDelay(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetDelayFromSampClkDelay(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetDelayFromSampClkDelay(TaskHandle taskHandle);
int32n __CFUNC DAQmxGetDelayFromSampClkDelayEx(TaskHandle taskHandle, const char deviceNames[], float64n *data);
int32n __CFUNC DAQmxSetDelayFromSampClkDelayEx(TaskHandle taskHandle, const char deviceNames[], float64n data);
int32n __CFUNC DAQmxResetDelayFromSampClkDelayEx(TaskHandle taskHandle, const char deviceNames[]);
//*** Set/Get functions for DAQmx_MasterTimebase_Rate ***
int32n __CFUNC DAQmxGetMasterTimebaseRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetMasterTimebaseRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetMasterTimebaseRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_MasterTimebase_Src ***
int32n __CFUNC DAQmxGetMasterTimebaseSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetMasterTimebaseSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetMasterTimebaseSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RefClk_Rate ***
int32n __CFUNC DAQmxGetRefClkRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetRefClkRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetRefClkRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RefClk_Src ***
int32n __CFUNC DAQmxGetRefClkSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetRefClkSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetRefClkSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SyncPulse_Src ***
int32n __CFUNC DAQmxGetSyncPulseSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetSyncPulseSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetSyncPulseSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SyncPulse_SyncTime ***
int32n __CFUNC DAQmxGetSyncPulseSyncTime(TaskHandle taskHandle, float64n *data);
//*** Set/Get functions for DAQmx_SyncPulse_MinDelayToStart ***
int32n __CFUNC DAQmxGetSyncPulseMinDelayToStart(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetSyncPulseMinDelayToStart(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetSyncPulseMinDelayToStart(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_SampTimingEngine ***
int32n __CFUNC DAQmxGetSampTimingEngine(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetSampTimingEngine(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetSampTimingEngine(TaskHandle taskHandle);

//********** Trigger **********
//*** Set/Get functions for DAQmx_StartTrig_Type ***
// Uses value set TriggerType8
int32n __CFUNC DAQmxGetStartTrigType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetStartTrigType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetStartTrigType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_StartTrig_Src ***
int32n __CFUNC DAQmxGetDigEdgeStartTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigEdgeStartTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigEdgeStartTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_StartTrig_Edge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetDigEdgeStartTrigEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigEdgeStartTrigEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigEdgeStartTrigEdge(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_StartTrig_DigFltr_Enable ***
int32n __CFUNC DAQmxGetDigEdgeStartTrigDigFltrEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetDigEdgeStartTrigDigFltrEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetDigEdgeStartTrigDigFltrEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_StartTrig_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetDigEdgeStartTrigDigFltrMinPulseWidth(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetDigEdgeStartTrigDigFltrMinPulseWidth(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetDigEdgeStartTrigDigFltrMinPulseWidth(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_StartTrig_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetDigEdgeStartTrigDigFltrTimebaseSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigEdgeStartTrigDigFltrTimebaseSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigEdgeStartTrigDigFltrTimebaseSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_StartTrig_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetDigEdgeStartTrigDigFltrTimebaseRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetDigEdgeStartTrigDigFltrTimebaseRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetDigEdgeStartTrigDigFltrTimebaseRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_StartTrig_DigSync_Enable ***
int32n __CFUNC DAQmxGetDigEdgeStartTrigDigSyncEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetDigEdgeStartTrigDigSyncEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetDigEdgeStartTrigDigSyncEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_StartTrig_Src ***
int32n __CFUNC DAQmxGetDigPatternStartTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigPatternStartTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigPatternStartTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_StartTrig_Pattern ***
int32n __CFUNC DAQmxGetDigPatternStartTrigPattern(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigPatternStartTrigPattern(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigPatternStartTrigPattern(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_StartTrig_When ***
// Uses value set DigitalPatternCondition1
int32n __CFUNC DAQmxGetDigPatternStartTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigPatternStartTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigPatternStartTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_StartTrig_Src ***
int32n __CFUNC DAQmxGetAnlgEdgeStartTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAnlgEdgeStartTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetAnlgEdgeStartTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_StartTrig_Slope ***
// Uses value set Slope1
int32n __CFUNC DAQmxGetAnlgEdgeStartTrigSlope(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgEdgeStartTrigSlope(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgEdgeStartTrigSlope(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_StartTrig_Lvl ***
int32n __CFUNC DAQmxGetAnlgEdgeStartTrigLvl(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgEdgeStartTrigLvl(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgEdgeStartTrigLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_StartTrig_Hyst ***
int32n __CFUNC DAQmxGetAnlgEdgeStartTrigHyst(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgEdgeStartTrigHyst(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgEdgeStartTrigHyst(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_StartTrig_Coupling ***
// Uses value set Coupling2
int32n __CFUNC DAQmxGetAnlgEdgeStartTrigCoupling(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgEdgeStartTrigCoupling(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgEdgeStartTrigCoupling(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_StartTrig_Src ***
int32n __CFUNC DAQmxGetAnlgWinStartTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAnlgWinStartTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetAnlgWinStartTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_StartTrig_When ***
// Uses value set WindowTriggerCondition1
int32n __CFUNC DAQmxGetAnlgWinStartTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgWinStartTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgWinStartTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_StartTrig_Top ***
int32n __CFUNC DAQmxGetAnlgWinStartTrigTop(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgWinStartTrigTop(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgWinStartTrigTop(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_StartTrig_Btm ***
int32n __CFUNC DAQmxGetAnlgWinStartTrigBtm(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgWinStartTrigBtm(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgWinStartTrigBtm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_StartTrig_Coupling ***
// Uses value set Coupling2
int32n __CFUNC DAQmxGetAnlgWinStartTrigCoupling(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgWinStartTrigCoupling(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgWinStartTrigCoupling(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_StartTrig_Delay ***
int32n __CFUNC DAQmxGetStartTrigDelay(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetStartTrigDelay(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetStartTrigDelay(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_StartTrig_DelayUnits ***
// Uses value set DigitalWidthUnits1
int32n __CFUNC DAQmxGetStartTrigDelayUnits(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetStartTrigDelayUnits(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetStartTrigDelayUnits(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_StartTrig_Retriggerable ***
int32n __CFUNC DAQmxGetStartTrigRetriggerable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetStartTrigRetriggerable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetStartTrigRetriggerable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RefTrig_Type ***
// Uses value set TriggerType8
int32n __CFUNC DAQmxGetRefTrigType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetRefTrigType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetRefTrigType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_RefTrig_PretrigSamples ***
int32n __CFUNC DAQmxGetRefTrigPretrigSamples(TaskHandle taskHandle, uint32n *data);
int32n __CFUNC DAQmxSetRefTrigPretrigSamples(TaskHandle taskHandle, uint32n data);
int32n __CFUNC DAQmxResetRefTrigPretrigSamples(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_RefTrig_Src ***
int32n __CFUNC DAQmxGetDigEdgeRefTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigEdgeRefTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigEdgeRefTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_RefTrig_Edge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetDigEdgeRefTrigEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigEdgeRefTrigEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigEdgeRefTrigEdge(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_RefTrig_Src ***
int32n __CFUNC DAQmxGetDigPatternRefTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigPatternRefTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigPatternRefTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_RefTrig_Pattern ***
int32n __CFUNC DAQmxGetDigPatternRefTrigPattern(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigPatternRefTrigPattern(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigPatternRefTrigPattern(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_RefTrig_When ***
// Uses value set DigitalPatternCondition1
int32n __CFUNC DAQmxGetDigPatternRefTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigPatternRefTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigPatternRefTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_RefTrig_Src ***
int32n __CFUNC DAQmxGetAnlgEdgeRefTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAnlgEdgeRefTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetAnlgEdgeRefTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_RefTrig_Slope ***
// Uses value set Slope1
int32n __CFUNC DAQmxGetAnlgEdgeRefTrigSlope(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgEdgeRefTrigSlope(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgEdgeRefTrigSlope(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_RefTrig_Lvl ***
int32n __CFUNC DAQmxGetAnlgEdgeRefTrigLvl(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgEdgeRefTrigLvl(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgEdgeRefTrigLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_RefTrig_Hyst ***
int32n __CFUNC DAQmxGetAnlgEdgeRefTrigHyst(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgEdgeRefTrigHyst(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgEdgeRefTrigHyst(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgEdge_RefTrig_Coupling ***
// Uses value set Coupling2
int32n __CFUNC DAQmxGetAnlgEdgeRefTrigCoupling(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgEdgeRefTrigCoupling(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgEdgeRefTrigCoupling(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_RefTrig_Src ***
int32n __CFUNC DAQmxGetAnlgWinRefTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAnlgWinRefTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetAnlgWinRefTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_RefTrig_When ***
// Uses value set WindowTriggerCondition1
int32n __CFUNC DAQmxGetAnlgWinRefTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgWinRefTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgWinRefTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_RefTrig_Top ***
int32n __CFUNC DAQmxGetAnlgWinRefTrigTop(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgWinRefTrigTop(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgWinRefTrigTop(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_RefTrig_Btm ***
int32n __CFUNC DAQmxGetAnlgWinRefTrigBtm(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgWinRefTrigBtm(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgWinRefTrigBtm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_RefTrig_Coupling ***
// Uses value set Coupling2
int32n __CFUNC DAQmxGetAnlgWinRefTrigCoupling(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgWinRefTrigCoupling(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgWinRefTrigCoupling(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AdvTrig_Type ***
// Uses value set TriggerType5
int32n __CFUNC DAQmxGetAdvTrigType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAdvTrigType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAdvTrigType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_AdvTrig_Src ***
int32n __CFUNC DAQmxGetDigEdgeAdvTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigEdgeAdvTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigEdgeAdvTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_AdvTrig_Edge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetDigEdgeAdvTrigEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigEdgeAdvTrigEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigEdgeAdvTrigEdge(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_AdvTrig_DigFltr_Enable ***
int32n __CFUNC DAQmxGetDigEdgeAdvTrigDigFltrEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetDigEdgeAdvTrigDigFltrEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetDigEdgeAdvTrigDigFltrEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_HshkTrig_Type ***
// Uses value set TriggerType9
int32n __CFUNC DAQmxGetHshkTrigType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetHshkTrigType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetHshkTrigType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Interlocked_HshkTrig_Src ***
int32n __CFUNC DAQmxGetInterlockedHshkTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetInterlockedHshkTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetInterlockedHshkTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Interlocked_HshkTrig_AssertedLvl ***
// Uses value set Level1
int32n __CFUNC DAQmxGetInterlockedHshkTrigAssertedLvl(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetInterlockedHshkTrigAssertedLvl(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetInterlockedHshkTrigAssertedLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_PauseTrig_Type ***
// Uses value set TriggerType6
int32n __CFUNC DAQmxGetPauseTrigType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetPauseTrigType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetPauseTrigType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgLvl_PauseTrig_Src ***
int32n __CFUNC DAQmxGetAnlgLvlPauseTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAnlgLvlPauseTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetAnlgLvlPauseTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgLvl_PauseTrig_When ***
// Uses value set ActiveLevel
int32n __CFUNC DAQmxGetAnlgLvlPauseTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgLvlPauseTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgLvlPauseTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgLvl_PauseTrig_Lvl ***
int32n __CFUNC DAQmxGetAnlgLvlPauseTrigLvl(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgLvlPauseTrigLvl(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgLvlPauseTrigLvl(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgLvl_PauseTrig_Hyst ***
int32n __CFUNC DAQmxGetAnlgLvlPauseTrigHyst(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgLvlPauseTrigHyst(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgLvlPauseTrigHyst(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgLvl_PauseTrig_Coupling ***
// Uses value set Coupling2
int32n __CFUNC DAQmxGetAnlgLvlPauseTrigCoupling(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgLvlPauseTrigCoupling(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgLvlPauseTrigCoupling(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_PauseTrig_Src ***
int32n __CFUNC DAQmxGetAnlgWinPauseTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetAnlgWinPauseTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetAnlgWinPauseTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_PauseTrig_When ***
// Uses value set WindowTriggerCondition2
int32n __CFUNC DAQmxGetAnlgWinPauseTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgWinPauseTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgWinPauseTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_PauseTrig_Top ***
int32n __CFUNC DAQmxGetAnlgWinPauseTrigTop(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgWinPauseTrigTop(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgWinPauseTrigTop(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_PauseTrig_Btm ***
int32n __CFUNC DAQmxGetAnlgWinPauseTrigBtm(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetAnlgWinPauseTrigBtm(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetAnlgWinPauseTrigBtm(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_AnlgWin_PauseTrig_Coupling ***
// Uses value set Coupling2
int32n __CFUNC DAQmxGetAnlgWinPauseTrigCoupling(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetAnlgWinPauseTrigCoupling(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetAnlgWinPauseTrigCoupling(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigLvl_PauseTrig_Src ***
int32n __CFUNC DAQmxGetDigLvlPauseTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigLvlPauseTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigLvlPauseTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigLvl_PauseTrig_When ***
// Uses value set Level1
int32n __CFUNC DAQmxGetDigLvlPauseTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigLvlPauseTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigLvlPauseTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigLvl_PauseTrig_DigFltr_Enable ***
int32n __CFUNC DAQmxGetDigLvlPauseTrigDigFltrEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetDigLvlPauseTrigDigFltrEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetDigLvlPauseTrigDigFltrEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigLvl_PauseTrig_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetDigLvlPauseTrigDigFltrMinPulseWidth(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetDigLvlPauseTrigDigFltrMinPulseWidth(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetDigLvlPauseTrigDigFltrMinPulseWidth(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigLvl_PauseTrig_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetDigLvlPauseTrigDigFltrTimebaseSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigLvlPauseTrigDigFltrTimebaseSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigLvlPauseTrigDigFltrTimebaseSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigLvl_PauseTrig_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetDigLvlPauseTrigDigFltrTimebaseRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetDigLvlPauseTrigDigFltrTimebaseRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetDigLvlPauseTrigDigFltrTimebaseRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigLvl_PauseTrig_DigSync_Enable ***
int32n __CFUNC DAQmxGetDigLvlPauseTrigDigSyncEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetDigLvlPauseTrigDigSyncEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetDigLvlPauseTrigDigSyncEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_PauseTrig_Src ***
int32n __CFUNC DAQmxGetDigPatternPauseTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigPatternPauseTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigPatternPauseTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_PauseTrig_Pattern ***
int32n __CFUNC DAQmxGetDigPatternPauseTrigPattern(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigPatternPauseTrigPattern(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigPatternPauseTrigPattern(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigPattern_PauseTrig_When ***
// Uses value set DigitalPatternCondition1
int32n __CFUNC DAQmxGetDigPatternPauseTrigWhen(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigPatternPauseTrigWhen(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigPatternPauseTrigWhen(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_ArmStartTrig_Type ***
// Uses value set TriggerType4
int32n __CFUNC DAQmxGetArmStartTrigType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetArmStartTrigType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetArmStartTrigType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_ArmStartTrig_Src ***
int32n __CFUNC DAQmxGetDigEdgeArmStartTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigEdgeArmStartTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigEdgeArmStartTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_ArmStartTrig_Edge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetDigEdgeArmStartTrigEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigEdgeArmStartTrigEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigEdgeArmStartTrigEdge(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_ArmStartTrig_DigFltr_Enable ***
int32n __CFUNC DAQmxGetDigEdgeArmStartTrigDigFltrEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetDigEdgeArmStartTrigDigFltrEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetDigEdgeArmStartTrigDigFltrEnable(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_ArmStartTrig_DigFltr_MinPulseWidth ***
int32n __CFUNC DAQmxGetDigEdgeArmStartTrigDigFltrMinPulseWidth(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetDigEdgeArmStartTrigDigFltrMinPulseWidth(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetDigEdgeArmStartTrigDigFltrMinPulseWidth(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_ArmStartTrig_DigFltr_TimebaseSrc ***
int32n __CFUNC DAQmxGetDigEdgeArmStartTrigDigFltrTimebaseSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigEdgeArmStartTrigDigFltrTimebaseSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigEdgeArmStartTrigDigFltrTimebaseSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_ArmStartTrig_DigFltr_TimebaseRate ***
int32n __CFUNC DAQmxGetDigEdgeArmStartTrigDigFltrTimebaseRate(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetDigEdgeArmStartTrigDigFltrTimebaseRate(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetDigEdgeArmStartTrigDigFltrTimebaseRate(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_ArmStartTrig_DigSync_Enable ***
int32n __CFUNC DAQmxGetDigEdgeArmStartTrigDigSyncEnable(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetDigEdgeArmStartTrigDigSyncEnable(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetDigEdgeArmStartTrigDigSyncEnable(TaskHandle taskHandle);

//********** Watchdog **********
//*** Set/Get functions for DAQmx_Watchdog_Timeout ***
int32n __CFUNC DAQmxGetWatchdogTimeout(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetWatchdogTimeout(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetWatchdogTimeout(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_WatchdogExpirTrig_Type ***
// Uses value set TriggerType4
int32n __CFUNC DAQmxGetWatchdogExpirTrigType(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetWatchdogExpirTrigType(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetWatchdogExpirTrigType(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_WatchdogExpirTrig_Src ***
int32n __CFUNC DAQmxGetDigEdgeWatchdogExpirTrigSrc(TaskHandle taskHandle, char *data, uint32n bufferSize);
int32n __CFUNC DAQmxSetDigEdgeWatchdogExpirTrigSrc(TaskHandle taskHandle, const char *data);
int32n __CFUNC DAQmxResetDigEdgeWatchdogExpirTrigSrc(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_DigEdge_WatchdogExpirTrig_Edge ***
// Uses value set Edge1
int32n __CFUNC DAQmxGetDigEdgeWatchdogExpirTrigEdge(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetDigEdgeWatchdogExpirTrigEdge(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetDigEdgeWatchdogExpirTrigEdge(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Watchdog_DO_ExpirState ***
// Uses value set DigitalLineState
int32n __CFUNC DAQmxGetWatchdogDOExpirState(TaskHandle taskHandle, const char lines[], int32n *data);
int32n __CFUNC DAQmxSetWatchdogDOExpirState(TaskHandle taskHandle, const char lines[], int32n data);
int32n __CFUNC DAQmxResetWatchdogDOExpirState(TaskHandle taskHandle, const char lines[]);
//*** Set/Get functions for DAQmx_Watchdog_HasExpired ***
int32n __CFUNC DAQmxGetWatchdogHasExpired(TaskHandle taskHandle, bool32n *data);

//********** Write **********
//*** Set/Get functions for DAQmx_Write_RelativeTo ***
// Uses value set WriteRelativeTo
int32n __CFUNC DAQmxGetWriteRelativeTo(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetWriteRelativeTo(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetWriteRelativeTo(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Write_Offset ***
int32n __CFUNC DAQmxGetWriteOffset(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetWriteOffset(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetWriteOffset(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Write_RegenMode ***
// Uses value set RegenerationMode1
int32n __CFUNC DAQmxGetWriteRegenMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetWriteRegenMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetWriteRegenMode(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Write_CurrWritePos ***
int32n __CFUNC DAQmxGetWriteCurrWritePos(TaskHandle taskHandle, uInt64n *data);
//*** Set/Get functions for DAQmx_Write_OvercurrentChansExist ***
int32n __CFUNC DAQmxGetWriteOvercurrentChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Write_OvercurrentChans ***
int32n __CFUNC DAQmxGetWriteOvercurrentChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Write_OvertemperatureChansExist ***
int32n __CFUNC DAQmxGetWriteOvertemperatureChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Write_OpenCurrentLoopChansExist ***
int32n __CFUNC DAQmxGetWriteOpenCurrentLoopChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Write_OpenCurrentLoopChans ***
int32n __CFUNC DAQmxGetWriteOpenCurrentLoopChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Write_PowerSupplyFaultChansExist ***
int32n __CFUNC DAQmxGetWritePowerSupplyFaultChansExist(TaskHandle taskHandle, bool32n *data);
//*** Set/Get functions for DAQmx_Write_PowerSupplyFaultChans ***
int32n __CFUNC DAQmxGetWritePowerSupplyFaultChans(TaskHandle taskHandle, char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_Write_SpaceAvail ***
int32n __CFUNC DAQmxGetWriteSpaceAvail(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Write_TotalSampPerChanGenerated ***
int32n __CFUNC DAQmxGetWriteTotalSampPerChanGenerated(TaskHandle taskHandle, uInt64n *data);
//*** Set/Get functions for DAQmx_Write_RawDataWidth ***
int32n __CFUNC DAQmxGetWriteRawDataWidth(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Write_NumChans ***
int32n __CFUNC DAQmxGetWriteNumChans(TaskHandle taskHandle, uint32n *data);
//*** Set/Get functions for DAQmx_Write_WaitMode ***
// Uses value set WaitMode2
int32n __CFUNC DAQmxGetWriteWaitMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetWriteWaitMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetWriteWaitMode(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Write_SleepTime ***
int32n __CFUNC DAQmxGetWriteSleepTime(TaskHandle taskHandle, float64n *data);
int32n __CFUNC DAQmxSetWriteSleepTime(TaskHandle taskHandle, float64n data);
int32n __CFUNC DAQmxResetWriteSleepTime(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Write_NextWriteIsLast ***
int32n __CFUNC DAQmxGetWriteNextWriteIsLast(TaskHandle taskHandle, bool32n *data);
int32n __CFUNC DAQmxSetWriteNextWriteIsLast(TaskHandle taskHandle, bool32n data);
int32n __CFUNC DAQmxResetWriteNextWriteIsLast(TaskHandle taskHandle);
//*** Set/Get functions for DAQmx_Write_DigitalLines_BytesPerChan ***
int32n __CFUNC DAQmxGetWriteDigitalLinesBytesPerChan(TaskHandle taskHandle, uint32n *data);

//********** Physical Channel **********
//*** Set/Get functions for DAQmx_PhysicalChan_AI_TermCfgs ***
// Uses bits from enum TerminalConfigurationBits
int32n __CFUNC DAQmxGetPhysicalChanAITermCfgs(const char physicalChannel[], int32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_AO_TermCfgs ***
// Uses bits from enum TerminalConfigurationBits
int32n __CFUNC DAQmxGetPhysicalChanAOTermCfgs(const char physicalChannel[], int32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_AO_ManualControlEnable ***
int32n __CFUNC DAQmxGetPhysicalChanAOManualControlEnable(const char physicalChannel[], bool32n *data);
int32n __CFUNC DAQmxSetPhysicalChanAOManualControlEnable(const char physicalChannel[], bool32n data);
int32n __CFUNC DAQmxResetPhysicalChanAOManualControlEnable(const char physicalChannel[]);
//*** Set/Get functions for DAQmx_PhysicalChan_AO_ManualControlAmplitude ***
int32n __CFUNC DAQmxGetPhysicalChanAOManualControlAmplitude(const char physicalChannel[], float64n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_AO_ManualControlFreq ***
int32n __CFUNC DAQmxGetPhysicalChanAOManualControlFreq(const char physicalChannel[], float64n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_DI_PortWidth ***
int32n __CFUNC DAQmxGetPhysicalChanDIPortWidth(const char physicalChannel[], uint32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_DI_SampClkSupported ***
int32n __CFUNC DAQmxGetPhysicalChanDISampClkSupported(const char physicalChannel[], bool32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_DI_ChangeDetectSupported ***
int32n __CFUNC DAQmxGetPhysicalChanDIChangeDetectSupported(const char physicalChannel[], bool32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_DO_PortWidth ***
int32n __CFUNC DAQmxGetPhysicalChanDOPortWidth(const char physicalChannel[], uint32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_DO_SampClkSupported ***
int32n __CFUNC DAQmxGetPhysicalChanDOSampClkSupported(const char physicalChannel[], bool32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_TEDS_MfgID ***
int32n __CFUNC DAQmxGetPhysicalChanTEDSMfgID(const char physicalChannel[], uint32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_TEDS_ModelNum ***
int32n __CFUNC DAQmxGetPhysicalChanTEDSModelNum(const char physicalChannel[], uint32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_TEDS_SerialNum ***
int32n __CFUNC DAQmxGetPhysicalChanTEDSSerialNum(const char physicalChannel[], uint32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_TEDS_VersionNum ***
int32n __CFUNC DAQmxGetPhysicalChanTEDSVersionNum(const char physicalChannel[], uint32n *data);
//*** Set/Get functions for DAQmx_PhysicalChan_TEDS_VersionLetter ***
int32n __CFUNC DAQmxGetPhysicalChanTEDSVersionLetter(const char physicalChannel[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_PhysicalChan_TEDS_BitStream ***
int32n __CFUNC DAQmxGetPhysicalChanTEDSBitStream(const char physicalChannel[], uInt8n *data, uint32n arraySizeInSamples);
//*** Set/Get functions for DAQmx_PhysicalChan_TEDS_TemplateIDs ***
int32n __CFUNC DAQmxGetPhysicalChanTEDSTemplateIDs(const char physicalChannel[], uint32n *data, uint32n arraySizeInSamples);

//********** Persisted Task **********
//*** Set/Get functions for DAQmx_PersistedTask_Author ***
int32n __CFUNC DAQmxGetPersistedTaskAuthor(const char taskName[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_PersistedTask_AllowInteractiveEditing ***
int32n __CFUNC DAQmxGetPersistedTaskAllowInteractiveEditing(const char taskName[], bool32n *data);
//*** Set/Get functions for DAQmx_PersistedTask_AllowInteractiveDeletion ***
int32n __CFUNC DAQmxGetPersistedTaskAllowInteractiveDeletion(const char taskName[], bool32n *data);

//********** Persisted Channel **********
//*** Set/Get functions for DAQmx_PersistedChan_Author ***
int32n __CFUNC DAQmxGetPersistedChanAuthor(const char channel[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_PersistedChan_AllowInteractiveEditing ***
int32n __CFUNC DAQmxGetPersistedChanAllowInteractiveEditing(const char channel[], bool32n *data);
//*** Set/Get functions for DAQmx_PersistedChan_AllowInteractiveDeletion ***
int32n __CFUNC DAQmxGetPersistedChanAllowInteractiveDeletion(const char channel[], bool32n *data);

//********** Persisted Scale **********
//*** Set/Get functions for DAQmx_PersistedScale_Author ***
int32n __CFUNC DAQmxGetPersistedScaleAuthor(const char scaleName[], char *data, uint32n bufferSize);
//*** Set/Get functions for DAQmx_PersistedScale_AllowInteractiveEditing ***
int32n __CFUNC DAQmxGetPersistedScaleAllowInteractiveEditing(const char scaleName[], bool32n *data);
//*** Set/Get functions for DAQmx_PersistedScale_AllowInteractiveDeletion ***
int32n __CFUNC DAQmxGetPersistedScaleAllowInteractiveDeletion(const char scaleName[], bool32n *data);

//*** Set/Get functions for DAQmx_SampClk_TimingResponseMode ***
// Uses value set TimingResponseMode
// Obsolete - always returns 0
int32n __CFUNC DAQmxGetSampClkTimingResponseMode(TaskHandle taskHandle, int32n *data);
int32n __CFUNC DAQmxSetSampClkTimingResponseMode(TaskHandle taskHandle, int32n data);
int32n __CFUNC DAQmxResetSampClkTimingResponseMode(TaskHandle taskHandle);

/******************************************************************************
 *** NI-DAQmx Error Codes *****************************************************
 ******************************************************************************/

#define DAQmxSuccess                                  (0)

#define DAQmxFailed(error)                            ((error)<0)

// Error and Warning Codes
#define DAQmxErrorCOCannotKeepUpInHWTimedSinglePoint                                    (-209805)
#define DAQmxErrorWaitForNextSampClkDetected3OrMoreSampClks                             (-209803)
#define DAQmxErrorWaitForNextSampClkDetectedMissedSampClk                               (-209802)
#define DAQmxErrorWriteNotCompleteBeforeSampClk                                         (-209801)
#define DAQmxErrorReadNotCompleteBeforeSampClk                                          (-209800)
#define DAQmxErrorNetworkDeviceInUse                                                    (-201273)
#define DAQmxErrorInvalidIPv4AddressFormat                                              (-201272)
#define DAQmxErrorNetworkProductTypeMismatch                                            (-201271)
#define DAQmxErrorOnlyPEMCertificatesAccepted                                           (-201270)
#define DAQmxErrorCalibrationRequiresPrototypingBoardEnabled                            (-201269)
#define DAQmxErrorAllCurrentLimitingResourcesAlreadyTaken                               (-201268)
#define DAQmxErrorUserDefInfoStringBadLength                                            (-201267)
#define DAQmxErrorPropertyNotFound                                                      (-201266)
#define DAQmxErrorOverVoltageProtectionActivated                                        (-201265)
#define DAQmxErrorScaledIQWaveformTooLarge                                              (-201264)
#define DAQmxErrorFirmwareFailedToDownload                                              (-201263)
#define DAQmxErrorPropertyNotSupportedForBusType                                        (-201262)
#define DAQmxErrorChangeRateWhileRunningCouldNotBeCompleted                             (-201261)
#define DAQmxErrorCannotQueryManualControlAttribute                                     (-201260)
#define DAQmxErrorInvalidNetworkConfiguration                                           (-201259)
#define DAQmxErrorInvalidWirelessConfiguration                                          (-201258)
#define DAQmxErrorInvalidWirelessCountryCode                                            (-201257)
#define DAQmxErrorInvalidWirelessChannel                                                (-201256)
#define DAQmxErrorNetworkEEPROMHasChanged                                               (-201255)
#define DAQmxErrorNetworkSerialNumberMismatch                                           (-201254)
#define DAQmxErrorNetworkStatusDown                                                     (-201253)
#define DAQmxErrorNetworkTargetUnreachable                                              (-201252)
#define DAQmxErrorNetworkTargetNotFound                                                 (-201251)
#define DAQmxErrorNetworkStatusTimedOut                                                 (-201250)
#define DAQmxErrorInvalidWirelessSecuritySelection                                      (-201249)
#define DAQmxErrorNetworkDeviceConfigurationLocked                                      (-201248)
#define DAQmxErrorNetworkDAQDeviceNotSupported                                          (-201247)
#define DAQmxErrorNetworkDAQCannotCreateEmptySleeve                                     (-201246)
#define DAQmxErrorModuleTypeDoesNotMatchModuleTypeInDestination                         (-201244)
#define DAQmxErrorInvalidTEDSInterfaceAddress                                           (-201243)
#define DAQmxErrorDevDoesNotSupportSCXIComm                                             (-201242)
#define DAQmxErrorSCXICommDevConnector0MustBeCabledToModule                             (-201241)
#define DAQmxErrorSCXIModuleDoesNotSupportDigitizationMode                              (-201240)
#define DAQmxErrorDevDoesNotSupportMultiplexedSCXIDigitizationMode                      (-201239)
#define DAQmxErrorDevOrDevPhysChanDoesNotSupportSCXIDigitization                        (-201238)
#define DAQmxErrorInvalidPhysChanName                                                   (-201237)
#define DAQmxErrorSCXIChassisCommModeInvalid                                            (-201236)
#define DAQmxErrorRequiredDependencyNotFound                                            (-201235)
#define DAQmxErrorInvalidStorage                                                        (-201234)
#define DAQmxErrorInvalidObject                                                         (-201233)
#define DAQmxErrorStorageAlteredPriorToSave                                             (-201232)
#define DAQmxErrorTaskDoesNotReferenceLocalChannel                                      (-201231)
#define DAQmxErrorReferencedDevSimMustMatchTarget                                       (-201230)
#define DAQmxErrorProgrammedIOFailsBecauseOfWatchdogTimer                               (-201229)
#define DAQmxErrorWatchdogTimerFailsBecauseOfProgrammedIO                               (-201228)
#define DAQmxErrorCantUseThisTimingEngineWithAPort                                      (-201227)
#define DAQmxErrorProgrammedIOConflict                                                  (-201226)
#define DAQmxErrorChangeDetectionIncompatibleWithProgrammedIO                           (-201225)
#define DAQmxErrorTristateNotEnoughLines                                                (-201224)
#define DAQmxErrorTristateConflict                                                      (-201223)
#define DAQmxErrorGenerateOrFiniteWaitExpectedBeforeBreakBlock                          (-201222)
#define DAQmxErrorBreakBlockNotAllowedInLoop                                            (-201221)
#define DAQmxErrorClearTriggerNotAllowedInBreakBlock                                    (-201220)
#define DAQmxErrorNestingNotAllowedInBreakBlock                                         (-201219)
#define DAQmxErrorIfElseBlockNotAllowedInBreakBlock                                     (-201218)
#define DAQmxErrorRepeatUntilTriggerLoopNotAllowedInBreakBlock                          (-201217)
#define DAQmxErrorWaitUntilTriggerNotAllowedInBreakBlock                                (-201216)
#define DAQmxErrorMarkerPosInvalidInBreakBlock                                          (-201215)
#define DAQmxErrorInvalidWaitDurationInBreakBlock                                       (-201214)
#define DAQmxErrorInvalidSubsetLengthInBreakBlock                                       (-201213)
#define DAQmxErrorInvalidWaveformLengthInBreakBlock                                     (-201212)
#define DAQmxErrorInvalidWaitDurationBeforeBreakBlock                                   (-201211)
#define DAQmxErrorInvalidSubsetLengthBeforeBreakBlock                                   (-201210)
#define DAQmxErrorInvalidWaveformLengthBeforeBreakBlock                                 (-201209)
#define DAQmxErrorSampleRateTooHighForADCTimingMode                                     (-201208)
#define DAQmxErrorActiveDevNotSupportedWithMultiDevTask                                 (-201207)
#define DAQmxErrorRealDevAndSimDevNotSupportedInSameTask                                (-201206)
#define DAQmxErrorRTSISimMustMatchDevSim                                                (-201205)
#define DAQmxErrorBridgeShuntCaNotSupported                                             (-201204)
#define DAQmxErrorStrainShuntCaNotSupported                                             (-201203)
#define DAQmxErrorGainTooLargeForGainCalConst                                           (-201202)
#define DAQmxErrorOffsetTooLargeForOffsetCalConst                                       (-201201)
#define DAQmxErrorElvisPrototypingBoardRemoved                                          (-201200)
#define DAQmxErrorElvis2PowerRailFault                                                  (-201199)
#define DAQmxErrorElvis2PhysicalChansFault                                              (-201198)
#define DAQmxErrorElvis2PhysicalChansThermalEvent                                       (-201197)
#define DAQmxErrorRXBitErrorRateLimitExceeded                                           (-201196)
#define DAQmxErrorPHYBitErrorRateLimitExceeded                                          (-201195)
#define DAQmxErrorTwoPartAttributeCalledOutOfOrder                                      (-201194)
#define DAQmxErrorInvalidSCXIChassisAddress                                             (-201193)
#define DAQmxErrorCouldNotConnectToRemoteMXS                                            (-201192)
#define DAQmxErrorExcitationStateRequiredForAttributes                                  (-201191)
#define DAQmxErrorDeviceNotUsableUntilUSBReplug                                         (-201190)
#define DAQmxErrorInputFIFOOverflowDuringCalibrationOnFullSpeedUSB                      (-201189)
#define DAQmxErrorInputFIFOOverflowDuringCalibration                                    (-201188)
#define DAQmxErrorCJCChanConflictsWithNonThermocoupleChan                               (-201187)
#define DAQmxErrorCommDeviceForPXIBackplaneNotInRightmostSlot                           (-201186)
#define DAQmxErrorCommDeviceForPXIBackplaneNotInSameChassis                             (-201185)
#define DAQmxErrorCommDeviceForPXIBackplaneNotPXI                                       (-201184)
#define DAQmxErrorInvalidCalExcitFrequency                                              (-201183)
#define DAQmxErrorInvalidCalExcitVoltage                                                (-201182)
#define DAQmxErrorInvalidAIInputSrc                                                     (-201181)
#define DAQmxErrorInvalidCalInputRef                                                    (-201180)
#define DAQmxErrordBReferenceValueNotGreaterThanZero                                    (-201179)
#define DAQmxErrorSampleClockRateIsTooFastForSampleClockTiming                          (-201178)
#define DAQmxErrorDeviceNotUsableUntilColdStart                                         (-201177)
#define DAQmxErrorSampleClockRateIsTooFastForBurstTiming                                (-201176)
#define DAQmxErrorDevImportFailedAssociatedResourceIDsNotSupported                      (-201175)
#define DAQmxErrorSCXI1600ImportNotSupported                                            (-201174)
#define DAQmxErrorPowerSupplyConfigurationFailed                                        (-201173)
#define DAQmxErrorIEPEWithDCNotAllowed                                                  (-201172)
#define DAQmxErrorMinTempForThermocoupleTypeOutsideAccuracyForPolyScaling               (-201171)
#define DAQmxErrorDevImportFailedNoDeviceToOverwriteAndSimulationNotSupported           (-201170)
#define DAQmxErrorDevImportFailedDeviceNotSupportedOnDestination                        (-201169)
#define DAQmxErrorFirmwareIsTooOld                                                      (-201168)
#define DAQmxErrorFirmwareCouldntUpdate                                                 (-201167)
#define DAQmxErrorFirmwareIsCorrupt                                                     (-201166)
#define DAQmxErrorFirmwareTooNew                                                        (-201165)
#define DAQmxErrorSampClockCannotBeExportedFromExternalSampClockSrc                     (-201164)
#define DAQmxErrorPhysChanReservedForInputWhenDesiredForOutput                          (-201163)
#define DAQmxErrorPhysChanReservedForOutputWhenDesiredForInput                          (-201162)
#define DAQmxErrorSpecifiedCDAQSlotNotEmpty                                             (-201161)
#define DAQmxErrorDeviceDoesNotSupportSimulation                                        (-201160)
#define DAQmxErrorInvalidCDAQSlotNumberSpecd                                            (-201159)
#define DAQmxErrorCSeriesModSimMustMatchCDAQChassisSim                                  (-201158)
#define DAQmxErrorSCCCabledDevMustNotBeSimWhenSCCCarrierIsNotSim                        (-201157)
#define DAQmxErrorSCCModSimMustMatchSCCCarrierSim                                       (-201156)
#define DAQmxErrorSCXIModuleDoesNotSupportSimulation                                    (-201155)
#define DAQmxErrorSCXICableDevMustNotBeSimWhenModIsNotSim                               (-201154)
#define DAQmxErrorSCXIDigitizerSimMustNotBeSimWhenModIsNotSim                           (-201153)
#define DAQmxErrorSCXIModSimMustMatchSCXIChassisSim                                     (-201152)
#define DAQmxErrorSimPXIDevReqSlotAndChassisSpecd                                       (-201151)
#define DAQmxErrorSimDevConflictWithRealDev                                             (-201150)
#define DAQmxErrorInsufficientDataForCalibration                                        (-201149)
#define DAQmxErrorTriggerChannelMustBeEnabled                                           (-201148)
#define DAQmxErrorCalibrationDataConflictCouldNotBeResolved                             (-201147)
#define DAQmxErrorSoftwareTooNewForSelfCalibrationData                                  (-201146)
#define DAQmxErrorSoftwareTooNewForExtCalibrationData                                   (-201145)
#define DAQmxErrorSelfCalibrationDataTooNewForSoftware                                  (-201144)
#define DAQmxErrorExtCalibrationDataTooNewForSoftware                                   (-201143)
#define DAQmxErrorSoftwareTooNewForEEPROM                                               (-201142)
#define DAQmxErrorEEPROMTooNewForSoftware                                               (-201141)
#define DAQmxErrorSoftwareTooNewForHardware                                             (-201140)
#define DAQmxErrorHardwareTooNewForSoftware                                             (-201139)
#define DAQmxErrorTaskCannotRestartFirstSampNotAvailToGenerate                          (-201138)
#define DAQmxErrorOnlyUseStartTrigSrcPrptyWithDevDataLines                              (-201137)
#define DAQmxErrorOnlyUsePauseTrigSrcPrptyWithDevDataLines                              (-201136)
#define DAQmxErrorOnlyUseRefTrigSrcPrptyWithDevDataLines                                (-201135)
#define DAQmxErrorPauseTrigDigPatternSizeDoesNotMatchSrcSize                            (-201134)
#define DAQmxErrorLineConflictCDAQ                                                      (-201133)
#define DAQmxErrorCannotWriteBeyondFinalFiniteSample                                    (-201132)
#define DAQmxErrorRefAndStartTriggerSrcCantBeSame                                       (-201131)
#define DAQmxErrorMemMappingIncompatibleWithPhysChansInTask                             (-201130)
#define DAQmxErrorOutputDriveTypeMemMappingConflict                                     (-201129)
#define DAQmxErrorCAPIDeviceIndexInvalid                                                (-201128)
#define DAQmxErrorRatiometricDevicesMustUseExcitationForScaling                         (-201127)
#define DAQmxErrorPropertyRequiresPerDeviceCfg                                          (-201126)
#define DAQmxErrorAICouplingAndAIInputSourceConflict                                    (-201125)
#define DAQmxErrorOnlyOneTaskCanPerformDOMemoryMappingAtATime                           (-201124)
#define DAQmxErrorTooManyChansForAnalogRefTrigCDAQ                                      (-201123)
#define DAQmxErrorSpecdPropertyValueIsIncompatibleWithSampleTimingType                  (-201122)
#define DAQmxErrorCPUNotSupportedRequireSSE                                             (-201121)
#define DAQmxErrorSpecdPropertyValueIsIncompatibleWithSampleTimingResponseMode          (-201120)
#define DAQmxErrorConflictingNextWriteIsLastAndRegenModeProperties                      (-201119)
#define DAQmxErrorMStudioOperationDoesNotSupportDeviceContext                           (-201118)
#define DAQmxErrorPropertyValueInChannelExpansionContextInvalid                         (-201117)
#define DAQmxErrorHWTimedNonBufferedAONotSupported                                      (-201116)
#define DAQmxErrorWaveformLengthNotMultOfQuantum                                        (-201115)
#define DAQmxErrorDSAExpansionMixedBoardsWrongOrderInPXIChassis                         (-201114)
#define DAQmxErrorPowerLevelTooLowForOOK                                                (-201113)
#define DAQmxErrorDeviceComponentTestFailure                                            (-201112)
#define DAQmxErrorUserDefinedWfmWithOOKUnsupported                                      (-201111)
#define DAQmxErrorInvalidDigitalModulationUserDefinedWaveform                           (-201110)
#define DAQmxErrorBothRefInAndRefOutEnabled                                             (-201109)
#define DAQmxErrorBothAnalogAndDigitalModulationEnabled                                 (-201108)
#define DAQmxErrorBufferedOpsNotSupportedInSpecdSlotForCDAQ                             (-201107)
#define DAQmxErrorPhysChanNotSupportedInSpecdSlotForCDAQ                                (-201106)
#define DAQmxErrorResourceReservedWithConflictingSettings                               (-201105)
#define DAQmxErrorInconsistentAnalogTrigSettingsCDAQ                                    (-201104)
#define DAQmxErrorTooManyChansForAnalogPauseTrigCDAQ                                    (-201103)
#define DAQmxErrorAnalogTrigNotFirstInScanListCDAQ                                      (-201102)
#define DAQmxErrorTooManyChansGivenTimingType                                           (-201101)
#define DAQmxErrorSampClkTimebaseDivWithExtSampClk                                      (-201100)
#define DAQmxErrorCantSaveTaskWithPerDeviceTimingProperties                             (-201099)
#define DAQmxErrorConflictingAutoZeroMode                                               (-201098)
#define DAQmxErrorSampClkRateNotSupportedWithEAREnabled                                 (-201097)
#define DAQmxErrorSampClkTimebaseRateNotSpecd                                           (-201096)
#define DAQmxErrorSessionCorruptedByDLLReload                                           (-201095)
#define DAQmxErrorActiveDevNotSupportedWithChanExpansion                                (-201094)
#define DAQmxErrorSampClkRateInvalid                                                    (-201093)
#define DAQmxErrorExtSyncPulseSrcCannotBeExported                                       (-201092)
#define DAQmxErrorSyncPulseMinDelayToStartNeededForExtSyncPulseSrc                      (-201091)
#define DAQmxErrorSyncPulseSrcInvalid                                                   (-201090)
#define DAQmxErrorSampClkTimebaseRateInvalid                                            (-201089)
#define DAQmxErrorSampClkTimebaseSrcInvalid                                             (-201088)
#define DAQmxErrorSampClkRateMustBeSpecd                                                (-201087)
#define DAQmxErrorInvalidAttributeName                                                  (-201086)
#define DAQmxErrorCJCChanNameMustBeSetWhenCJCSrcIsScannableChan                         (-201085)
#define DAQmxErrorHiddenChanMissingInChansPropertyInCfgFile                             (-201084)
#define DAQmxErrorChanNamesNotSpecdInCfgFile                                            (-201083)
#define DAQmxErrorDuplicateHiddenChanNamesInCfgFile                                     (-201082)
#define DAQmxErrorDuplicateChanNameInCfgFile                                            (-201081)
#define DAQmxErrorInvalidSCCModuleForSlotSpecd                                          (-201080)
#define DAQmxErrorInvalidSCCSlotNumberSpecd                                             (-201079)
#define DAQmxErrorInvalidSectionIdentifier                                              (-201078)
#define DAQmxErrorInvalidSectionName                                                    (-201077)
#define DAQmxErrorDAQmxVersionNotSupported                                              (-201076)
#define DAQmxErrorSWObjectsFoundInFile                                                  (-201075)
#define DAQmxErrorHWObjectsFoundInFile                                                  (-201074)
#define DAQmxErrorLocalChannelSpecdWithNoParentTask                                     (-201073)
#define DAQmxErrorTaskReferencesMissingLocalChannel                                     (-201072)
#define DAQmxErrorTaskReferencesLocalChannelFromOtherTask                               (-201071)
#define DAQmxErrorTaskMissingChannelProperty                                            (-201070)
#define DAQmxErrorInvalidLocalChanName                                                  (-201069)
#define DAQmxErrorInvalidEscapeCharacterInString                                        (-201068)
#define DAQmxErrorInvalidTableIdentifier                                                (-201067)
#define DAQmxErrorValueFoundInInvalidColumn                                             (-201066)
#define DAQmxErrorMissingStartOfTable                                                   (-201065)
#define DAQmxErrorFileMissingRequiredDAQmxHeader                                        (-201064)
#define DAQmxErrorDeviceIDDoesNotMatch                                                  (-201063)
#define DAQmxErrorBufferedOperationsNotSupportedOnSelectedLines                         (-201062)
#define DAQmxErrorPropertyConflictsWithScale                                            (-201061)
#define DAQmxErrorInvalidINIFileSyntax                                                  (-201060)
#define DAQmxErrorDeviceInfoFailedPXIChassisNotIdentified                               (-201059)
#define DAQmxErrorInvalidHWProductNumber                                                (-201058)
#define DAQmxErrorInvalidHWProductType                                                  (-201057)
#define DAQmxErrorInvalidNumericFormatSpecd                                             (-201056)
#define DAQmxErrorDuplicatePropertyInObject                                             (-201055)
#define DAQmxErrorInvalidEnumValueSpecd                                                 (-201054)
#define DAQmxErrorTEDSSensorPhysicalChannelConflict                                     (-201053)
#define DAQmxErrorTooManyPhysicalChansForTEDSInterfaceSpecd                             (-201052)
#define DAQmxErrorIncapableTEDSInterfaceControllingDeviceSpecd                          (-201051)
#define DAQmxErrorSCCCarrierSpecdIsMissing                                              (-201050)
#define DAQmxErrorIncapableSCCDigitizingDeviceSpecd                                     (-201049)
#define DAQmxErrorAccessorySettingNotApplicable                                         (-201048)
#define DAQmxErrorDeviceAndConnectorSpecdAlreadyOccupied                                (-201047)
#define DAQmxErrorIllegalAccessoryTypeForDeviceSpecd                                    (-201046)
#define DAQmxErrorInvalidDeviceConnectorNumberSpecd                                     (-201045)
#define DAQmxErrorInvalidAccessoryName                                                  (-201044)
#define DAQmxErrorMoreThanOneMatchForSpecdDevice                                        (-201043)
#define DAQmxErrorNoMatchForSpecdDevice                                                 (-201042)
#define DAQmxErrorProductTypeAndProductNumberConflict                                   (-201041)
#define DAQmxErrorExtraPropertyDetectedInSpecdObject                                    (-201040)
#define DAQmxErrorRequiredPropertyMissing                                               (-201039)
#define DAQmxErrorCantSetAuthorForLocalChan                                             (-201038)
#define DAQmxErrorInvalidTimeValue                                                      (-201037)
#define DAQmxErrorInvalidTimeFormat                                                     (-201036)
#define DAQmxErrorDigDevChansSpecdInModeOtherThanParallel                               (-201035)
#define DAQmxErrorCascadeDigitizationModeNotSupported                                   (-201034)
#define DAQmxErrorSpecdSlotAlreadyOccupied                                              (-201033)
#define DAQmxErrorInvalidSCXISlotNumberSpecd                                            (-201032)
#define DAQmxErrorAddressAlreadyInUse                                                   (-201031)
#define DAQmxErrorSpecdDeviceDoesNotSupportRTSI                                         (-201030)
#define DAQmxErrorSpecdDeviceIsAlreadyOnRTSIBus                                         (-201029)
#define DAQmxErrorIdentifierInUse                                                       (-201028)
#define DAQmxErrorWaitForNextSampleClockOrReadDetected3OrMoreMissedSampClks             (-201027)
#define DAQmxErrorHWTimedAndDataXferPIO                                                 (-201026)
#define DAQmxErrorNonBufferedAndHWTimed                                                 (-201025)
#define DAQmxErrorCTROutSampClkPeriodShorterThanGenPulseTrainPeriodPolled               (-201024)
#define DAQmxErrorCTROutSampClkPeriodShorterThanGenPulseTrainPeriod2                    (-201023)
#define DAQmxErrorCOCannotKeepUpInHWTimedSinglePointPolled                              (-201022)
#define DAQmxErrorWriteRecoveryCannotKeepUpInHWTimedSinglePoint                         (-201021)
#define DAQmxErrorNoChangeDetectionOnSelectedLineForDevice                              (-201020)
#define DAQmxErrorSMIOPauseTriggersNotSupportedWithChannelExpansion                     (-201019)
#define DAQmxErrorClockMasterForExternalClockNotLongestPipeline                         (-201018)
#define DAQmxErrorUnsupportedUnicodeByteOrderMarker                                     (-201017)
#define DAQmxErrorTooManyInstructionsInLoopInScript                                     (-201016)
#define DAQmxErrorPLLNotLocked                                                          (-201015)
#define DAQmxErrorIfElseBlockNotAllowedInFiniteRepeatLoopInScript                       (-201014)
#define DAQmxErrorIfElseBlockNotAllowedInConditionalRepeatLoopInScript                  (-201013)
#define DAQmxErrorClearIsLastInstructionInIfElseBlockInScript                           (-201012)
#define DAQmxErrorInvalidWaitDurationBeforeIfElseBlockInScript                          (-201011)
#define DAQmxErrorMarkerPosInvalidBeforeIfElseBlockInScript                             (-201010)
#define DAQmxErrorInvalidSubsetLengthBeforeIfElseBlockInScript                          (-201009)
#define DAQmxErrorInvalidWaveformLengthBeforeIfElseBlockInScript                        (-201008)
#define DAQmxErrorGenerateOrFiniteWaitInstructionExpectedBeforeIfElseBlockInScript      (-201007)
#define DAQmxErrorCalPasswordNotSupported                                               (-201006)
#define DAQmxErrorSetupCalNeededBeforeAdjustCal                                         (-201005)
#define DAQmxErrorMultipleChansNotSupportedDuringCalSetup                               (-201004)
#define DAQmxErrorDevCannotBeAccessed                                                   (-201003)
#define DAQmxErrorSampClkRateDoesntMatchSampClkSrc                                      (-201002)
#define DAQmxErrorSampClkRateNotSupportedWithEARDisabled                                (-201001)
#define DAQmxErrorLabVIEWVersionDoesntSupportDAQmxEvents                                (-201000)
#define DAQmxErrorCOReadyForNewValNotSupportedWithOnDemand                              (-200999)
#define DAQmxErrorCIHWTimedSinglePointNotSupportedForMeasType                           (-200998)
#define DAQmxErrorOnDemandNotSupportedWithHWTimedSinglePoint                            (-200997)
#define DAQmxErrorHWTimedSinglePointAndDataXferNotProgIO                                (-200996)
#define DAQmxErrorMemMapAndHWTimedSinglePoint                                           (-200995)
#define DAQmxErrorCannotSetPropertyWhenHWTimedSinglePointTaskIsRunning                  (-200994)
#define DAQmxErrorCTROutSampClkPeriodShorterThanGenPulseTrainPeriod                     (-200993)
#define DAQmxErrorTooManyEventsGenerated                                                (-200992)
#define DAQmxErrorMStudioCppRemoveEventsBeforeStop                                      (-200991)
#define DAQmxErrorCAPICannotRegisterSyncEventsFromMultipleThreads                       (-200990)
#define DAQmxErrorReadWaitNextSampClkWaitMismatchTwo                                    (-200989)
#define DAQmxErrorReadWaitNextSampClkWaitMismatchOne                                    (-200988)
#define DAQmxErrorDAQmxSignalEventTypeNotSupportedByChanTypesOrDevicesInTask            (-200987)
#define DAQmxErrorCannotUnregisterDAQmxSoftwareEventWhileTaskIsRunning                  (-200986)
#define DAQmxErrorAutoStartWriteNotAllowedEventRegistered                               (-200985)
#define DAQmxErrorAutoStartReadNotAllowedEventRegistered                                (-200984)
#define DAQmxErrorCannotGetPropertyWhenTaskNotReservedCommittedOrRunning                (-200983)
#define DAQmxErrorSignalEventsNotSupportedByDevice                                      (-200982)
#define DAQmxErrorEveryNSamplesAcqIntoBufferEventNotSupportedByDevice                   (-200981)
#define DAQmxErrorEveryNSampsTransferredFromBufferEventNotSupportedByDevice             (-200980)
#define DAQmxErrorCAPISyncEventsTaskStateChangeNotAllowedFromDifferentThread            (-200979)
#define DAQmxErrorDAQmxSWEventsWithDifferentCallMechanisms                              (-200978)
#define DAQmxErrorCantSaveChanWithPolyCalScaleAndAllowInteractiveEdit                   (-200977)
#define DAQmxErrorChanDoesNotSupportCJC                                                 (-200976)
#define DAQmxErrorCOReadyForNewValNotSupportedWithHWTimedSinglePoint                    (-200975)
#define DAQmxErrorDACAllowConnToGndNotSupportedByDevWhenRefSrcExt                       (-200974)
#define DAQmxErrorCantGetPropertyTaskNotRunning                                         (-200973)
#define DAQmxErrorCantSetPropertyTaskNotRunning                                         (-200972)
#define DAQmxErrorCantSetPropertyTaskNotRunningCommitted                                (-200971)
#define DAQmxErrorAIEveryNSampsEventIntervalNotMultipleOf2                              (-200970)
#define DAQmxErrorInvalidTEDSPhysChanNotAI                                              (-200969)
#define DAQmxErrorCAPICannotPerformTaskOperationInAsyncCallback                         (-200968)
#define DAQmxErrorEveryNSampsTransferredFromBufferEventAlreadyRegistered                (-200967)
#define DAQmxErrorEveryNSampsAcqIntoBufferEventAlreadyRegistered                        (-200966)
#define DAQmxErrorEveryNSampsTransferredFromBufferNotForInput                           (-200965)
#define DAQmxErrorEveryNSampsAcqIntoBufferNotForOutput                                  (-200964)
#define DAQmxErrorAOSampTimingTypeDifferentIn2Tasks                                     (-200963)
#define DAQmxErrorCouldNotDownloadFirmwareHWDamaged                                     (-200962)
#define DAQmxErrorCouldNotDownloadFirmwareFileMissingOrDamaged                          (-200961)
#define DAQmxErrorCannotRegisterDAQmxSoftwareEventWhileTaskIsRunning                    (-200960)
#define DAQmxErrorDifferentRawDataCompression                                           (-200959)
#define DAQmxErrorConfiguredTEDSInterfaceDevNotDetected                                 (-200958)
#define DAQmxErrorCompressedSampSizeExceedsResolution                                   (-200957)
#define DAQmxErrorChanDoesNotSupportCompression                                         (-200956)
#define DAQmxErrorDifferentRawDataFormats                                               (-200955)
#define DAQmxErrorSampClkOutputTermIncludesStartTrigSrc                                 (-200954)
#define DAQmxErrorStartTrigSrcEqualToSampClkSrc                                         (-200953)
#define DAQmxErrorEventOutputTermIncludesTrigSrc                                        (-200952)
#define DAQmxErrorCOMultipleWritesBetweenSampClks                                       (-200951)
#define DAQmxErrorDoneEventAlreadyRegistered                                            (-200950)
#define DAQmxErrorSignalEventAlreadyRegistered                                          (-200949)
#define DAQmxErrorCannotHaveTimedLoopAndDAQmxSignalEventsInSameTask                     (-200948)
#define DAQmxErrorNeedLabVIEW711PatchToUseDAQmxEvents                                   (-200947)
#define DAQmxErrorStartFailedDueToWriteFailure                                          (-200946)
#define DAQmxErrorDataXferCustomThresholdNotDMAXferMethodSpecifiedForDev                (-200945)
#define DAQmxErrorDataXferRequestConditionNotSpecifiedForCustomThreshold                (-200944)
#define DAQmxErrorDataXferCustomThresholdNotSpecified                                   (-200943)
#define DAQmxErrorCAPISyncCallbackNotSupportedOnThisPlatform                            (-200942)
#define DAQmxErrorCalChanReversePolyCoefNotSpecd                                        (-200941)
#define DAQmxErrorCalChanForwardPolyCoefNotSpecd                                        (-200940)
#define DAQmxErrorChanCalRepeatedNumberInPreScaledVals                                  (-200939)
#define DAQmxErrorChanCalTableNumScaledNotEqualNumPrescaledVals                         (-200938)
#define DAQmxErrorChanCalTableScaledValsNotSpecd                                        (-200937)
#define DAQmxErrorChanCalTablePreScaledValsNotSpecd                                     (-200936)
#define DAQmxErrorChanCalScaleTypeNotSet                                                (-200935)
#define DAQmxErrorChanCalExpired                                                        (-200934)
#define DAQmxErrorChanCalExpirationDateNotSet                                           (-200933)
#define DAQmxError3OutputPortCombinationGivenSampTimingType653x                         (-200932)
#define DAQmxError3InputPortCombinationGivenSampTimingType653x                          (-200931)
#define DAQmxError2OutputPortCombinationGivenSampTimingType653x                         (-200930)
#define DAQmxError2InputPortCombinationGivenSampTimingType653x                          (-200929)
#define DAQmxErrorPatternMatcherMayBeUsedByOneTrigOnly                                  (-200928)
#define DAQmxErrorNoChansSpecdForPatternSource                                          (-200927)
#define DAQmxErrorChangeDetectionChanNotInTask                                          (-200926)
#define DAQmxErrorChangeDetectionChanNotTristated                                       (-200925)
#define DAQmxErrorWaitModeValueNotSupportedNonBuffered                                  (-200924)
#define DAQmxErrorWaitModePropertyNotSupportedNonBuffered                               (-200923)
#define DAQmxErrorCantSavePerLineConfigDigChanSoInteractiveEditsAllowed                 (-200922)
#define DAQmxErrorCantSaveNonPortMultiLineDigChanSoInteractiveEditsAllowed              (-200921)
#define DAQmxErrorBufferSizeNotMultipleOfEveryNSampsEventIntervalNoIrqOnDev             (-200920)
#define DAQmxErrorGlobalTaskNameAlreadyChanName                                         (-200919)
#define DAQmxErrorGlobalChanNameAlreadyTaskName                                         (-200918)
#define DAQmxErrorAOEveryNSampsEventIntervalNotMultipleOf2                              (-200917)
#define DAQmxErrorSampleTimebaseDivisorNotSupportedGivenTimingType                      (-200916)
#define DAQmxErrorHandshakeEventOutputTermNotSupportedGivenTimingType                   (-200915)
#define DAQmxErrorChangeDetectionOutputTermNotSupportedGivenTimingType                  (-200914)
#define DAQmxErrorReadyForTransferOutputTermNotSupportedGivenTimingType                 (-200913)
#define DAQmxErrorRefTrigOutputTermNotSupportedGivenTimingType                          (-200912)
#define DAQmxErrorStartTrigOutputTermNotSupportedGivenTimingType                        (-200911)
#define DAQmxErrorSampClockOutputTermNotSupportedGivenTimingType                        (-200910)
#define DAQmxError20MhzTimebaseNotSupportedGivenTimingType                              (-200909)
#define DAQmxErrorSampClockSourceNotSupportedGivenTimingType                            (-200908)
#define DAQmxErrorRefTrigTypeNotSupportedGivenTimingType                                (-200907)
#define DAQmxErrorPauseTrigTypeNotSupportedGivenTimingType                              (-200906)
#define DAQmxErrorHandshakeTrigTypeNotSupportedGivenTimingType                          (-200905)
#define DAQmxErrorStartTrigTypeNotSupportedGivenTimingType                              (-200904)
#define DAQmxErrorRefClkSrcNotSupported                                                 (-200903)
#define DAQmxErrorDataVoltageLowAndHighIncompatible                                     (-200902)
#define DAQmxErrorInvalidCharInDigPatternString                                         (-200901)
#define DAQmxErrorCantUsePort3AloneGivenSampTimingTypeOn653x                            (-200900)
#define DAQmxErrorCantUsePort1AloneGivenSampTimingTypeOn653x                            (-200899)
#define DAQmxErrorPartialUseOfPhysicalLinesWithinPortNotSupported653x                   (-200898)
#define DAQmxErrorPhysicalChanNotSupportedGivenSampTimingType653x                       (-200897)
#define DAQmxErrorCanExportOnlyDigEdgeTrigs                                             (-200896)
#define DAQmxErrorRefTrigDigPatternSizeDoesNotMatchSourceSize                           (-200895)
#define DAQmxErrorStartTrigDigPatternSizeDoesNotMatchSourceSize                         (-200894)
#define DAQmxErrorChangeDetectionRisingAndFallingEdgeChanDontMatch                      (-200893)
#define DAQmxErrorPhysicalChansForChangeDetectionAndPatternMatch653x                    (-200892)
#define DAQmxErrorCanExportOnlyOnboardSampClk                                           (-200891)
#define DAQmxErrorInternalSampClkNotRisingEdge                                          (-200890)
#define DAQmxErrorRefTrigDigPatternChanNotInTask                                        (-200889)
#define DAQmxErrorRefTrigDigPatternChanNotTristated                                     (-200888)
#define DAQmxErrorStartTrigDigPatternChanNotInTask                                      (-200887)
#define DAQmxErrorStartTrigDigPatternChanNotTristated                                   (-200886)
#define DAQmxErrorPXIStarAndClock10Sync                                                 (-200885)
#define DAQmxErrorGlobalChanCannotBeSavedSoInteractiveEditsAllowed                      (-200884)
#define DAQmxErrorTaskCannotBeSavedSoInteractiveEditsAllowed                            (-200883)
#define DAQmxErrorInvalidGlobalChan                                                     (-200882)
#define DAQmxErrorEveryNSampsEventAlreadyRegistered                                     (-200881)
#define DAQmxErrorEveryNSampsEventIntervalZeroNotSupported                              (-200880)
#define DAQmxErrorChanSizeTooBigForU16PortWrite                                         (-200879)
#define DAQmxErrorChanSizeTooBigForU16PortRead                                          (-200878)
#define DAQmxErrorBufferSizeNotMultipleOfEveryNSampsEventIntervalWhenDMA                (-200877)
#define DAQmxErrorWriteWhenTaskNotRunningCOTicks                                        (-200876)
#define DAQmxErrorWriteWhenTaskNotRunningCOFreq                                         (-200875)
#define DAQmxErrorWriteWhenTaskNotRunningCOTime                                         (-200874)
#define DAQmxErrorAOMinMaxNotSupportedDACRangeTooSmall                                  (-200873)
#define DAQmxErrorAOMinMaxNotSupportedGivenDACRange                                     (-200872)
#define DAQmxErrorAOMinMaxNotSupportedGivenDACRangeAndOffsetVal                         (-200871)
#define DAQmxErrorAOMinMaxNotSupportedDACOffsetValInappropriate                         (-200870)
#define DAQmxErrorAOMinMaxNotSupportedGivenDACOffsetVal                                 (-200869)
#define DAQmxErrorAOMinMaxNotSupportedDACRefValTooSmall                                 (-200868)
#define DAQmxErrorAOMinMaxNotSupportedGivenDACRefVal                                    (-200867)
#define DAQmxErrorAOMinMaxNotSupportedGivenDACRefAndOffsetVal                           (-200866)
#define DAQmxErrorWhenAcqCompAndNumSampsPerChanExceedsOnBrdBufSize                      (-200865)
#define DAQmxErrorWhenAcqCompAndNoRefTrig                                               (-200864)
#define DAQmxErrorWaitForNextSampClkNotSupported                                        (-200863)
#define DAQmxErrorDevInUnidentifiedPXIChassis                                           (-200862)
#define DAQmxErrorMaxSoundPressureMicSensitivitRelatedAIPropertiesNotSupportedByDev     (-200861)
#define DAQmxErrorMaxSoundPressureAndMicSensitivityNotSupportedByDev                    (-200860)
#define DAQmxErrorAOBufferSizeZeroForSampClkTimingType                                  (-200859)
#define DAQmxErrorAOCallWriteBeforeStartForSampClkTimingType                            (-200858)
#define DAQmxErrorInvalidCalLowPassCutoffFreq                                           (-200857)
#define DAQmxErrorSimulationCannotBeDisabledForDevCreatedAsSimulatedDev                 (-200856)
#define DAQmxErrorCannotAddNewDevsAfterTaskConfiguration                                (-200855)
#define DAQmxErrorDifftSyncPulseSrcAndSampClkTimebaseSrcDevMultiDevTask                 (-200854)
#define DAQmxErrorTermWithoutDevInMultiDevTask                                          (-200853)
#define DAQmxErrorSyncNoDevSampClkTimebaseOrSyncPulseInPXISlot2                         (-200852)
#define DAQmxErrorPhysicalChanNotOnThisConnector                                        (-200851)
#define DAQmxErrorNumSampsToWaitNotGreaterThanZeroInScript                              (-200850)
#define DAQmxErrorNumSampsToWaitNotMultipleOfAlignmentQuantumInScript                   (-200849)
#define DAQmxErrorEveryNSamplesEventNotSupportedForNonBufferedTasks                     (-200848)
#define DAQmxErrorBufferedAndDataXferPIO                                                (-200847)
#define DAQmxErrorCannotWriteWhenAutoStartFalseAndTaskNotRunning                        (-200846)
#define DAQmxErrorNonBufferedAndDataXferInterrupts                                      (-200845)
#define DAQmxErrorWriteFailedMultipleCtrsWithFREQOUT                                    (-200844)
#define DAQmxErrorReadNotCompleteBefore3SampClkEdges                                    (-200843)
#define DAQmxErrorCtrHWTimedSinglePointAndDataXferNotProgIO                             (-200842)
#define DAQmxErrorPrescalerNot1ForInputTerminal                                         (-200841)
#define DAQmxErrorPrescalerNot1ForTimebaseSrc                                           (-200840)
#define DAQmxErrorSampClkTimingTypeWhenTristateIsFalse                                  (-200839)
#define DAQmxErrorOutputBufferSizeNotMultOfXferSize                                     (-200838)
#define DAQmxErrorSampPerChanNotMultOfXferSize                                          (-200837)
#define DAQmxErrorWriteToTEDSFailed                                                     (-200836)
#define DAQmxErrorSCXIDevNotUsablePowerTurnedOff                                        (-200835)
#define DAQmxErrorCannotReadWhenAutoStartFalseBufSizeZeroAndTaskNotRunning              (-200834)
#define DAQmxErrorCannotReadWhenAutoStartFalseHWTimedSinglePtAndTaskNotRunning          (-200833)
#define DAQmxErrorCannotReadWhenAutoStartFalseOnDemandAndTaskNotRunning                 (-200832)
#define DAQmxErrorSimultaneousAOWhenNotOnDemandTiming                                   (-200831)
#define DAQmxErrorMemMapAndSimultaneousAO                                               (-200830)
#define DAQmxErrorWriteFailedMultipleCOOutputTypes                                      (-200829)
#define DAQmxErrorWriteToTEDSNotSupportedOnRT                                           (-200828)
#define DAQmxErrorVirtualTEDSDataFileError                                              (-200827)
#define DAQmxErrorTEDSSensorDataError                                                   (-200826)
#define DAQmxErrorDataSizeMoreThanSizeOfEEPROMOnTEDS                                    (-200825)
#define DAQmxErrorPROMOnTEDSContainsBasicTEDSData                                       (-200824)
#define DAQmxErrorPROMOnTEDSAlreadyWritten                                              (-200823)
#define DAQmxErrorTEDSDoesNotContainPROM                                                (-200822)
#define DAQmxErrorHWTimedSinglePointNotSupportedAI                                      (-200821)
#define DAQmxErrorHWTimedSinglePointOddNumChansInAITask                                 (-200820)
#define DAQmxErrorCantUseOnlyOnBoardMemWithProgrammedIO                                 (-200819)
#define DAQmxErrorSwitchDevShutDownDueToHighTemp                                        (-200818)
#define DAQmxErrorExcitationNotSupportedWhenTermCfgDiff                                 (-200817)
#define DAQmxErrorTEDSMinElecValGEMaxElecVal                                            (-200816)
#define DAQmxErrorTEDSMinPhysValGEMaxPhysVal                                            (-200815)
#define DAQmxErrorCIOnboardClockNotSupportedAsInputTerm                                 (-200814)
#define DAQmxErrorInvalidSampModeForPositionMeas                                        (-200813)
#define DAQmxErrorTrigWhenAOHWTimedSinglePtSampMode                                     (-200812)
#define DAQmxErrorDAQmxCantUseStringDueToUnknownChar                                    (-200811)
#define DAQmxErrorDAQmxCantRetrieveStringDueToUnknownChar                               (-200810)
#define DAQmxErrorClearTEDSNotSupportedOnRT                                             (-200809)
#define DAQmxErrorCfgTEDSNotSupportedOnRT                                               (-200808)
#define DAQmxErrorProgFilterClkCfgdToDifferentMinPulseWidthBySameTask1PerDev            (-200807)
#define DAQmxErrorProgFilterClkCfgdToDifferentMinPulseWidthByAnotherTask1PerDev         (-200806)
#define DAQmxErrorNoLastExtCalDateTimeLastExtCalNotDAQmx                                (-200804)
#define DAQmxErrorCannotWriteNotStartedAutoStartFalseNotOnDemandHWTimedSglPt            (-200803)
#define DAQmxErrorCannotWriteNotStartedAutoStartFalseNotOnDemandBufSizeZero             (-200802)
#define DAQmxErrorCOInvalidTimingSrcDueToSignal                                         (-200801)
#define DAQmxErrorCIInvalidTimingSrcForSampClkDueToSampTimingType                       (-200800)
#define DAQmxErrorCIInvalidTimingSrcForEventCntDueToSampMode                            (-200799)
#define DAQmxErrorNoChangeDetectOnNonInputDigLineForDev                                 (-200798)
#define DAQmxErrorEmptyStringTermNameNotSupported                                       (-200797)
#define DAQmxErrorMemMapEnabledForHWTimedNonBufferedAO                                  (-200796)
#define DAQmxErrorDevOnboardMemOverflowDuringHWTimedNonBufferedGen                      (-200795)
#define DAQmxErrorCODAQmxWriteMultipleChans                                             (-200794)
#define DAQmxErrorCantMaintainExistingValueAOSync                                       (-200793)
#define DAQmxErrorMStudioMultiplePhysChansNotSupported                                  (-200792)
#define DAQmxErrorCantConfigureTEDSForChan                                              (-200791)
#define DAQmxErrorWriteDataTypeTooSmall                                                 (-200790)
#define DAQmxErrorReadDataTypeTooSmall                                                  (-200789)
#define DAQmxErrorMeasuredBridgeOffsetTooHigh                                           (-200788)
#define DAQmxErrorStartTrigConflictWithCOHWTimedSinglePt                                (-200787)
#define DAQmxErrorSampClkRateExtSampClkTimebaseRateMismatch                             (-200786)
#define DAQmxErrorInvalidTimingSrcDueToSampTimingType                                   (-200785)
#define DAQmxErrorVirtualTEDSFileNotFound                                               (-200784)
#define DAQmxErrorMStudioNoForwardPolyScaleCoeffs                                       (-200783)
#define DAQmxErrorMStudioNoReversePolyScaleCoeffs                                       (-200782)
#define DAQmxErrorMStudioNoPolyScaleCoeffsUseCalc                                       (-200781)
#define DAQmxErrorMStudioNoForwardPolyScaleCoeffsUseCalc                                (-200780)
#define DAQmxErrorMStudioNoReversePolyScaleCoeffsUseCalc                                (-200779)
#define DAQmxErrorCOSampModeSampTimingTypeSampClkConflict                               (-200778)
#define DAQmxErrorDevCannotProduceMinPulseWidth                                         (-200777)
#define DAQmxErrorCannotProduceMinPulseWidthGivenPropertyValues                         (-200776)
#define DAQmxErrorTermCfgdToDifferentMinPulseWidthByAnotherTask                         (-200775)
#define DAQmxErrorTermCfgdToDifferentMinPulseWidthByAnotherProperty                     (-200774)
#define DAQmxErrorDigSyncNotAvailableOnTerm                                             (-200773)
#define DAQmxErrorDigFilterNotAvailableOnTerm                                           (-200772)
#define DAQmxErrorDigFilterEnabledMinPulseWidthNotCfg                                   (-200771)
#define DAQmxErrorDigFilterAndSyncBothEnabled                                           (-200770)
#define DAQmxErrorHWTimedSinglePointAOAndDataXferNotProgIO                              (-200769)
#define DAQmxErrorNonBufferedAOAndDataXferNotProgIO                                     (-200768)
#define DAQmxErrorProgIODataXferForBufferedAO                                           (-200767)
#define DAQmxErrorTEDSLegacyTemplateIDInvalidOrUnsupported                              (-200766)
#define DAQmxErrorTEDSMappingMethodInvalidOrUnsupported                                 (-200765)
#define DAQmxErrorTEDSLinearMappingSlopeZero                                            (-200764)
#define DAQmxErrorAIInputBufferSizeNotMultOfXferSize                                    (-200763)
#define DAQmxErrorNoSyncPulseExtSampClkTimebase                                         (-200762)
#define DAQmxErrorNoSyncPulseAnotherTaskRunning                                         (-200761)
#define DAQmxErrorAOMinMaxNotInGainRange                                                (-200760)
#define DAQmxErrorAOMinMaxNotInDACRange                                                 (-200759)
#define DAQmxErrorDevOnlySupportsSampClkTimingAO                                        (-200758)
#define DAQmxErrorDevOnlySupportsSampClkTimingAI                                        (-200757)
#define DAQmxErrorTEDSIncompatibleSensorAndMeasType                                     (-200756)
#define DAQmxErrorTEDSMultipleCalTemplatesNotSupported                                  (-200755)
#define DAQmxErrorTEDSTemplateParametersNotSupported                                    (-200754)
#define DAQmxErrorParsingTEDSData                                                       (-200753)
#define DAQmxErrorMultipleActivePhysChansNotSupported                                   (-200752)
#define DAQmxErrorNoChansSpecdForChangeDetect                                           (-200751)
#define DAQmxErrorInvalidCalVoltageForGivenGain                                         (-200750)
#define DAQmxErrorInvalidCalGain                                                        (-200749)
#define DAQmxErrorMultipleWritesBetweenSampClks                                         (-200748)
#define DAQmxErrorInvalidAcqTypeForFREQOUT                                              (-200747)
#define DAQmxErrorSuitableTimebaseNotFoundTimeCombo2                                    (-200746)
#define DAQmxErrorSuitableTimebaseNotFoundFrequencyCombo2                               (-200745)
#define DAQmxErrorRefClkRateRefClkSrcMismatch                                           (-200744)
#define DAQmxErrorNoTEDSTerminalBlock                                                   (-200743)
#define DAQmxErrorCorruptedTEDSMemory                                                   (-200742)
#define DAQmxErrorTEDSNotSupported                                                      (-200741)
#define DAQmxErrorTimingSrcTaskStartedBeforeTimedLoop                                   (-200740)
#define DAQmxErrorPropertyNotSupportedForTimingSrc                                      (-200739)
#define DAQmxErrorTimingSrcDoesNotExist                                                 (-200738)
#define DAQmxErrorInputBufferSizeNotEqualSampsPerChanForFiniteSampMode                  (-200737)
#define DAQmxErrorFREQOUTCannotProduceDesiredFrequency2                                 (-200736)
#define DAQmxErrorExtRefClkRateNotSpecified                                             (-200735)
#define DAQmxErrorDeviceDoesNotSupportDMADataXferForNonBufferedAcq                      (-200734)
#define DAQmxErrorDigFilterMinPulseWidthSetWhenTristateIsFalse                          (-200733)
#define DAQmxErrorDigFilterEnableSetWhenTristateIsFalse                                 (-200732)
#define DAQmxErrorNoHWTimingWithOnDemand                                                (-200731)
#define DAQmxErrorCannotDetectChangesWhenTristateIsFalse                                (-200730)
#define DAQmxErrorCannotHandshakeWhenTristateIsFalse                                    (-200729)
#define DAQmxErrorLinesUsedForStaticInputNotForHandshakingControl                       (-200728)
#define DAQmxErrorLinesUsedForHandshakingControlNotForStaticInput                       (-200727)
#define DAQmxErrorLinesUsedForStaticInputNotForHandshakingInput                         (-200726)
#define DAQmxErrorLinesUsedForHandshakingInputNotForStaticInput                         (-200725)
#define DAQmxErrorDifferentDITristateValsForChansInTask                                 (-200724)
#define DAQmxErrorTimebaseCalFreqVarianceTooLarge                                       (-200723)
#define DAQmxErrorTimebaseCalFailedToConverge                                           (-200722)
#define DAQmxErrorInadequateResolutionForTimebaseCal                                    (-200721)
#define DAQmxErrorInvalidAOGainCalConst                                                 (-200720)
#define DAQmxErrorInvalidAOOffsetCalConst                                               (-200719)
#define DAQmxErrorInvalidAIGainCalConst                                                 (-200718)
#define DAQmxErrorInvalidAIOffsetCalConst                                               (-200717)
#define DAQmxErrorDigOutputOverrun                                                      (-200716)
#define DAQmxErrorDigInputOverrun                                                       (-200715)
#define DAQmxErrorAcqStoppedDriverCantXferDataFastEnough                                (-200714)
#define DAQmxErrorChansCantAppearInSameTask                                             (-200713)
#define DAQmxErrorInputCfgFailedBecauseWatchdogExpired                                  (-200712)
#define DAQmxErrorAnalogTrigChanNotExternal                                             (-200711)
#define DAQmxErrorTooManyChansForInternalAIInputSrc                                     (-200710)
#define DAQmxErrorTEDSSensorNotDetected                                                 (-200709)
#define DAQmxErrorPrptyGetSpecdActiveItemFailedDueToDifftValues                         (-200708)
#define DAQmxErrorRoutingDestTermPXIClk10InNotInSlot2                                   (-200706)
#define DAQmxErrorRoutingDestTermPXIStarXNotInSlot2                                     (-200705)
#define DAQmxErrorRoutingSrcTermPXIStarXNotInSlot2                                      (-200704)
#define DAQmxErrorRoutingSrcTermPXIStarInSlot16AndAbove                                 (-200703)
#define DAQmxErrorRoutingDestTermPXIStarInSlot16AndAbove                                (-200702)
#define DAQmxErrorRoutingDestTermPXIStarInSlot2                                         (-200701)
#define DAQmxErrorRoutingSrcTermPXIStarInSlot2                                          (-200700)
#define DAQmxErrorRoutingDestTermPXIChassisNotIdentified                                (-200699)
#define DAQmxErrorRoutingSrcTermPXIChassisNotIdentified                                 (-200698)
#define DAQmxErrorFailedToAcquireCalData                                                (-200697)
#define DAQmxErrorBridgeOffsetNullingCalNotSupported                                    (-200696)
#define DAQmxErrorAIMaxNotSpecified                                                     (-200695)
#define DAQmxErrorAIMinNotSpecified                                                     (-200694)
#define DAQmxErrorOddTotalBufferSizeToWrite                                             (-200693)
#define DAQmxErrorOddTotalNumSampsToWrite                                               (-200692)
#define DAQmxErrorBufferWithWaitMode                                                    (-200691)
#define DAQmxErrorBufferWithHWTimedSinglePointSampMode                                  (-200690)
#define DAQmxErrorCOWritePulseLowTicksNotSupported                                      (-200689)
#define DAQmxErrorCOWritePulseHighTicksNotSupported                                     (-200688)
#define DAQmxErrorCOWritePulseLowTimeOutOfRange                                         (-200687)
#define DAQmxErrorCOWritePulseHighTimeOutOfRange                                        (-200686)
#define DAQmxErrorCOWriteFreqOutOfRange                                                 (-200685)
#define DAQmxErrorCOWriteDutyCycleOutOfRange                                            (-200684)
#define DAQmxErrorInvalidInstallation                                                   (-200683)
#define DAQmxErrorRefTrigMasterSessionUnavailable                                       (-200682)
#define DAQmxErrorRouteFailedBecauseWatchdogExpired                                     (-200681)
#define DAQmxErrorDeviceShutDownDueToHighTemp                                           (-200680)
#define DAQmxErrorNoMemMapWhenHWTimedSinglePoint                                        (-200679)
#define DAQmxErrorWriteFailedBecauseWatchdogExpired                                     (-200678)
#define DAQmxErrorDifftInternalAIInputSrcs                                              (-200677)
#define DAQmxErrorDifftAIInputSrcInOneChanGroup                                         (-200676)
#define DAQmxErrorInternalAIInputSrcInMultChanGroups                                    (-200675)
#define DAQmxErrorSwitchOpFailedDueToPrevError                                          (-200674)
#define DAQmxErrorWroteMultiSampsUsingSingleSampWrite                                   (-200673)
#define DAQmxErrorMismatchedInputArraySizes                                             (-200672)
#define DAQmxErrorCantExceedRelayDriveLimit                                             (-200671)
#define DAQmxErrorDACRngLowNotEqualToMinusRefVal                                        (-200670)
#define DAQmxErrorCantAllowConnectDACToGnd                                              (-200669)
#define DAQmxErrorWatchdogTimeoutOutOfRangeAndNotSpecialVal                             (-200668)
#define DAQmxErrorNoWatchdogOutputOnPortReservedForInput                                (-200667)
#define DAQmxErrorNoInputOnPortCfgdForWatchdogOutput                                    (-200666)
#define DAQmxErrorWatchdogExpirationStateNotEqualForLinesInPort                         (-200665)
#define DAQmxErrorCannotPerformOpWhenTaskNotReserved                                    (-200664)
#define DAQmxErrorPowerupStateNotSupported                                              (-200663)
#define DAQmxErrorWatchdogTimerNotSupported                                             (-200662)
#define DAQmxErrorOpNotSupportedWhenRefClkSrcNone                                       (-200661)
#define DAQmxErrorSampClkRateUnavailable                                                (-200660)
#define DAQmxErrorPrptyGetSpecdSingleActiveChanFailedDueToDifftVals                     (-200659)
#define DAQmxErrorPrptyGetImpliedActiveChanFailedDueToDifftVals                         (-200658)
#define DAQmxErrorPrptyGetSpecdActiveChanFailedDueToDifftVals                           (-200657)
#define DAQmxErrorNoRegenWhenUsingBrdMem                                                (-200656)
#define DAQmxErrorNonbufferedReadMoreThanSampsPerChan                                   (-200655)
#define DAQmxErrorWatchdogExpirationTristateNotSpecdForEntirePort                       (-200654)
#define DAQmxErrorPowerupTristateNotSpecdForEntirePort                                  (-200653)
#define DAQmxErrorPowerupStateNotSpecdForEntirePort                                     (-200652)
#define DAQmxErrorCantSetWatchdogExpirationOnDigInChan                                  (-200651)
#define DAQmxErrorCantSetPowerupStateOnDigInChan                                        (-200650)
#define DAQmxErrorPhysChanNotInTask                                                     (-200649)
#define DAQmxErrorPhysChanDevNotInTask                                                  (-200648)
#define DAQmxErrorDigInputNotSupported                                                  (-200647)
#define DAQmxErrorDigFilterIntervalNotEqualForLines                                     (-200646)
#define DAQmxErrorDigFilterIntervalAlreadyCfgd                                          (-200645)
#define DAQmxErrorCantResetExpiredWatchdog                                              (-200644)
#define DAQmxErrorActiveChanTooManyLinesSpecdWhenGettingPrpty                           (-200643)
#define DAQmxErrorActiveChanNotSpecdWhenGetting1LinePrpty                               (-200642)
#define DAQmxErrorDigPrptyCannotBeSetPerLine                                            (-200641)
#define DAQmxErrorSendAdvCmpltAfterWaitForTrigInScanlist                                (-200640)
#define DAQmxErrorDisconnectionRequiredInScanlist                                       (-200639)
#define DAQmxErrorTwoWaitForTrigsAfterConnectionInScanlist                              (-200638)
#define DAQmxErrorActionSeparatorRequiredAfterBreakingConnectionInScanlist              (-200637)
#define DAQmxErrorConnectionInScanlistMustWaitForTrig                                   (-200636)
#define DAQmxErrorActionNotSupportedTaskNotWatchdog                                     (-200635)
#define DAQmxErrorWfmNameSameAsScriptName                                               (-200634)
#define DAQmxErrorScriptNameSameAsWfmName                                               (-200633)
#define DAQmxErrorDSFStopClock                                                          (-200632)
#define DAQmxErrorDSFReadyForStartClock                                                 (-200631)
#define DAQmxErrorWriteOffsetNotMultOfIncr                                              (-200630)
#define DAQmxErrorDifferentPrptyValsNotSupportedOnDev                                   (-200629)
#define DAQmxErrorRefAndPauseTrigConfigured                                             (-200628)
#define DAQmxErrorFailedToEnableHighSpeedInputClock                                     (-200627)
#define DAQmxErrorEmptyPhysChanInPowerUpStatesArray                                     (-200626)
#define DAQmxErrorActivePhysChanTooManyLinesSpecdWhenGettingPrpty                       (-200625)
#define DAQmxErrorActivePhysChanNotSpecdWhenGetting1LinePrpty                           (-200624)
#define DAQmxErrorPXIDevTempCausedShutDown                                              (-200623)
#define DAQmxErrorInvalidNumSampsToWrite                                                (-200622)
#define DAQmxErrorOutputFIFOUnderflow2                                                  (-200621)
#define DAQmxErrorRepeatedAIPhysicalChan                                                (-200620)
#define DAQmxErrorMultScanOpsInOneChassis                                               (-200619)
#define DAQmxErrorInvalidAIChanOrder                                                    (-200618)
#define DAQmxErrorReversePowerProtectionActivated                                       (-200617)
#define DAQmxErrorInvalidAsynOpHandle                                                   (-200616)
#define DAQmxErrorFailedToEnableHighSpeedOutput                                         (-200615)
#define DAQmxErrorCannotReadPastEndOfRecord                                             (-200614)
#define DAQmxErrorAcqStoppedToPreventInputBufferOverwriteOneDataXferMech                (-200613)
#define DAQmxErrorZeroBasedChanIndexInvalid                                             (-200612)
#define DAQmxErrorNoChansOfGivenTypeInTask                                              (-200611)
#define DAQmxErrorSampClkSrcInvalidForOutputValidForInput                               (-200610)
#define DAQmxErrorOutputBufSizeTooSmallToStartGen                                       (-200609)
#define DAQmxErrorInputBufSizeTooSmallToStartAcq                                        (-200608)
#define DAQmxErrorExportTwoSignalsOnSameTerminal                                        (-200607)
#define DAQmxErrorChanIndexInvalid                                                      (-200606)
#define DAQmxErrorRangeSyntaxNumberTooBig                                               (-200605)
#define DAQmxErrorNULLPtr                                                               (-200604)
#define DAQmxErrorScaledMinEqualMax                                                     (-200603)
#define DAQmxErrorPreScaledMinEqualMax                                                  (-200602)
#define DAQmxErrorPropertyNotSupportedForScaleType                                      (-200601)
#define DAQmxErrorChannelNameGenerationNumberTooBig                                     (-200600)
#define DAQmxErrorRepeatedNumberInScaledValues                                          (-200599)
#define DAQmxErrorRepeatedNumberInPreScaledValues                                       (-200598)
#define DAQmxErrorLinesAlreadyReservedForOutput                                         (-200597)
#define DAQmxErrorSwitchOperationChansSpanMultipleDevsInList                            (-200596)
#define DAQmxErrorInvalidIDInListAtBeginningOfSwitchOperation                           (-200595)
#define DAQmxErrorMStudioInvalidPolyDirection                                           (-200594)
#define DAQmxErrorMStudioPropertyGetWhileTaskNotVerified                                (-200593)
#define DAQmxErrorRangeWithTooManyObjects                                               (-200592)
#define DAQmxErrorCppDotNetAPINegativeBufferSize                                        (-200591)
#define DAQmxErrorCppCantRemoveInvalidEventHandler                                      (-200590)
#define DAQmxErrorCppCantRemoveEventHandlerTwice                                        (-200589)
#define DAQmxErrorCppCantRemoveOtherObjectsEventHandler                                 (-200588)
#define DAQmxErrorDigLinesReservedOrUnavailable                                         (-200587)
#define DAQmxErrorDSFFailedToResetStream                                                (-200586)
#define DAQmxErrorDSFReadyForOutputNotAsserted                                          (-200585)
#define DAQmxErrorSampToWritePerChanNotMultipleOfIncr                                   (-200584)
#define DAQmxErrorAOPropertiesCauseVoltageBelowMin                                      (-200583)
#define DAQmxErrorAOPropertiesCauseVoltageOverMax                                       (-200582)
#define DAQmxErrorPropertyNotSupportedWhenRefClkSrcNone                                 (-200581)
#define DAQmxErrorAIMaxTooSmall                                                         (-200580)
#define DAQmxErrorAIMaxTooLarge                                                         (-200579)
#define DAQmxErrorAIMinTooSmall                                                         (-200578)
#define DAQmxErrorAIMinTooLarge                                                         (-200577)
#define DAQmxErrorBuiltInCJCSrcNotSupported                                             (-200576)
#define DAQmxErrorTooManyPostTrigSampsPerChan                                           (-200575)
#define DAQmxErrorTrigLineNotFoundSingleDevRoute                                        (-200574)
#define DAQmxErrorDifferentInternalAIInputSources                                       (-200573)
#define DAQmxErrorDifferentAIInputSrcInOneChanGroup                                     (-200572)
#define DAQmxErrorInternalAIInputSrcInMultipleChanGroups                                (-200571)
#define DAQmxErrorCAPIChanIndexInvalid                                                  (-200570)
#define DAQmxErrorCollectionDoesNotMatchChanType                                        (-200569)
#define DAQmxErrorOutputCantStartChangedRegenerationMode                                (-200568)
#define DAQmxErrorOutputCantStartChangedBufferSize                                      (-200567)
#define DAQmxErrorChanSizeTooBigForU32PortWrite                                         (-200566)
#define DAQmxErrorChanSizeTooBigForU8PortWrite                                          (-200565)
#define DAQmxErrorChanSizeTooBigForU32PortRead                                          (-200564)
#define DAQmxErrorChanSizeTooBigForU8PortRead                                           (-200563)
#define DAQmxErrorInvalidDigDataWrite                                                   (-200562)
#define DAQmxErrorInvalidAODataWrite                                                    (-200561)
#define DAQmxErrorWaitUntilDoneDoesNotIndicateDone                                      (-200560)
#define DAQmxErrorMultiChanTypesInTask                                                  (-200559)
#define DAQmxErrorMultiDevsInTask                                                       (-200558)
#define DAQmxErrorCannotSetPropertyWhenTaskRunning                                      (-200557)
#define DAQmxErrorCannotGetPropertyWhenTaskNotCommittedOrRunning                        (-200556)
#define DAQmxErrorLeadingUnderscoreInString                                             (-200555)
#define DAQmxErrorTrailingSpaceInString                                                 (-200554)
#define DAQmxErrorLeadingSpaceInString                                                  (-200553)
#define DAQmxErrorInvalidCharInString                                                   (-200552)
#define DAQmxErrorDLLBecameUnlocked                                                     (-200551)
#define DAQmxErrorDLLLock                                                               (-200550)
#define DAQmxErrorSelfCalConstsInvalid                                                  (-200549)
#define DAQmxErrorInvalidTrigCouplingExceptForExtTrigChan                               (-200548)
#define DAQmxErrorWriteFailsBufferSizeAutoConfigured                                    (-200547)
#define DAQmxErrorExtCalAdjustExtRefVoltageFailed                                       (-200546)
#define DAQmxErrorSelfCalFailedExtNoiseOrRefVoltageOutOfCal                             (-200545)
#define DAQmxErrorExtCalTemperatureNotDAQmx                                             (-200544)
#define DAQmxErrorExtCalDateTimeNotDAQmx                                                (-200543)
#define DAQmxErrorSelfCalTemperatureNotDAQmx                                            (-200542)
#define DAQmxErrorSelfCalDateTimeNotDAQmx                                               (-200541)
#define DAQmxErrorDACRefValNotSet                                                       (-200540)
#define DAQmxErrorAnalogMultiSampWriteNotSupported                                      (-200539)
#define DAQmxErrorInvalidActionInControlTask                                            (-200538)
#define DAQmxErrorPolyCoeffsInconsistent                                                (-200537)
#define DAQmxErrorSensorValTooLow                                                       (-200536)
#define DAQmxErrorSensorValTooHigh                                                      (-200535)
#define DAQmxErrorWaveformNameTooLong                                                   (-200534)
#define DAQmxErrorIdentifierTooLongInScript                                             (-200533)
#define DAQmxErrorUnexpectedIDFollowingSwitchChanName                                   (-200532)
#define DAQmxErrorRelayNameNotSpecifiedInList                                           (-200531)
#define DAQmxErrorUnexpectedIDFollowingRelayNameInList                                  (-200530)
#define DAQmxErrorUnexpectedIDFollowingSwitchOpInList                                   (-200529)
#define DAQmxErrorInvalidLineGrouping                                                   (-200528)
#define DAQmxErrorCtrMinMax                                                             (-200527)
#define DAQmxErrorWriteChanTypeMismatch                                                 (-200526)
#define DAQmxErrorReadChanTypeMismatch                                                  (-200525)
#define DAQmxErrorWriteNumChansMismatch                                                 (-200524)
#define DAQmxErrorOneChanReadForMultiChanTask                                           (-200523)
#define DAQmxErrorCannotSelfCalDuringExtCal                                             (-200522)
#define DAQmxErrorMeasCalAdjustOscillatorPhaseDAC                                       (-200521)
#define DAQmxErrorInvalidCalConstCalADCAdjustment                                       (-200520)
#define DAQmxErrorInvalidCalConstOscillatorFreqDACValue                                 (-200519)
#define DAQmxErrorInvalidCalConstOscillatorPhaseDACValue                                (-200518)
#define DAQmxErrorInvalidCalConstOffsetDACValue                                         (-200517)
#define DAQmxErrorInvalidCalConstGainDACValue                                           (-200516)
#define DAQmxErrorInvalidNumCalADCReadsToAverage                                        (-200515)
#define DAQmxErrorInvalidCfgCalAdjustDirectPathOutputImpedance                          (-200514)
#define DAQmxErrorInvalidCfgCalAdjustMainPathOutputImpedance                            (-200513)
#define DAQmxErrorInvalidCfgCalAdjustMainPathPostAmpGainAndOffset                       (-200512)
#define DAQmxErrorInvalidCfgCalAdjustMainPathPreAmpGain                                 (-200511)
#define DAQmxErrorInvalidCfgCalAdjustMainPreAmpOffset                                   (-200510)
#define DAQmxErrorMeasCalAdjustCalADC                                                   (-200509)
#define DAQmxErrorMeasCalAdjustOscillatorFrequency                                      (-200508)
#define DAQmxErrorMeasCalAdjustDirectPathOutputImpedance                                (-200507)
#define DAQmxErrorMeasCalAdjustMainPathOutputImpedance                                  (-200506)
#define DAQmxErrorMeasCalAdjustDirectPathGain                                           (-200505)
#define DAQmxErrorMeasCalAdjustMainPathPostAmpGainAndOffset                             (-200504)
#define DAQmxErrorMeasCalAdjustMainPathPreAmpGain                                       (-200503)
#define DAQmxErrorMeasCalAdjustMainPathPreAmpOffset                                     (-200502)
#define DAQmxErrorInvalidDateTimeInEEPROM                                               (-200501)
#define DAQmxErrorUnableToLocateErrorResources                                          (-200500)
#define DAQmxErrorDotNetAPINotUnsigned32BitNumber                                       (-200499)
#define DAQmxErrorInvalidRangeOfObjectsSyntaxInString                                   (-200498)
#define DAQmxErrorAttemptToEnableLineNotPreviouslyDisabled                              (-200497)
#define DAQmxErrorInvalidCharInPattern                                                  (-200496)
#define DAQmxErrorIntermediateBufferFull                                                (-200495)
#define DAQmxErrorLoadTaskFailsBecauseNoTimingOnDev                                     (-200494)
#define DAQmxErrorCAPIReservedParamNotNULLNorEmpty                                      (-200493)
#define DAQmxErrorCAPIReservedParamNotNULL                                              (-200492)
#define DAQmxErrorCAPIReservedParamNotZero                                              (-200491)
#define DAQmxErrorSampleValueOutOfRange                                                 (-200490)
#define DAQmxErrorChanAlreadyInTask                                                     (-200489)
#define DAQmxErrorVirtualChanDoesNotExist                                               (-200488)
#define DAQmxErrorChanNotInTask                                                         (-200486)
#define DAQmxErrorTaskNotInDataNeighborhood                                             (-200485)
#define DAQmxErrorCantSaveTaskWithoutReplace                                            (-200484)
#define DAQmxErrorCantSaveChanWithoutReplace                                            (-200483)
#define DAQmxErrorDevNotInTask                                                          (-200482)
#define DAQmxErrorDevAlreadyInTask                                                      (-200481)
#define DAQmxErrorCanNotPerformOpWhileTaskRunning                                       (-200479)
#define DAQmxErrorCanNotPerformOpWhenNoChansInTask                                      (-200478)
#define DAQmxErrorCanNotPerformOpWhenNoDevInTask                                        (-200477)
#define DAQmxErrorCannotPerformOpWhenTaskNotRunning                                     (-200475)
#define DAQmxErrorOperationTimedOut                                                     (-200474)
#define DAQmxErrorCannotReadWhenAutoStartFalseAndTaskNotRunningOrCommitted              (-200473)
#define DAQmxErrorCannotWriteWhenAutoStartFalseAndTaskNotRunningOrCommitted             (-200472)
#define DAQmxErrorTaskVersionNew                                                        (-200470)
#define DAQmxErrorChanVersionNew                                                        (-200469)
#define DAQmxErrorEmptyString                                                           (-200467)
#define DAQmxErrorChannelSizeTooBigForPortReadType                                      (-200466)
#define DAQmxErrorChannelSizeTooBigForPortWriteType                                     (-200465)
#define DAQmxErrorExpectedNumberOfChannelsVerificationFailed                            (-200464)
#define DAQmxErrorNumLinesMismatchInReadOrWrite                                         (-200463)
#define DAQmxErrorOutputBufferEmpty                                                     (-200462)
#define DAQmxErrorInvalidChanName                                                       (-200461)
#define DAQmxErrorReadNoInputChansInTask                                                (-200460)
#define DAQmxErrorWriteNoOutputChansInTask                                              (-200459)
#define DAQmxErrorPropertyNotSupportedNotInputTask                                      (-200457)
#define DAQmxErrorPropertyNotSupportedNotOutputTask                                     (-200456)
#define DAQmxErrorGetPropertyNotInputBufferedTask                                       (-200455)
#define DAQmxErrorGetPropertyNotOutputBufferedTask                                      (-200454)
#define DAQmxErrorInvalidTimeoutVal                                                     (-200453)
#define DAQmxErrorAttributeNotSupportedInTaskContext                                    (-200452)
#define DAQmxErrorAttributeNotQueryableUnlessTaskIsCommitted                            (-200451)
#define DAQmxErrorAttributeNotSettableWhenTaskIsRunning                                 (-200450)
#define DAQmxErrorDACRngLowNotMinusRefValNorZero                                        (-200449)
#define DAQmxErrorDACRngHighNotEqualRefVal                                              (-200448)
#define DAQmxErrorUnitsNotFromCustomScale                                               (-200447)
#define DAQmxErrorInvalidVoltageReadingDuringExtCal                                     (-200446)
#define DAQmxErrorCalFunctionNotSupported                                               (-200445)
#define DAQmxErrorInvalidPhysicalChanForCal                                             (-200444)
#define DAQmxErrorExtCalNotComplete                                                     (-200443)
#define DAQmxErrorCantSyncToExtStimulusFreqDuringCal                                    (-200442)
#define DAQmxErrorUnableToDetectExtStimulusFreqDuringCal                                (-200441)
#define DAQmxErrorInvalidCloseAction                                                    (-200440)
#define DAQmxErrorExtCalFunctionOutsideExtCalSession                                    (-200439)
#define DAQmxErrorInvalidCalArea                                                        (-200438)
#define DAQmxErrorExtCalConstsInvalid                                                   (-200437)
#define DAQmxErrorStartTrigDelayWithExtSampClk                                          (-200436)
#define DAQmxErrorDelayFromSampClkWithExtConv                                           (-200435)
#define DAQmxErrorFewerThan2PreScaledVals                                               (-200434)
#define DAQmxErrorFewerThan2ScaledValues                                                (-200433)
#define DAQmxErrorPhysChanOutputType                                                    (-200432)
#define DAQmxErrorPhysChanMeasType                                                      (-200431)
#define DAQmxErrorInvalidPhysChanType                                                   (-200430)
#define DAQmxErrorLabVIEWEmptyTaskOrChans                                               (-200429)
#define DAQmxErrorLabVIEWInvalidTaskOrChans                                             (-200428)
#define DAQmxErrorInvalidRefClkRate                                                     (-200427)
#define DAQmxErrorInvalidExtTrigImpedance                                               (-200426)
#define DAQmxErrorHystTrigLevelAIMax                                                    (-200425)
#define DAQmxErrorLineNumIncompatibleWithVideoSignalFormat                              (-200424)
#define DAQmxErrorTrigWindowAIMinAIMaxCombo                                             (-200423)
#define DAQmxErrorTrigAIMinAIMax                                                        (-200422)
#define DAQmxErrorHystTrigLevelAIMin                                                    (-200421)
#define DAQmxErrorInvalidSampRateConsiderRIS                                            (-200420)
#define DAQmxErrorInvalidReadPosDuringRIS                                               (-200419)
#define DAQmxErrorImmedTrigDuringRISMode                                                (-200418)
#define DAQmxErrorTDCNotEnabledDuringRISMode                                            (-200417)
#define DAQmxErrorMultiRecWithRIS                                                       (-200416)
#define DAQmxErrorInvalidRefClkSrc                                                      (-200415)
#define DAQmxErrorInvalidSampClkSrc                                                     (-200414)
#define DAQmxErrorInsufficientOnBoardMemForNumRecsAndSamps                              (-200413)
#define DAQmxErrorInvalidAIAttenuation                                                  (-200412)
#define DAQmxErrorACCouplingNotAllowedWith50OhmImpedance                                (-200411)
#define DAQmxErrorInvalidRecordNum                                                      (-200410)
#define DAQmxErrorZeroSlopeLinearScale                                                  (-200409)
#define DAQmxErrorZeroReversePolyScaleCoeffs                                            (-200408)
#define DAQmxErrorZeroForwardPolyScaleCoeffs                                            (-200407)
#define DAQmxErrorNoReversePolyScaleCoeffs                                              (-200406)
#define DAQmxErrorNoForwardPolyScaleCoeffs                                              (-200405)
#define DAQmxErrorNoPolyScaleCoeffs                                                     (-200404)
#define DAQmxErrorReversePolyOrderLessThanNumPtsToCompute                               (-200403)
#define DAQmxErrorReversePolyOrderNotPositive                                           (-200402)
#define DAQmxErrorNumPtsToComputeNotPositive                                            (-200401)
#define DAQmxErrorWaveformLengthNotMultipleOfIncr                                       (-200400)
#define DAQmxErrorCAPINoExtendedErrorInfoAvailable                                      (-200399)
#define DAQmxErrorCVIFunctionNotFoundInDAQmxDLL                                         (-200398)
#define DAQmxErrorCVIFailedToLoadDAQmxDLL                                               (-200397)
#define DAQmxErrorNoCommonTrigLineForImmedRoute                                         (-200396)
#define DAQmxErrorNoCommonTrigLineForTaskRoute                                          (-200395)
#define DAQmxErrorF64PrptyValNotUnsignedInt                                             (-200394)
#define DAQmxErrorRegisterNotWritable                                                   (-200393)
#define DAQmxErrorInvalidOutputVoltageAtSampClkRate                                     (-200392)
#define DAQmxErrorStrobePhaseShiftDCMBecameUnlocked                                     (-200391)
#define DAQmxErrorDrivePhaseShiftDCMBecameUnlocked                                      (-200390)
#define DAQmxErrorClkOutPhaseShiftDCMBecameUnlocked                                     (-200389)
#define DAQmxErrorOutputBoardClkDCMBecameUnlocked                                       (-200388)
#define DAQmxErrorInputBoardClkDCMBecameUnlocked                                        (-200387)
#define DAQmxErrorInternalClkDCMBecameUnlocked                                          (-200386)
#define DAQmxErrorDCMLock                                                               (-200385)
#define DAQmxErrorDataLineReservedForDynamicOutput                                      (-200384)
#define DAQmxErrorInvalidRefClkSrcGivenSampClkSrc                                       (-200383)
#define DAQmxErrorNoPatternMatcherAvailable                                             (-200382)
#define DAQmxErrorInvalidDelaySampRateBelowPhaseShiftDCMThresh                          (-200381)
#define DAQmxErrorStrainGageCalibration                                                 (-200380)
#define DAQmxErrorInvalidExtClockFreqAndDivCombo                                        (-200379)
#define DAQmxErrorCustomScaleDoesNotExist                                               (-200378)
#define DAQmxErrorOnlyFrontEndChanOpsDuringScan                                         (-200377)
#define DAQmxErrorInvalidOptionForDigitalPortChannel                                    (-200376)
#define DAQmxErrorUnsupportedSignalTypeExportSignal                                     (-200375)
#define DAQmxErrorInvalidSignalTypeExportSignal                                         (-200374)
#define DAQmxErrorUnsupportedTrigTypeSendsSWTrig                                        (-200373)
#define DAQmxErrorInvalidTrigTypeSendsSWTrig                                            (-200372)
#define DAQmxErrorRepeatedPhysicalChan                                                  (-200371)
#define DAQmxErrorResourcesInUseForRouteInTask                                          (-200370)
#define DAQmxErrorResourcesInUseForRoute                                                (-200369)
#define DAQmxErrorRouteNotSupportedByHW                                                 (-200368)
#define DAQmxErrorResourcesInUseForExportSignalPolarity                                 (-200367)
#define DAQmxErrorResourcesInUseForInversionInTask                                      (-200366)
#define DAQmxErrorResourcesInUseForInversion                                            (-200365)
#define DAQmxErrorExportSignalPolarityNotSupportedByHW                                  (-200364)
#define DAQmxErrorInversionNotSupportedByHW                                             (-200363)
#define DAQmxErrorOverloadedChansExistNotRead                                           (-200362)
#define DAQmxErrorInputFIFOOverflow2                                                    (-200361)
#define DAQmxErrorCJCChanNotSpecd                                                       (-200360)
#define DAQmxErrorCtrExportSignalNotPossible                                            (-200359)
#define DAQmxErrorRefTrigWhenContinuous                                                 (-200358)
#define DAQmxErrorIncompatibleSensorOutputAndDeviceInputRanges                          (-200357)
#define DAQmxErrorCustomScaleNameUsed                                                   (-200356)
#define DAQmxErrorPropertyValNotSupportedByHW                                           (-200355)
#define DAQmxErrorPropertyValNotValidTermName                                           (-200354)
#define DAQmxErrorResourcesInUseForProperty                                             (-200353)
#define DAQmxErrorCJCChanAlreadyUsed                                                    (-200352)
#define DAQmxErrorForwardPolynomialCoefNotSpecd                                         (-200351)
#define DAQmxErrorTableScaleNumPreScaledAndScaledValsNotEqual                           (-200350)
#define DAQmxErrorTableScalePreScaledValsNotSpecd                                       (-200349)
#define DAQmxErrorTableScaleScaledValsNotSpecd                                          (-200348)
#define DAQmxErrorIntermediateBufferSizeNotMultipleOfIncr                               (-200347)
#define DAQmxErrorEventPulseWidthOutOfRange                                             (-200346)
#define DAQmxErrorEventDelayOutOfRange                                                  (-200345)
#define DAQmxErrorSampPerChanNotMultipleOfIncr                                          (-200344)
#define DAQmxErrorCannotCalculateNumSampsTaskNotStarted                                 (-200343)
#define DAQmxErrorScriptNotInMem                                                        (-200342)
#define DAQmxErrorOnboardMemTooSmall                                                    (-200341)
#define DAQmxErrorReadAllAvailableDataWithoutBuffer                                     (-200340)
#define DAQmxErrorPulseActiveAtStart                                                    (-200339)
#define DAQmxErrorCalTempNotSupported                                                   (-200338)
#define DAQmxErrorDelayFromSampClkTooLong                                               (-200337)
#define DAQmxErrorDelayFromSampClkTooShort                                              (-200336)
#define DAQmxErrorAIConvRateTooHigh                                                     (-200335)
#define DAQmxErrorDelayFromStartTrigTooLong                                             (-200334)
#define DAQmxErrorDelayFromStartTrigTooShort                                            (-200333)
#define DAQmxErrorSampRateTooHigh                                                       (-200332)
#define DAQmxErrorSampRateTooLow                                                        (-200331)
#define DAQmxErrorPFI0UsedForAnalogAndDigitalSrc                                        (-200330)
#define DAQmxErrorPrimingCfgFIFO                                                        (-200329)
#define DAQmxErrorCannotOpenTopologyCfgFile                                             (-200328)
#define DAQmxErrorInvalidDTInsideWfmDataType                                            (-200327)
#define DAQmxErrorRouteSrcAndDestSame                                                   (-200326)
#define DAQmxErrorReversePolynomialCoefNotSpecd                                         (-200325)
#define DAQmxErrorDevAbsentOrUnavailable                                                (-200324)
#define DAQmxErrorNoAdvTrigForMultiDevScan                                              (-200323)
#define DAQmxErrorInterruptsInsufficientDataXferMech                                    (-200322)
#define DAQmxErrorInvalidAttentuationBasedOnMinMax                                      (-200321)
#define DAQmxErrorCabledModuleCannotRouteSSH                                            (-200320)
#define DAQmxErrorCabledModuleCannotRouteConvClk                                        (-200319)
#define DAQmxErrorInvalidExcitValForScaling                                             (-200318)
#define DAQmxErrorNoDevMemForScript                                                     (-200317)
#define DAQmxErrorScriptDataUnderflow                                                   (-200316)
#define DAQmxErrorNoDevMemForWaveform                                                   (-200315)
#define DAQmxErrorStreamDCMBecameUnlocked                                               (-200314)
#define DAQmxErrorStreamDCMLock                                                         (-200313)
#define DAQmxErrorWaveformNotInMem                                                      (-200312)
#define DAQmxErrorWaveformWriteOutOfBounds                                              (-200311)
#define DAQmxErrorWaveformPreviouslyAllocated                                           (-200310)
#define DAQmxErrorSampClkTbMasterTbDivNotAppropriateForSampTbSrc                        (-200309)
#define DAQmxErrorSampTbRateSampTbSrcMismatch                                           (-200308)
#define DAQmxErrorMasterTbRateMasterTbSrcMismatch                                       (-200307)
#define DAQmxErrorSampsPerChanTooBig                                                    (-200306)
#define DAQmxErrorFinitePulseTrainNotPossible                                           (-200305)
#define DAQmxErrorExtMasterTimebaseRateNotSpecified                                     (-200304)
#define DAQmxErrorExtSampClkSrcNotSpecified                                             (-200303)
#define DAQmxErrorInputSignalSlowerThanMeasTime                                         (-200302)
#define DAQmxErrorCannotUpdatePulseGenProperty                                          (-200301)
#define DAQmxErrorInvalidTimingType                                                     (-200300)
#define DAQmxErrorPropertyUnavailWhenUsingOnboardMemory                                 (-200297)
#define DAQmxErrorCannotWriteAfterStartWithOnboardMemory                                (-200295)
#define DAQmxErrorNotEnoughSampsWrittenForInitialXferRqstCondition                      (-200294)
#define DAQmxErrorNoMoreSpace                                                           (-200293)
#define DAQmxErrorSamplesCanNotYetBeWritten                                             (-200292)
#define DAQmxErrorGenStoppedToPreventIntermediateBufferRegenOfOldSamples                (-200291)
#define DAQmxErrorGenStoppedToPreventRegenOfOldSamples                                  (-200290)
#define DAQmxErrorSamplesNoLongerWriteable                                              (-200289)
#define DAQmxErrorSamplesWillNeverBeGenerated                                           (-200288)
#define DAQmxErrorNegativeWriteSampleNumber                                             (-200287)
#define DAQmxErrorNoAcqStarted                                                          (-200286)
#define DAQmxErrorSamplesNotYetAvailable                                                (-200284)
#define DAQmxErrorAcqStoppedToPreventIntermediateBufferOverflow                         (-200283)
#define DAQmxErrorNoRefTrigConfigured                                                   (-200282)
#define DAQmxErrorCannotReadRelativeToRefTrigUntilDone                                  (-200281)
#define DAQmxErrorSamplesNoLongerAvailable                                              (-200279)
#define DAQmxErrorSamplesWillNeverBeAvailable                                           (-200278)
#define DAQmxErrorNegativeReadSampleNumber                                              (-200277)
#define DAQmxErrorExternalSampClkAndRefClkThruSameTerm                                  (-200276)
#define DAQmxErrorExtSampClkRateTooLowForClkIn                                          (-200275)
#define DAQmxErrorExtSampClkRateTooHighForBackplane                                     (-200274)
#define DAQmxErrorSampClkRateAndDivCombo                                                (-200273)
#define DAQmxErrorSampClkRateTooLowForDivDown                                           (-200272)
#define DAQmxErrorProductOfAOMinAndGainTooSmall                                         (-200271)
#define DAQmxErrorInterpolationRateNotPossible                                          (-200270)
#define DAQmxErrorOffsetTooLarge                                                        (-200269)
#define DAQmxErrorOffsetTooSmall                                                        (-200268)
#define DAQmxErrorProductOfAOMaxAndGainTooLarge                                         (-200267)
#define DAQmxErrorMinAndMaxNotSymmetric                                                 (-200266)
#define DAQmxErrorInvalidAnalogTrigSrc                                                  (-200265)
#define DAQmxErrorTooManyChansForAnalogRefTrig                                          (-200264)
#define DAQmxErrorTooManyChansForAnalogPauseTrig                                        (-200263)
#define DAQmxErrorTrigWhenOnDemandSampTiming                                            (-200262)
#define DAQmxErrorInconsistentAnalogTrigSettings                                        (-200261)
#define DAQmxErrorMemMapDataXferModeSampTimingCombo                                     (-200260)
#define DAQmxErrorInvalidJumperedAttr                                                   (-200259)
#define DAQmxErrorInvalidGainBasedOnMinMax                                              (-200258)
#define DAQmxErrorInconsistentExcit                                                     (-200257)
#define DAQmxErrorTopologyNotSupportedByCfgTermBlock                                    (-200256)
#define DAQmxErrorBuiltInTempSensorNotSupported                                         (-200255)
#define DAQmxErrorInvalidTerm                                                           (-200254)
#define DAQmxErrorCannotTristateTerm                                                    (-200253)
#define DAQmxErrorCannotTristateBusyTerm                                                (-200252)
#define DAQmxErrorNoDMAChansAvailable                                                   (-200251)
#define DAQmxErrorInvalidWaveformLengthWithinLoopInScript                               (-200250)
#define DAQmxErrorInvalidSubsetLengthWithinLoopInScript                                 (-200249)
#define DAQmxErrorMarkerPosInvalidForLoopInScript                                       (-200248)
#define DAQmxErrorIntegerExpectedInScript                                               (-200247)
#define DAQmxErrorPLLBecameUnlocked                                                     (-200246)
#define DAQmxErrorPLLLock                                                               (-200245)
#define DAQmxErrorDDCClkOutDCMBecameUnlocked                                            (-200244)
#define DAQmxErrorDDCClkOutDCMLock                                                      (-200243)
#define DAQmxErrorClkDoublerDCMBecameUnlocked                                           (-200242)
#define DAQmxErrorClkDoublerDCMLock                                                     (-200241)
#define DAQmxErrorSampClkDCMBecameUnlocked                                              (-200240)
#define DAQmxErrorSampClkDCMLock                                                        (-200239)
#define DAQmxErrorSampClkTimebaseDCMBecameUnlocked                                      (-200238)
#define DAQmxErrorSampClkTimebaseDCMLock                                                (-200237)
#define DAQmxErrorAttrCannotBeReset                                                     (-200236)
#define DAQmxErrorExplanationNotFound                                                   (-200235)
#define DAQmxErrorWriteBufferTooSmall                                                   (-200234)
#define DAQmxErrorSpecifiedAttrNotValid                                                 (-200233)
#define DAQmxErrorAttrCannotBeRead                                                      (-200232)
#define DAQmxErrorAttrCannotBeSet                                                       (-200231)
#define DAQmxErrorNULLPtrForC_Api                                                       (-200230)
#define DAQmxErrorReadBufferTooSmall                                                    (-200229)
#define DAQmxErrorBufferTooSmallForString                                               (-200228)
#define DAQmxErrorNoAvailTrigLinesOnDevice                                              (-200227)
#define DAQmxErrorTrigBusLineNotAvail                                                   (-200226)
#define DAQmxErrorCouldNotReserveRequestedTrigLine                                      (-200225)
#define DAQmxErrorTrigLineNotFound                                                      (-200224)
#define DAQmxErrorSCXI1126ThreshHystCombination                                         (-200223)
#define DAQmxErrorAcqStoppedToPreventInputBufferOverwrite                               (-200222)
#define DAQmxErrorTimeoutExceeded                                                       (-200221)
#define DAQmxErrorInvalidDeviceID                                                       (-200220)
#define DAQmxErrorInvalidAOChanOrder                                                    (-200219)
#define DAQmxErrorSampleTimingTypeAndDataXferMode                                       (-200218)
#define DAQmxErrorBufferWithOnDemandSampTiming                                          (-200217)
#define DAQmxErrorBufferAndDataXferMode                                                 (-200216)
#define DAQmxErrorMemMapAndBuffer                                                       (-200215)
#define DAQmxErrorNoAnalogTrigHW                                                        (-200214)
#define DAQmxErrorTooManyPretrigPlusMinPostTrigSamps                                    (-200213)
#define DAQmxErrorInconsistentUnitsSpecified                                            (-200212)
#define DAQmxErrorMultipleRelaysForSingleRelayOp                                        (-200211)
#define DAQmxErrorMultipleDevIDsPerChassisSpecifiedInList                               (-200210)
#define DAQmxErrorDuplicateDevIDInList                                                  (-200209)
#define DAQmxErrorInvalidRangeStatementCharInList                                       (-200208)
#define DAQmxErrorInvalidDeviceIDInList                                                 (-200207)
#define DAQmxErrorTriggerPolarityConflict                                               (-200206)
#define DAQmxErrorCannotScanWithCurrentTopology                                         (-200205)
#define DAQmxErrorUnexpectedIdentifierInFullySpecifiedPathInList                        (-200204)
#define DAQmxErrorSwitchCannotDriveMultipleTrigLines                                    (-200203)
#define DAQmxErrorInvalidRelayName                                                      (-200202)
#define DAQmxErrorSwitchScanlistTooBig                                                  (-200201)
#define DAQmxErrorSwitchChanInUse                                                       (-200200)
#define DAQmxErrorSwitchNotResetBeforeScan                                              (-200199)
#define DAQmxErrorInvalidTopology                                                       (-200198)
#define DAQmxErrorAttrNotSupported                                                      (-200197)
#define DAQmxErrorUnexpectedEndOfActionsInList                                          (-200196)
#define DAQmxErrorPowerBudgetExceeded                                                   (-200195)
#define DAQmxErrorHWUnexpectedlyPoweredOffAndOn                                         (-200194)
#define DAQmxErrorSwitchOperationNotSupported                                           (-200193)
#define DAQmxErrorOnlyContinuousScanSupported                                           (-200192)
#define DAQmxErrorSwitchDifferentTopologyWhenScanning                                   (-200191)
#define DAQmxErrorDisconnectPathNotSameAsExistingPath                                   (-200190)
#define DAQmxErrorConnectionNotPermittedOnChanReservedForRouting                        (-200189)
#define DAQmxErrorCannotConnectSrcChans                                                 (-200188)
#define DAQmxErrorCannotConnectChannelToItself                                          (-200187)
#define DAQmxErrorChannelNotReservedForRouting                                          (-200186)
#define DAQmxErrorCannotConnectChansDirectly                                            (-200185)
#define DAQmxErrorChansAlreadyConnected                                                 (-200184)
#define DAQmxErrorChanDuplicatedInPath                                                  (-200183)
#define DAQmxErrorNoPathToDisconnect                                                    (-200182)
#define DAQmxErrorInvalidSwitchChan                                                     (-200181)
#define DAQmxErrorNoPathAvailableBetween2SwitchChans                                    (-200180)
#define DAQmxErrorExplicitConnectionExists                                              (-200179)
#define DAQmxErrorSwitchDifferentSettlingTimeWhenScanning                               (-200178)
#define DAQmxErrorOperationOnlyPermittedWhileScanning                                   (-200177)
#define DAQmxErrorOperationNotPermittedWhileScanning                                    (-200176)
#define DAQmxErrorHardwareNotResponding                                                 (-200175)
#define DAQmxErrorInvalidSampAndMasterTimebaseRateCombo                                 (-200173)
#define DAQmxErrorNonZeroBufferSizeInProgIOXfer                                         (-200172)
#define DAQmxErrorVirtualChanNameUsed                                                   (-200171)
#define DAQmxErrorPhysicalChanDoesNotExist                                              (-200170)
#define DAQmxErrorMemMapOnlyForProgIOXfer                                               (-200169)
#define DAQmxErrorTooManyChans                                                          (-200168)
#define DAQmxErrorCannotHaveCJTempWithOtherChans                                        (-200167)
#define DAQmxErrorOutputBufferUnderwrite                                                (-200166)
#define DAQmxErrorSensorInvalidCompletionResistance                                     (-200163)
#define DAQmxErrorVoltageExcitIncompatibleWith2WireCfg                                  (-200162)
#define DAQmxErrorIntExcitSrcNotAvailable                                               (-200161)
#define DAQmxErrorCannotCreateChannelAfterTaskVerified                                  (-200160)
#define DAQmxErrorLinesReservedForSCXIControl                                           (-200159)
#define DAQmxErrorCouldNotReserveLinesForSCXIControl                                    (-200158)
#define DAQmxErrorCalibrationFailed                                                     (-200157)
#define DAQmxErrorReferenceFrequencyInvalid                                             (-200156)
#define DAQmxErrorReferenceResistanceInvalid                                            (-200155)
#define DAQmxErrorReferenceCurrentInvalid                                               (-200154)
#define DAQmxErrorReferenceVoltageInvalid                                               (-200153)
#define DAQmxErrorEEPROMDataInvalid                                                     (-200152)
#define DAQmxErrorCabledModuleNotCapableOfRoutingAI                                     (-200151)
#define DAQmxErrorChannelNotAvailableInParallelMode                                     (-200150)
#define DAQmxErrorExternalTimebaseRateNotKnownForDelay                                  (-200149)
#define DAQmxErrorFREQOUTCannotProduceDesiredFrequency                                  (-200148)
#define DAQmxErrorMultipleCounterInputTask                                              (-200147)
#define DAQmxErrorCounterStartPauseTriggerConflict                                      (-200146)
#define DAQmxErrorCounterInputPauseTriggerAndSampleClockInvalid                         (-200145)
#define DAQmxErrorCounterOutputPauseTriggerInvalid                                      (-200144)
#define DAQmxErrorCounterTimebaseRateNotSpecified                                       (-200143)
#define DAQmxErrorCounterTimebaseRateNotFound                                           (-200142)
#define DAQmxErrorCounterOverflow                                                       (-200141)
#define DAQmxErrorCounterNoTimebaseEdgesBetweenGates                                    (-200140)
#define DAQmxErrorCounterMaxMinRangeFreq                                                (-200139)
#define DAQmxErrorCounterMaxMinRangeTime                                                (-200138)
#define DAQmxErrorSuitableTimebaseNotFoundTimeCombo                                     (-200137)
#define DAQmxErrorSuitableTimebaseNotFoundFrequencyCombo                                (-200136)
#define DAQmxErrorInternalTimebaseSourceDivisorCombo                                    (-200135)
#define DAQmxErrorInternalTimebaseSourceRateCombo                                       (-200134)
#define DAQmxErrorInternalTimebaseRateDivisorSourceCombo                                (-200133)
#define DAQmxErrorExternalTimebaseRateNotknownForRate                                   (-200132)
#define DAQmxErrorAnalogTrigChanNotFirstInScanList                                      (-200131)
#define DAQmxErrorNoDivisorForExternalSignal                                            (-200130)
#define DAQmxErrorAttributeInconsistentAcrossRepeatedPhysicalChannels                   (-200128)
#define DAQmxErrorCannotHandshakeWithPort0                                              (-200127)
#define DAQmxErrorControlLineConflictOnPortC                                            (-200126)
#define DAQmxErrorLines4To7ConfiguredForOutput                                          (-200125)
#define DAQmxErrorLines4To7ConfiguredForInput                                           (-200124)
#define DAQmxErrorLines0To3ConfiguredForOutput                                          (-200123)
#define DAQmxErrorLines0To3ConfiguredForInput                                           (-200122)
#define DAQmxErrorPortConfiguredForOutput                                               (-200121)
#define DAQmxErrorPortConfiguredForInput                                                (-200120)
#define DAQmxErrorPortConfiguredForStaticDigitalOps                                     (-200119)
#define DAQmxErrorPortReservedForHandshaking                                            (-200118)
#define DAQmxErrorPortDoesNotSupportHandshakingDataIO                                   (-200117)
#define DAQmxErrorCannotTristate8255OutputLines                                         (-200116)
#define DAQmxErrorTemperatureOutOfRangeForCalibration                                   (-200113)
#define DAQmxErrorCalibrationHandleInvalid                                              (-200112)
#define DAQmxErrorPasswordRequired                                                      (-200111)
#define DAQmxErrorIncorrectPassword                                                     (-200110)
#define DAQmxErrorPasswordTooLong                                                       (-200109)
#define DAQmxErrorCalibrationSessionAlreadyOpen                                         (-200108)
#define DAQmxErrorSCXIModuleIncorrect                                                   (-200107)
#define DAQmxErrorAttributeInconsistentAcrossChannelsOnDevice                           (-200106)
#define DAQmxErrorSCXI1122ResistanceChanNotSupportedForCfg                              (-200105)
#define DAQmxErrorBracketPairingMismatchInList                                          (-200104)
#define DAQmxErrorInconsistentNumSamplesToWrite                                         (-200103)
#define DAQmxErrorIncorrectDigitalPattern                                               (-200102)
#define DAQmxErrorIncorrectNumChannelsToWrite                                           (-200101)
#define DAQmxErrorIncorrectReadFunction                                                 (-200100)
#define DAQmxErrorPhysicalChannelNotSpecified                                           (-200099)
#define DAQmxErrorMoreThanOneTerminal                                                   (-200098)
#define DAQmxErrorMoreThanOneActiveChannelSpecified                                     (-200097)
#define DAQmxErrorInvalidNumberSamplesToRead                                            (-200096)
#define DAQmxErrorAnalogWaveformExpected                                                (-200095)
#define DAQmxErrorDigitalWaveformExpected                                               (-200094)
#define DAQmxErrorActiveChannelNotSpecified                                             (-200093)
#define DAQmxErrorFunctionNotSupportedForDeviceTasks                                    (-200092)
#define DAQmxErrorFunctionNotInLibrary                                                  (-200091)
#define DAQmxErrorLibraryNotPresent                                                     (-200090)
#define DAQmxErrorDuplicateTask                                                         (-200089)
#define DAQmxErrorInvalidTask                                                           (-200088)
#define DAQmxErrorInvalidChannel                                                        (-200087)
#define DAQmxErrorInvalidSyntaxForPhysicalChannelRange                                  (-200086)
#define DAQmxErrorMinNotLessThanMax                                                     (-200082)
#define DAQmxErrorSampleRateNumChansConvertPeriodCombo                                  (-200081)
#define DAQmxErrorAODuringCounter1DMAConflict                                           (-200079)
#define DAQmxErrorAIDuringCounter0DMAConflict                                           (-200078)
#define DAQmxErrorInvalidAttributeValue                                                 (-200077)
#define DAQmxErrorSuppliedCurrentDataOutsideSpecifiedRange                              (-200076)
#define DAQmxErrorSuppliedVoltageDataOutsideSpecifiedRange                              (-200075)
#define DAQmxErrorCannotStoreCalConst                                                   (-200074)
#define DAQmxErrorSCXIModuleNotFound                                                    (-200073)
#define DAQmxErrorDuplicatePhysicalChansNotSupported                                    (-200072)
#define DAQmxErrorTooManyPhysicalChansInList                                            (-200071)
#define DAQmxErrorInvalidAdvanceEventTriggerType                                        (-200070)
#define DAQmxErrorDeviceIsNotAValidSwitch                                               (-200069)
#define DAQmxErrorDeviceDoesNotSupportScanning                                          (-200068)
#define DAQmxErrorScanListCannotBeTimed                                                 (-200067)
#define DAQmxErrorConnectOperatorInvalidAtPointInList                                   (-200066)
#define DAQmxErrorUnexpectedSwitchActionInList                                          (-200065)
#define DAQmxErrorUnexpectedSeparatorInList                                             (-200064)
#define DAQmxErrorExpectedTerminatorInList                                              (-200063)
#define DAQmxErrorExpectedConnectOperatorInList                                         (-200062)
#define DAQmxErrorExpectedSeparatorInList                                               (-200061)
#define DAQmxErrorFullySpecifiedPathInListContainsRange                                 (-200060)
#define DAQmxErrorConnectionSeparatorAtEndOfList                                        (-200059)
#define DAQmxErrorIdentifierInListTooLong                                               (-200058)
#define DAQmxErrorDuplicateDeviceIDInListWhenSettling                                   (-200057)
#define DAQmxErrorChannelNameNotSpecifiedInList                                         (-200056)
#define DAQmxErrorDeviceIDNotSpecifiedInList                                            (-200055)
#define DAQmxErrorSemicolonDoesNotFollowRangeInList                                     (-200054)
#define DAQmxErrorSwitchActionInListSpansMultipleDevices                                (-200053)
#define DAQmxErrorRangeWithoutAConnectActionInList                                      (-200052)
#define DAQmxErrorInvalidIdentifierFollowingSeparatorInList                             (-200051)
#define DAQmxErrorInvalidChannelNameInList                                              (-200050)
#define DAQmxErrorInvalidNumberInRepeatStatementInList                                  (-200049)
#define DAQmxErrorInvalidTriggerLineInList                                              (-200048)
#define DAQmxErrorInvalidIdentifierInListFollowingDeviceID                              (-200047)
#define DAQmxErrorInvalidIdentifierInListAtEndOfSwitchAction                            (-200046)
#define DAQmxErrorDeviceRemoved                                                         (-200045)
#define DAQmxErrorRoutingPathNotAvailable                                               (-200044)
#define DAQmxErrorRoutingHardwareBusy                                                   (-200043)
#define DAQmxErrorRequestedSignalInversionForRoutingNotPossible                         (-200042)
#define DAQmxErrorInvalidRoutingDestinationTerminalName                                 (-200041)
#define DAQmxErrorInvalidRoutingSourceTerminalName                                      (-200040)
#define DAQmxErrorRoutingNotSupportedForDevice                                          (-200039)
#define DAQmxErrorWaitIsLastInstructionOfLoopInScript                                   (-200038)
#define DAQmxErrorClearIsLastInstructionOfLoopInScript                                  (-200037)
#define DAQmxErrorInvalidLoopIterationsInScript                                         (-200036)
#define DAQmxErrorRepeatLoopNestingTooDeepInScript                                      (-200035)
#define DAQmxErrorMarkerPositionOutsideSubsetInScript                                   (-200034)
#define DAQmxErrorSubsetStartOffsetNotAlignedInScript                                   (-200033)
#define DAQmxErrorInvalidSubsetLengthInScript                                           (-200032)
#define DAQmxErrorMarkerPositionNotAlignedInScript                                      (-200031)
#define DAQmxErrorSubsetOutsideWaveformInScript                                         (-200030)
#define DAQmxErrorMarkerOutsideWaveformInScript                                         (-200029)
#define DAQmxErrorWaveformInScriptNotInMem                                              (-200028)
#define DAQmxErrorKeywordExpectedInScript                                               (-200027)
#define DAQmxErrorBufferNameExpectedInScript                                            (-200026)
#define DAQmxErrorProcedureNameExpectedInScript                                         (-200025)
#define DAQmxErrorScriptHasInvalidIdentifier                                            (-200024)
#define DAQmxErrorScriptHasInvalidCharacter                                             (-200023)
#define DAQmxErrorResourceAlreadyReserved                                               (-200022)
#define DAQmxErrorSelfTestFailed                                                        (-200020)
#define DAQmxErrorADCOverrun                                                            (-200019)
#define DAQmxErrorDACUnderflow                                                          (-200018)
#define DAQmxErrorInputFIFOUnderflow                                                    (-200017)
#define DAQmxErrorOutputFIFOUnderflow                                                   (-200016)
#define DAQmxErrorSCXISerialCommunication                                               (-200015)
#define DAQmxErrorDigitalTerminalSpecifiedMoreThanOnce                                  (-200014)
#define DAQmxErrorDigitalOutputNotSupported                                             (-200012)
#define DAQmxErrorInconsistentChannelDirections                                         (-200011)
#define DAQmxErrorInputFIFOOverflow                                                     (-200010)
#define DAQmxErrorTimeStampOverwritten                                                  (-200009)
#define DAQmxErrorStopTriggerHasNotOccurred                                             (-200008)
#define DAQmxErrorRecordNotAvailable                                                    (-200007)
#define DAQmxErrorRecordOverwritten                                                     (-200006)
#define DAQmxErrorDataNotAvailable                                                      (-200005)
#define DAQmxErrorDataOverwrittenInDeviceMemory                                         (-200004)
#define DAQmxErrorDuplicatedChannel                                                     (-200003)
#define DAQmxWarningTimestampCounterRolledOver                                           (200003)
#define DAQmxWarningInputTerminationOverloaded                                           (200004)
#define DAQmxWarningADCOverloaded                                                        (200005)
#define DAQmxWarningPLLUnlocked                                                          (200007)
#define DAQmxWarningCounter0DMADuringAIConflict                                          (200008)
#define DAQmxWarningCounter1DMADuringAOConflict                                          (200009)
#define DAQmxWarningStoppedBeforeDone                                                    (200010)
#define DAQmxWarningRateViolatesSettlingTime                                             (200011)
#define DAQmxWarningRateViolatesMaxADCRate                                               (200012)
#define DAQmxWarningUserDefInfoStringTooLong                                             (200013)
#define DAQmxWarningTooManyInterruptsPerSecond                                           (200014)
#define DAQmxWarningPotentialGlitchDuringWrite                                           (200015)
#define DAQmxWarningDevNotSelfCalibratedWithDAQmx                                        (200016)
#define DAQmxWarningAISampRateTooLow                                                     (200017)
#define DAQmxWarningAIConvRateTooLow                                                     (200018)
#define DAQmxWarningReadOffsetCoercion                                                   (200019)
#define DAQmxWarningPretrigCoercion                                                      (200020)
#define DAQmxWarningSampValCoercedToMax                                                  (200021)
#define DAQmxWarningSampValCoercedToMin                                                  (200022)
#define DAQmxWarningPropertyVersionNew                                                   (200024)
#define DAQmxWarningUserDefinedInfoTooLong                                               (200025)
#define DAQmxWarningCAPIStringTruncatedToFitBuffer                                       (200026)
#define DAQmxWarningSampClkRateTooLow                                                    (200027)
#define DAQmxWarningPossiblyInvalidCTRSampsInFiniteDMAAcq                                (200028)
#define DAQmxWarningRISAcqCompletedSomeBinsNotFilled                                     (200029)
#define DAQmxWarningPXIDevTempExceedsMaxOpTemp                                           (200030)
#define DAQmxWarningOutputGainTooLowForRFFreq                                            (200031)
#define DAQmxWarningOutputGainTooHighForRFFreq                                           (200032)
#define DAQmxWarningMultipleWritesBetweenSampClks                                        (200033)
#define DAQmxWarningDeviceMayShutDownDueToHighTemp                                       (200034)
#define DAQmxWarningRateViolatesMinADCRate                                               (200035)
#define DAQmxWarningSampClkRateAboveDevSpecs                                             (200036)
#define DAQmxWarningCOPrevDAQmxWriteSettingsOverwrittenForHWTimedSinglePoint             (200037)
#define DAQmxWarningLowpassFilterSettlingTimeExceedsUserTimeBetween2ADCConversions       (200038)
#define DAQmxWarningLowpassFilterSettlingTimeExceedsDriverTimeBetween2ADCConversions     (200039)
#define DAQmxWarningSampClkRateViolatesSettlingTimeForGen                                (200040)
#define DAQmxWarningInvalidCalConstValueForAI                                            (200041)
#define DAQmxWarningInvalidCalConstValueForAO                                            (200042)
#define DAQmxWarningChanCalExpired                                                       (200043)
#define DAQmxWarningUnrecognizedEnumValueEncounteredInStorage                            (200044)
#define DAQmxWarningTableCRCNotCorrect                                                   (200045)
#define DAQmxWarningExternalCRCNotCorrect                                                (200046)
#define DAQmxWarningSelfCalCRCNotCorrect                                                 (200047)
#define DAQmxWarningDeviceSpecExceeded                                                   (200048)
#define DAQmxWarningOnlyGainCalibrated                                                   (200049)
#define DAQmxWarningReversePowerProtectionActivated                                      (200050)
#define DAQmxWarningOverVoltageProtectionActivated                                       (200051)
#define DAQmxWarningReadNotCompleteBeforeSampClk                                         (209800)
#define DAQmxWarningWriteNotCompleteBeforeSampClk                                        (209801)
#define DAQmxWarningWaitForNextSampClkDetectedMissedSampClk                              (209802)
#define DAQmxErrorRoutingDestTermPXIDStarXNotInSystemTimingSlot_Routing                  (-89167)
#define DAQmxErrorRoutingSrcTermPXIDStarXNotInSystemTimingSlot_Routing                   (-89166)
#define DAQmxErrorRoutingSrcTermPXIDStarInNonDStarTriggerSlot_Routing                    (-89165)
#define DAQmxErrorRoutingDestTermPXIDStarInNonDStarTriggerSlot_Routing                   (-89164)
#define DAQmxErrorRoutingDestTermPXIClk10InNotInStarTriggerSlot_Routing                  (-89162)
#define DAQmxErrorRoutingDestTermPXIClk10InNotInSystemTimingSlot_Routing                 (-89161)
#define DAQmxErrorRoutingDestTermPXIStarXNotInStarTriggerSlot_Routing                    (-89160)
#define DAQmxErrorRoutingDestTermPXIStarXNotInSystemTimingSlot_Routing                   (-89159)
#define DAQmxErrorRoutingSrcTermPXIStarXNotInStarTriggerSlot_Routing                     (-89158)
#define DAQmxErrorRoutingSrcTermPXIStarXNotInSystemTimingSlot_Routing                    (-89157)
#define DAQmxErrorRoutingSrcTermPXIStarInNonStarTriggerSlot_Routing                      (-89156)
#define DAQmxErrorRoutingDestTermPXIStarInNonStarTriggerSlot_Routing                     (-89155)
#define DAQmxErrorRoutingDestTermPXIStarInStarTriggerSlot_Routing                        (-89154)
#define DAQmxErrorRoutingDestTermPXIStarInSystemTimingSlot_Routing                       (-89153)
#define DAQmxErrorRoutingSrcTermPXIStarInStarTriggerSlot_Routing                         (-89152)
#define DAQmxErrorRoutingSrcTermPXIStarInSystemTimingSlot_Routing                        (-89151)
#define DAQmxErrorInvalidSignalModifier_Routing                                          (-89150)
#define DAQmxErrorRoutingDestTermPXIClk10InNotInSlot2_Routing                            (-89149)
#define DAQmxErrorRoutingDestTermPXIStarXNotInSlot2_Routing                              (-89148)
#define DAQmxErrorRoutingSrcTermPXIStarXNotInSlot2_Routing                               (-89147)
#define DAQmxErrorRoutingSrcTermPXIStarInSlot16AndAbove_Routing                          (-89146)
#define DAQmxErrorRoutingDestTermPXIStarInSlot16AndAbove_Routing                         (-89145)
#define DAQmxErrorRoutingDestTermPXIStarInSlot2_Routing                                  (-89144)
#define DAQmxErrorRoutingSrcTermPXIStarInSlot2_Routing                                   (-89143)
#define DAQmxErrorRoutingDestTermPXIChassisNotIdentified_Routing                         (-89142)
#define DAQmxErrorRoutingSrcTermPXIChassisNotIdentified_Routing                          (-89141)
#define DAQmxErrorTrigLineNotFoundSingleDevRoute_Routing                                 (-89140)
#define DAQmxErrorNoCommonTrigLineForRoute_Routing                                       (-89139)
#define DAQmxErrorResourcesInUseForRouteInTask_Routing                                   (-89138)
#define DAQmxErrorResourcesInUseForRoute_Routing                                         (-89137)
#define DAQmxErrorRouteNotSupportedByHW_Routing                                          (-89136)
#define DAQmxErrorResourcesInUseForInversionInTask_Routing                               (-89135)
#define DAQmxErrorResourcesInUseForInversion_Routing                                     (-89134)
#define DAQmxErrorInversionNotSupportedByHW_Routing                                      (-89133)
#define DAQmxErrorResourcesInUseForProperty_Routing                                      (-89132)
#define DAQmxErrorRouteSrcAndDestSame_Routing                                            (-89131)
#define DAQmxErrorDevAbsentOrUnavailable_Routing                                         (-89130)
#define DAQmxErrorInvalidTerm_Routing                                                    (-89129)
#define DAQmxErrorCannotTristateTerm_Routing                                             (-89128)
#define DAQmxErrorCannotTristateBusyTerm_Routing                                         (-89127)
#define DAQmxErrorCouldNotReserveRequestedTrigLine_Routing                               (-89126)
#define DAQmxErrorTrigLineNotFound_Routing                                               (-89125)
#define DAQmxErrorRoutingPathNotAvailable_Routing                                        (-89124)
#define DAQmxErrorRoutingHardwareBusy_Routing                                            (-89123)
#define DAQmxErrorRequestedSignalInversionForRoutingNotPossible_Routing                  (-89122)
#define DAQmxErrorInvalidRoutingDestinationTerminalName_Routing                          (-89121)
#define DAQmxErrorInvalidRoutingSourceTerminalName_Routing                               (-89120)
#define DAQmxErrorServiceLocatorNotAvailable_Routing                                     (-88907)
#define DAQmxErrorCouldNotConnectToServer_Routing                                        (-88900)
#define DAQmxErrorDeviceNameContainsSpacesOrPunctuation_Routing                          (-88720)
#define DAQmxErrorDeviceNameContainsNonprintableCharacters_Routing                       (-88719)
#define DAQmxErrorDeviceNameIsEmpty_Routing                                              (-88718)
#define DAQmxErrorDeviceNameNotFound_Routing                                             (-88717)
#define DAQmxErrorLocalRemoteDriverVersionMismatch_Routing                               (-88716)
#define DAQmxErrorDuplicateDeviceName_Routing                                            (-88715)
#define DAQmxErrorRuntimeAborting_Routing                                                (-88710)
#define DAQmxErrorRuntimeAborted_Routing                                                 (-88709)
#define DAQmxErrorResourceNotInPool_Routing                                              (-88708)
#define DAQmxErrorDriverDeviceGUIDNotFound_Routing                                       (-88705)
#define DAQmxErrorPALInvalidAddressComponent                                             (-50806)
#define DAQmxErrorPALSharingViolation                                                    (-50805)
#define DAQmxErrorPALInvalidDeviceState                                                  (-50804)
#define DAQmxErrorPALConnectionReset                                                     (-50803)
#define DAQmxErrorPALConnectionAborted                                                   (-50802)
#define DAQmxErrorPALConnectionRefused                                                   (-50801)
#define DAQmxErrorPALBusResetOccurred                                                    (-50800)
#define DAQmxErrorPALWaitInterrupted                                                     (-50700)
#define DAQmxErrorPALMessageUnderflow                                                    (-50651)
#define DAQmxErrorPALMessageOverflow                                                     (-50650)
#define DAQmxErrorPALThreadAlreadyDead                                                   (-50604)
#define DAQmxErrorPALThreadStackSizeNotSupported                                         (-50603)
#define DAQmxErrorPALThreadControllerIsNotThreadCreator                                  (-50602)
#define DAQmxErrorPALThreadHasNoThreadObject                                             (-50601)
#define DAQmxErrorPALThreadCouldNotRun                                                   (-50600)
#define DAQmxErrorPALSyncTimedOut                                                        (-50550)
#define DAQmxErrorPALReceiverSocketInvalid                                               (-50503)
#define DAQmxErrorPALSocketListenerInvalid                                               (-50502)
#define DAQmxErrorPALSocketListenerAlreadyRegistered                                     (-50501)
#define DAQmxErrorPALDispatcherAlreadyExported                                           (-50500)
#define DAQmxErrorPALDMALinkEventMissed                                                  (-50450)
#define DAQmxErrorPALBusError                                                            (-50413)
#define DAQmxErrorPALRetryLimitExceeded                                                  (-50412)
#define DAQmxErrorPALTransferOverread                                                    (-50411)
#define DAQmxErrorPALTransferOverwritten                                                 (-50410)
#define DAQmxErrorPALPhysicalBufferFull                                                  (-50409)
#define DAQmxErrorPALPhysicalBufferEmpty                                                 (-50408)
#define DAQmxErrorPALLogicalBufferFull                                                   (-50407)
#define DAQmxErrorPALLogicalBufferEmpty                                                  (-50406)
#define DAQmxErrorPALTransferAborted                                                     (-50405)
#define DAQmxErrorPALTransferStopped                                                     (-50404)
#define DAQmxErrorPALTransferInProgress                                                  (-50403)
#define DAQmxErrorPALTransferNotInProgress                                               (-50402)
#define DAQmxErrorPALCommunicationsFault                                                 (-50401)
#define DAQmxErrorPALTransferTimedOut                                                    (-50400)
#define DAQmxErrorPALMemoryBlockCheckFailed                                              (-50354)
#define DAQmxErrorPALMemoryPageLockFailed                                                (-50353)
#define DAQmxErrorPALMemoryFull                                                          (-50352)
#define DAQmxErrorPALMemoryAlignmentFault                                                (-50351)
#define DAQmxErrorPALMemoryConfigurationFault                                            (-50350)
#define DAQmxErrorPALDeviceInitializationFault                                           (-50303)
#define DAQmxErrorPALDeviceNotSupported                                                  (-50302)
#define DAQmxErrorPALDeviceUnknown                                                       (-50301)
#define DAQmxErrorPALDeviceNotFound                                                      (-50300)
#define DAQmxErrorPALFeatureDisabled                                                     (-50265)
#define DAQmxErrorPALComponentBusy                                                       (-50264)
#define DAQmxErrorPALComponentAlreadyInstalled                                           (-50263)
#define DAQmxErrorPALComponentNotUnloadable                                              (-50262)
#define DAQmxErrorPALComponentNeverLoaded                                                (-50261)
#define DAQmxErrorPALComponentAlreadyLoaded                                              (-50260)
#define DAQmxErrorPALComponentCircularDependency                                         (-50259)
#define DAQmxErrorPALComponentInitializationFault                                        (-50258)
#define DAQmxErrorPALComponentImageCorrupt                                               (-50257)
#define DAQmxErrorPALFeatureNotSupported                                                 (-50256)
#define DAQmxErrorPALFunctionNotFound                                                    (-50255)
#define DAQmxErrorPALFunctionObsolete                                                    (-50254)
#define DAQmxErrorPALComponentTooNew                                                     (-50253)
#define DAQmxErrorPALComponentTooOld                                                     (-50252)
#define DAQmxErrorPALComponentNotFound                                                   (-50251)
#define DAQmxErrorPALVersionMismatch                                                     (-50250)
#define DAQmxErrorPALFileFault                                                           (-50209)
#define DAQmxErrorPALFileWriteFault                                                      (-50208)
#define DAQmxErrorPALFileReadFault                                                       (-50207)
#define DAQmxErrorPALFileSeekFault                                                       (-50206)
#define DAQmxErrorPALFileCloseFault                                                      (-50205)
#define DAQmxErrorPALFileOpenFault                                                       (-50204)
#define DAQmxErrorPALDiskFull                                                            (-50203)
#define DAQmxErrorPALOSFault                                                             (-50202)
#define DAQmxErrorPALOSInitializationFault                                               (-50201)
#define DAQmxErrorPALOSUnsupported                                                       (-50200)
#define DAQmxErrorPALCalculationOverflow                                                 (-50175)
#define DAQmxErrorPALHardwareFault                                                       (-50152)
#define DAQmxErrorPALFirmwareFault                                                       (-50151)
#define DAQmxErrorPALSoftwareFault                                                       (-50150)
#define DAQmxErrorPALMessageQueueFull                                                    (-50108)
#define DAQmxErrorPALResourceAmbiguous                                                   (-50107)
#define DAQmxErrorPALResourceBusy                                                        (-50106)
#define DAQmxErrorPALResourceInitialized                                                 (-50105)
#define DAQmxErrorPALResourceNotInitialized                                              (-50104)
#define DAQmxErrorPALResourceReserved                                                    (-50103)
#define DAQmxErrorPALResourceNotReserved                                                 (-50102)
#define DAQmxErrorPALResourceNotAvailable                                                (-50101)
#define DAQmxErrorPALResourceOwnedBySystem                                               (-50100)
#define DAQmxErrorPALBadToken                                                            (-50020)
#define DAQmxErrorPALBadThreadMultitask                                                  (-50019)
#define DAQmxErrorPALBadLibrarySpecifier                                                 (-50018)
#define DAQmxErrorPALBadAddressSpace                                                     (-50017)
#define DAQmxErrorPALBadWindowType                                                       (-50016)
#define DAQmxErrorPALBadAddressClass                                                     (-50015)
#define DAQmxErrorPALBadWriteCount                                                       (-50014)
#define DAQmxErrorPALBadWriteOffset                                                      (-50013)
#define DAQmxErrorPALBadWriteMode                                                        (-50012)
#define DAQmxErrorPALBadReadCount                                                        (-50011)
#define DAQmxErrorPALBadReadOffset                                                       (-50010)
#define DAQmxErrorPALBadReadMode                                                         (-50009)
#define DAQmxErrorPALBadCount                                                            (-50008)
#define DAQmxErrorPALBadOffset                                                           (-50007)
#define DAQmxErrorPALBadMode                                                             (-50006)
#define DAQmxErrorPALBadDataSize                                                         (-50005)
#define DAQmxErrorPALBadPointer                                                          (-50004)
#define DAQmxErrorPALBadSelector                                                         (-50003)
#define DAQmxErrorPALBadDevice                                                           (-50002)
#define DAQmxErrorPALIrrelevantAttribute                                                 (-50001)
#define DAQmxErrorPALValueConflict                                                       (-50000)
#define DAQmxWarningPALValueConflict                                                      (50000)
#define DAQmxWarningPALIrrelevantAttribute                                                (50001)
#define DAQmxWarningPALBadDevice                                                          (50002)
#define DAQmxWarningPALBadSelector                                                        (50003)
#define DAQmxWarningPALBadPointer                                                         (50004)
#define DAQmxWarningPALBadDataSize                                                        (50005)
#define DAQmxWarningPALBadMode                                                            (50006)
#define DAQmxWarningPALBadOffset                                                          (50007)
#define DAQmxWarningPALBadCount                                                           (50008)
#define DAQmxWarningPALBadReadMode                                                        (50009)
#define DAQmxWarningPALBadReadOffset                                                      (50010)
#define DAQmxWarningPALBadReadCount                                                       (50011)
#define DAQmxWarningPALBadWriteMode                                                       (50012)
#define DAQmxWarningPALBadWriteOffset                                                     (50013)
#define DAQmxWarningPALBadWriteCount                                                      (50014)
#define DAQmxWarningPALBadAddressClass                                                    (50015)
#define DAQmxWarningPALBadWindowType                                                      (50016)
#define DAQmxWarningPALBadThreadMultitask                                                 (50019)
#define DAQmxWarningPALResourceOwnedBySystem                                              (50100)
#define DAQmxWarningPALResourceNotAvailable                                               (50101)
#define DAQmxWarningPALResourceNotReserved                                                (50102)
#define DAQmxWarningPALResourceReserved                                                   (50103)
#define DAQmxWarningPALResourceNotInitialized                                             (50104)
#define DAQmxWarningPALResourceInitialized                                                (50105)
#define DAQmxWarningPALResourceBusy                                                       (50106)
#define DAQmxWarningPALResourceAmbiguous                                                  (50107)
#define DAQmxWarningPALFirmwareFault                                                      (50151)
#define DAQmxWarningPALHardwareFault                                                      (50152)
#define DAQmxWarningPALOSUnsupported                                                      (50200)
#define DAQmxWarningPALOSFault                                                            (50202)
#define DAQmxWarningPALFunctionObsolete                                                   (50254)
#define DAQmxWarningPALFunctionNotFound                                                   (50255)
#define DAQmxWarningPALFeatureNotSupported                                                (50256)
#define DAQmxWarningPALComponentInitializationFault                                       (50258)
#define DAQmxWarningPALComponentAlreadyLoaded                                             (50260)
#define DAQmxWarningPALComponentNotUnloadable                                             (50262)
#define DAQmxWarningPALMemoryAlignmentFault                                               (50351)
#define DAQmxWarningPALMemoryHeapNotEmpty                                                 (50355)
#define DAQmxWarningPALTransferNotInProgress                                              (50402)
#define DAQmxWarningPALTransferInProgress                                                 (50403)
#define DAQmxWarningPALTransferStopped                                                    (50404)
#define DAQmxWarningPALTransferAborted                                                    (50405)
#define DAQmxWarningPALLogicalBufferEmpty                                                 (50406)
#define DAQmxWarningPALLogicalBufferFull                                                  (50407)
#define DAQmxWarningPALPhysicalBufferEmpty                                                (50408)
#define DAQmxWarningPALPhysicalBufferFull                                                 (50409)
#define DAQmxWarningPALTransferOverwritten                                                (50410)
#define DAQmxWarningPALTransferOverread                                                   (50411)
#define DAQmxWarningPALDispatcherAlreadyExported                                          (50500)
#define DAQmxWarningPALSyncAbandoned                                                      (50551)


#ifdef __cplusplus
	}
#endif

#endif // __nidaqmx_h__
