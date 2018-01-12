/*
 * name:        HTS221
 * description: Capacitive digital sensor for relative humidity and temperature.
 * manuf:       ST Microelectronics
 * version:     Version 0.1
 * url:         http://www.st.com/resource/en/datasheet/hts221.pdf
 * date:        2018-01-12
 * author       https://chisl.io/
 * file:        HTS221.hpp
 */

/*                                                                                                       *
 *                                   THIS FILE IS AUTOMATICALLY CREATED                                  *
 *                                    D O     N O T     M O D I F Y  !                                   *
 *                                                                                                       */

#include <cinttypes>

/* Derive from class HTS221_Base and implement the read and write functions! */

/* HTS221: Capacitive digital sensor for relative humidity and temperature. */
class HTS221_Base
{
public:
	/* Pure virtual functions that need to be implemented in derived class: */
	virtual uint8_t read8(uint16_t address, uint16_t n=8) = 0;  // 8 bit read
	virtual void write(uint16_t address, uint8_t value, uint16_t n=8) = 0;  // 8 bit write
	virtual uint16_t read16(uint16_t address, uint16_t n=16) = 0;  // 16 bit read
	virtual void write(uint16_t address, uint16_t value, uint16_t n=16) = 0;  // 16 bit write
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG WHO_AM_I                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG WHO_AM_I:
	 * 7.1
	 * Device identification. This read-only register contains the device identifier, set to BCh
	 */
	struct WHO_AM_I
	{
		static const uint16_t __address = 15;
		
		/* Bits WHO_AM_I: */
		struct WHO_AM_I_
		{
			/* Mode:r */
			static const uint8_t dflt = 0b10111100; // 8'b10111100
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register WHO_AM_I */
	void setWHO_AM_I(uint8_t value)
	{
		write(WHO_AM_I::__address, value, 8);
	}
	
	/* Get register WHO_AM_I */
	uint8_t getWHO_AM_I()
	{
		return read8(WHO_AM_I::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG AV_CONF                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG AV_CONF:
	 * 7.2
	 * Humidity and temperature resolution mode. To configure humidity/temperature average.
	 * Table 16. Humidity and temperature average configuration
	 * |               Nr. internal  average            |    Noise (RMS)   | IDD 1 Hz |
	 * | AVGx2:0 | Temperature (AVGT) | Humidity (AVGH) | Temp (°C) | rH % |    μA    |
	 * | 000     |                  2 |               4 |      0.08 |  0.4 |     0.80 |
	 * | 001     |                  4 |               8 |      0.05 |  0.3 |     1.05 |
	 * | 010     |                  8 |              16 |      0.04 |  0.2 |     1.40 |
	 * | 011     |                 16 |              32 |      0.03 | 0.15 |     2.10 |
	 * | 100     |                 32 |              64 |      0.02 |  0.1 |     3.43 |
	 * | 101     |                 64 |             128 |     0.015 | 0.07 |     6.15 |
	 * | 110     |                128 |             256 |      0.01 | 0.05 |    11.60 |
	 * | 111     |                256 |             512 |     0.007 | 0.03 |    22.50 |
	 * 
	 */
	struct AV_CONF
	{
		static const uint16_t __address = 16;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits AVGT: */
		/* To select the numbers of averaged temperature samples (2 - 256), see Table 16.  */
		struct AVGT
		{
			static const uint8_t dflt = 0b011; // 3'b11
			static const uint8_t mask = 0b00111000; // [3,4,5]
			static const uint8_t T2 = 0b00; // 
			static const uint8_t T4 = 0b01; // 
			static const uint8_t T8 = 0b10; // 
			static const uint8_t T16 = 0b11; // 
			static const uint8_t T32 = 0b100; // 
			static const uint8_t T64 = 0b101; // 
			static const uint8_t T128 = 0b110; // 
			static const uint8_t T256 = 0b111; // 
		};
		/* Bits AVGH: */
		/* To select the numbers of averaged humidity samples (4 - 512), see Table 16.  */
		struct AVGH
		{
			static const uint8_t dflt = 0b011; // 3'b11
			static const uint8_t mask = 0b00000111; // [0,1,2]
			static const uint8_t H4 = 0b00; // 
			static const uint8_t H8 = 0b01; // 
			static const uint8_t H16 = 0b10; // 
			static const uint8_t H32 = 0b11; // 
			static const uint8_t H64 = 0b100; // 
			static const uint8_t H128 = 0b101; // 
			static const uint8_t H256 = 0b110; // 
			static const uint8_t H512 = 0b111; // 
		};
	};
	
	/* Set register AV_CONF */
	void setAV_CONF(uint8_t value)
	{
		write(AV_CONF::__address, value, 8);
	}
	
	/* Get register AV_CONF */
	uint8_t getAV_CONF()
	{
		return read8(AV_CONF::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG1                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG1:
	 * 7.3
	 * Control register 1
	 */
	struct CTRL_REG1
	{
		static const uint16_t __address = 32;
		
		/* Bits PD: */
		/* power-down control  */
		struct PD
		{
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t POWER_DOWN = 0b0; // power-down mode
			static const uint8_t ACTIVE = 0b1; // active mode
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b01111000; // [3,4,5,6]
		};
		/* Bits BDU: */
		/* block data update  */
		struct BDU
		{
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t CONTINUOUS = 0b0; // continuous update
			static const uint8_t NOT_UPDATED_UNTIL_READ = 0b1; // output registers not updated until MSB and LSB reading)
		};
		/* Bits ODR: */
		/*
		 * output data rate selection (see table 17)
		 * The PD bit is used to turn on the device. The device is in power-down mode when PD = ‘0’
		 * (default value after boot). The device is active when PD is set to ‘1’.
		 * The BDU bit is used to inhibit the output register update between the reading of the upper and lower register parts. In default mode (BDU = ‘0’), the lower and upper register parts are updated continuously. If it is not certain whether the read will be faster than output data rate, it is recommended to set the BDU bit to ‘1’. In this way, after the reading of the lower (upper) register part, the content of that output register is not updated until the upper (lower) part is read also.
		 * This feature prevents the reading of LSB and MSB related to different samples. The ODR1 and ODR0 bits permit changes to the output data rates of humidity and
		 * temperature samples.The default value corresponds to a “one-shot” configuration for both
		 * humidity and temperature output. ODR1 and ODR0 can be configured as described in
		 * Table 17. Output data rate configuration
		 * ODR1 ODR0  Humidity (Hz)  Temperature  (Hz)
		 * 0       0       One-shot           One-shot
		 * 0       1           1 Hz               1 Hz
		 * 1       0           7 Hz               7 Hz
		 * 1       1        12.5 Hz            12.5 Hz
		 * 
		 */
		struct ODR
		{
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t ONE_SHOT = 0b00; // 
			static const uint8_t RATE_1_HZ = 0b01; // 
			static const uint8_t RATE_7_HZ = 0b10; // 
			static const uint8_t RATE_12_5_HZ = 0b11; // 
		};
	};
	
	/* Set register CTRL_REG1 */
	void setCTRL_REG1(uint8_t value)
	{
		write(CTRL_REG1::__address, value, 8);
	}
	
	/* Get register CTRL_REG1 */
	uint8_t getCTRL_REG1()
	{
		return read8(CTRL_REG1::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG2                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG2:
	 * 7.4
	 * Control register 2
	 */
	struct CTRL_REG2
	{
		static const uint16_t __address = 33;
		
		/* Bits BOOT: */
		/*
		 * Reboot memory content
		 * The BOOT bit is used to refresh the content of the internal registers stored in the Flash
		 * memory block. At device power-up, the content of the Flash memory block is transferred to the internal registers related to trimming functions to permit good behavior of the device itself. If, for any reason, the content of the trimming registers is modified, it is sufficient to use this bit to restore the correct values. When the BOOT bit is set to ‘1’ the content of the internal Flash is copied inside the corresponding internal registers and is used to calibrate the device. These values are factory trimmed and are different for every device. They permit good behavior of the device and normally they should not be changed. At the end of the boot process, the BOOT bit is set again to ‘0’.
		 */
		struct BOOT
		{
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t REBOOT_MEMORY = 0b1; // reboot memory content
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b01111100; // [2,3,4,5,6]
		};
		/* Bits Heater: */
		/*
		 * The Heater bit is used to control an internal heating element, that can effectively be used to speed up the sensor recovery time in case of condensation. The heater can be operated only by an external controller, which means that it has to be switched on/off directly by FW. Humidity and temperature output should not be read during the heating cycle; valid data can be read out once the heater has been turned off, after the completion of the heating cycle. Typical power consumption related to VDD is described in Table 18.
		 * Table 18. Typical power consumption with heater ON:
		 * VDD [V]   I [mA]
		 * 3.3           33
		 * 2.5	        22
		 * 1.8	        12
		 * 
		 */
		struct Heater
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // heater disable
			static const uint8_t ENABLE = 0b1; // heater enable
		};
		/* Bits ONE_SHOT_ENABLE: */
		/*
		 * One-shot enable
		 * The ONE_SHOT bit is used to start a new conversion. In this situation a single acquisition of temperature and humidity is started when the ONE_SHOT bit is set to ‘1’. At the end of conversion the new data are available in the output registers, the STATUS_REGBIT 0 =  and STATUS_REGBIT 1 =  bits are set to ‘1’ and the ONE_SHOT bit comes back to ‘0’ by hardware.
		 */
		struct ONE_SHOT_ENABLE
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t WAITING = 0b0; // waiting for start of conversion
			static const uint8_t START = 0b1; // start for a new dataset)
		};
	};
	
	/* Set register CTRL_REG2 */
	void setCTRL_REG2(uint8_t value)
	{
		write(CTRL_REG2::__address, value, 8);
	}
	
	/* Get register CTRL_REG2 */
	uint8_t getCTRL_REG2()
	{
		return read8(CTRL_REG2::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG3                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG3:
	 * 7.5
	 * Control register 3: Control register for data ready output signal
	 */
	struct CTRL_REG3
	{
		static const uint16_t __address = 34;
		
		/* Bits DRDY_H_L: */
		/* Data Ready output signal active high, low  */
		struct DRDY_H_L
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t ACTIVE_HIGH = 0b0; // active high - default
			static const uint8_t ACTIVE_LOW = 0b1; // active low
		};
		/* Bits PP_OD: */
		/* Push-pull / Open Drain selection on pin 3 (DRDY)  */
		struct PP_OD
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t PUSH_PULL = 0b0; // push-pull - default
			static const uint8_t OPEN_DAIN = 0b1; // open drain
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b00111000; // [3,4,5]
		};
		/* Bits DRDY_EN: */
		/*
		 * Data Ready enable
		 * The DRDY_EN bit enables the DRDY signal on pin 3. Normally inactive, the DRDY output
		 * signal becomes active on new data available: logical OR of the bits STATUS_REGBIT[1]
		 * and STATUS_REGBIT[0] for humidity and temperature, respectively. The DRDY signal returns
		 * inactive after both HUMIDITY_OUT_H and TEMP_OUT_H registers are read.
		 */
		struct DRDY_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t DISABLED = 0b0; // Data Ready disabled - default
			static const uint8_t SIGNAL_AVAILABLE = 0b1; // Data Ready signal available on pin 3
		};
		/* Bits reserved_1: */
		struct reserved_1
		{
			static const uint8_t mask = 0b00000011; // [0,1]
		};
	};
	
	/* Set register CTRL_REG3 */
	void setCTRL_REG3(uint8_t value)
	{
		write(CTRL_REG3::__address, value, 8);
	}
	
	/* Get register CTRL_REG3 */
	uint8_t getCTRL_REG3()
	{
		return read8(CTRL_REG3::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG STATUS_REG                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG STATUS_REG:
	 * 7.6
	 * Status register; the content of this register is updated every one-shot reading, and after
	 * completion of every ODR cycle, regardless of the BDU value in CTRL_REG1.
	 */
	struct STATUS_REG
	{
		static const uint16_t __address = 39;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b11111100; // [2,3,4,5,6,7]
		};
		/* Bits H_DA: */
		/*
		 * Humidity data available.
		 * H_DA is set to 1 whenever a new humidity sample is available. H_DA is cleared anytime
		 * HUMIDITY_OUT_H (29h) register is read.
		 */
		struct H_DA
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NOT_AVAILABLE = 0b0; // new data for humidity is not yet available
			static const uint8_t AVAILABLE = 0b1; // new data for humidity is available
		};
		/* Bits T_DA: */
		/*
		 * Temperature data available.
		 * T_DA is set to 1 whenever a new temperature sample is available. T_DA is cleared anytime TEMP_OUT_H (2Bh) register is read.
		 * Relative humidity data (LSB)
		 */
		struct T_DA
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NOT_AVAILABLE = 0b0; // new data for temperature is not yet available
			static const uint8_t AVAILABLE = 0b1; // new data for temperature is available
		};
	};
	
	/* Set register STATUS_REG */
	void setSTATUS_REG(uint8_t value)
	{
		write(STATUS_REG::__address, value, 8);
	}
	
	/* Get register STATUS_REG */
	uint8_t getSTATUS_REG()
	{
		return read8(STATUS_REG::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG HUMIDITY_OUT                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG HUMIDITY_OUT:
	 * 7.8
	 * Relative humidity data
	 * Humidity data are expressed as HUMIDITY_OUT_H & HUMIDITY_OUT_L in 2’s complement. Values exceeding the operating humidity range (see Table 3) must be clipped by SW.
	 */
	struct HUMIDITY_OUT
	{
		static const uint16_t __address = 41;
		
		/* Bits HUMIDITY_OUT: */
		struct HUMIDITY_OUT_
		{
			/* Mode: */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register HUMIDITY_OUT */
	void setHUMIDITY_OUT(uint16_t value)
	{
		write(HUMIDITY_OUT::__address, value, 16);
	}
	
	/* Get register HUMIDITY_OUT */
	uint16_t getHUMIDITY_OUT()
	{
		return read16(HUMIDITY_OUT::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG TEMP_OUT_L                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG TEMP_OUT_L:
	 * 7.9
	 * Temperature data
	 * Temperature data are expressed as TEMP_OUT_H & TEMP_OUT_L as 2’s complement numbers.
	 * The relative humidity and temperature values must be computed by linear interpolation of current registers with calibration registers, according to Table 19 and scaling as described in Section 8.
	 */
	struct TEMP_OUT_L
	{
		static const uint16_t __address = 42;
		
		/* Bits TEMP_OUT_L: */
		struct TEMP_OUT_L_
		{
			/* Mode: */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register TEMP_OUT_L */
	void setTEMP_OUT_L(uint8_t value)
	{
		write(TEMP_OUT_L::__address, value, 8);
	}
	
	/* Get register TEMP_OUT_L */
	uint8_t getTEMP_OUT_L()
	{
		return read8(TEMP_OUT_L::__address, 8);
	}
	
};
