package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.I2C;

@SuppressWarnings("unused")

/**
 * TCS34725ColorSensor - Simple API for the TCS34725 color sensor from adafruit
 * Provides interfacing for initializing the sensor and performing qualified reads of color values.
 * Based on Robot Casserole 1736's code here 
 * https://github.com/RobotCasserole1736/CasseroleLib/blob/master/java/src/org/usfirst/frc/team1736/lib/Sensors/TCS34725ColorSensor.java
 * But with changes:
 * 		1. using the COMMAND_BIT and COMMAND_AUTO_INCREMENT (got no response from sensor without them)
 * 		2. and adding HSV calculation
 */
public class TCS34725ColorSensor 
{
	// FRC I2C object, since color sensor will work over that
	I2C i2c;
	
	// I2C constants
	private static final int TCS34725_I2C_ADDR    =   			(0x29);	// TCS34725 I2C Address

	private static final int TCS34725_COMMAND_BIT =   			(0x80);	// must set as 1 when addressing command register
	private static final int TCS34725_COMMAND_AUTO_INCREMENT =  (0x20);	// auto-increment to read consecutive bytes

	private static final int TCS34725_ENABLE      =   (0x00);
	private static final int TCS34725_ENABLE_AIEN =   (0x10);    /* RGBC Interrupt Enable */
	private static final int TCS34725_ENABLE_WEN  =   (0x08);    /* Wait enable - Writing 1 activates the wait timer */
	private static final int TCS34725_ENABLE_AEN  =   (0x02);    /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
	private static final int TCS34725_ENABLE_PON  =   (0x01);    /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
	private static final int TCS34725_CONFIG      =   (0x0D);
	private static final int TCS34725_CONFIG_WLONG =  (0x02);    /* Choose between short and long (12x) wait times via TCS34725_WTIME */
	private static final int TCS34725_CONTROL     =   (0x0F);    /* Set the gain level for the sensor */
	private static final int TCS34725_ID          =   (0x12);    /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
	private static final int TCS34725_STATUS      =   (0x13);
	private static final int TCS34725_STATUS_AINT =   (0x10);    /* RGBC Clean channel interrupt */
	private static final int TCS34725_STATUS_AVALID = (0x01);    /* Indicates that the RGBC channels have completed an integration cycle */
	private static final int TCS34725_CDATAL      =   (0x14);    /* Clear channel data */
	private static final int TCS34725_CDATAH      =   (0x15);
	private static final int TCS34725_RDATAL      =   (0x16);    /* Red channel data */
	private static final int TCS34725_RDATAH      =   (0x17);
	private static final int TCS34725_GDATAL      =   (0x18);    /* Green channel data */
	private static final int TCS34725_GDATAH      =   (0x19);
	private static final int TCS34725_BDATAL      =   (0x1A);    /* Blue channel data */
	private static final int TCS34725_BDATAH      =   (0x1B);

	private static final int TCS34725_ATIME       =   (0x01);    /* Integration time */
	private static final int TCS34725_INTEGRATIONTIME_2_4MS  = 0xFF;   /**<  2.4ms - 1 cycle    - Max Count: 1024  */
	private static final int TCS34725_INTEGRATIONTIME_24MS   = 0xF6;   /**<  24ms  - 10 cycles  - Max Count: 10240 */
	private static final int TCS34725_INTEGRATIONTIME_50MS   = 0xEB;   /**<  50ms  - 20 cycles  - Max Count: 20480 */
	private static final int TCS34725_INTEGRATIONTIME_101MS  = 0xD5;   /**<  101ms - 42 cycles  - Max Count: 43008 */
	private static final int TCS34725_INTEGRATIONTIME_154MS  = 0xC0;   /**<  154ms - 64 cycles  - Max Count: 65535 */
	private static final int TCS34725_INTEGRATIONTIME_700MS  = 0x00;    /**<  700ms - 256 cycles - Max Count: 65535 */
		  
	private static final int TCS34725_GAIN_1X                = 0x00;   /**<  No gain  */
	private static final int TCS34725_GAIN_4X                = 0x01;   /**<  4x gain  */
	private static final int TCS34725_GAIN_16X               = 0x02;   /**<  16x gain */
	private static final int TCS34725_GAIN_60X               = 0x03;   /**<  60x gain */
	
	
	//State Variables
	public boolean sensor_initialized;
	public boolean good_data_read;
	
	
	/**
	 * Constructor for TCS34725 Color Sensor from Adafruit.
	 * Initializes internal data structures and opens I2C coms to the device.
	 */
	public TCS34725ColorSensor()
	{
		i2c = new I2C(I2C.Port.kOnboard, TCS34725_I2C_ADDR);
		sensor_initialized = false;
		good_data_read = false;
	}
	

	/**
	 * init - initializes the actual sensor state so colors can be read.
	 * By default the sensor powers up to a "disabled" state. This enables it.
	 * Additionally, checks the sensor has the proper internal ID and 
	 * sets hard-coded gains and integrator times.
	 * @return 0 on success, -1 on failure to initialize
	 */
	public int init()
	{
		sensor_initialized = false;
		
		System.out.print("Checking for Color Sensor...");
		
		if (!isSensorPresent())
		{
			System.out.println("Error - DeviceID register mismatch on Color Sensor!  Cannot Initalize!");
			return -1;
		}
		else
		{
			System.out.println("found!");
		}

		System.out.print("Initalizing Color Sensor...");
		
		//Set the integration time
		write8(TCS34725_ATIME, TCS34725_INTEGRATIONTIME_50MS);
		
		//Set the gain
		write8(TCS34725_CONTROL, TCS34725_GAIN_4X);
		
		//Power-on the sensor's internals (it defaults to off)
		write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
		safeSleep(3);
		write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);	

		sensor_initialized = true;
		System.out.println("done!");
		return 0;
	}

	public boolean isSensorPresent()
	{
		//Check we're actually connected to the sensor
		int deviceID = read8(TCS34725_ID);
		// System.out.printf("Color Sensor DeviceID = 0x%02X\n", deviceID);
		// deviceID: 0x44 --> TCS34725
		// deviceID: 0x4D --> TCS34727
		boolean sensorPresent = ((deviceID == 0x44) || (deviceID == 0x4D));
		return sensorPresent;
	}

	/**
	 * readColors - queries the sensor for the red, green, blue, and clear values
	 * Qualifies the read to ensure the sensor has not been reset since the last read.
	 * @return 0 on read success, -1 on failure.
	 */
	public TCSColor readColors()
	{
		int r = 0;
		int g = 0;
		int b = 0;
		int c = 0;

		//Don't bother doing anything if the sensor isn't initialized
		if (!sensor_initialized)
		{
			System.out.println("Error: Attempt to read from color sensor, but it's not initalized!");
			return new TCSColor(r, b, g, c);
		}
		
		//Call the read bad if the enable register isn't set properly
		//(this gets reset to a different value if the sensor is power-cycled)
		int enable_test_buf = read8(TCS34725_ENABLE);
		if (enable_test_buf != (TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN))
		{
			System.out.println("Error: Attempt to read from color sensor, but the enable register did not read as expected! Sensor has probably been reset.");
			sensor_initialized = false;
			good_data_read = false;
			return new TCSColor(r, b, g, c);
		}
		
		//Read data off of the sensor
		r = read16(TCS34725_RDATAL);
		g = read16(TCS34725_GDATAL);
		b = read16(TCS34725_BDATAL);
		c = read16(TCS34725_CDATAL);

		//Set that we've got good data and return.
		good_data_read = true;
		return new TCSColor(r, g, b, c);
	}
	

	/**
	 * Writes an 8 bit value over I2C
	 * @param addr the register to write the data to
	 * @param value a byte of data to write
	 * @return whatever I2CJNI.i2CWrite returns. It's not documented in the wpilib javadocs!
	 */
	private boolean write8(int addr, int value) {
		boolean retVal = false;
		retVal = i2c.write(TCS34725_COMMAND_BIT | addr, (byte)(value & 0xFF));
		return retVal;
	}

	/**
	 * Reads an 8 bit value over I2C
	 * @param addr the register to read from.
	 * @return
	 */
	private int read8(int addr) {
		byte[] buffer = new byte[1];
		i2c.read(TCS34725_COMMAND_BIT | addr, 1, buffer);
		return (buffer[0] & 0xFF);
	}

	/**
	 * Reads an 16 bit value over I2C
	 * @param addr the register to read from.
	 * @return
	 */
	private int read16(int addr) {
		byte[] buffer = new byte[2];
		i2c.read(TCS34725_COMMAND_BIT | TCS34725_COMMAND_AUTO_INCREMENT | addr, 2, buffer);
		int ret_val = ((buffer[1] & 0xFF) << 8) | (buffer[0] & 0xFF);
		return ret_val;
	}

	/**
	 * Reads the specified number of bytes over I2C
	 *
	 * @param addr the address to read from
	 * @param buffer the size of the data to read
	 * @return true on success
	 */
	private boolean readLen(int addr, byte[] buffer) {
		boolean retVal = true;
		if (buffer == null || buffer.length < 1) {
			return false;
		}

		retVal = !i2c.read(TCS34725_COMMAND_BIT | TCS34725_COMMAND_AUTO_INCREMENT | addr, buffer.length, buffer);
		return retVal;
	}

	/**
	 * safeSleep - wrapped Thread.sleep call to safely delay for a given period of time
	 * @param milliseconds - time to delay for in ms.
	 */
	private void safeSleep(long milliseconds)
	{
		try 
		{
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) 
		{
			e.printStackTrace();
		}
	}


	public static class TCSColor {
		private int r, g, b, c;
		private int h, s, v;

		public TCSColor(int r, int g, int b, int c) {
			this.r = r;
			this.g = g;
			this.b = b;
			this.c = c;

			// convert RGB to HSV
			// first, convert to floating point numbers between 0 and 1
			double rr = (double)r/255.0;
			double gg = (double)g/255.0;
			double bb = (double)b/255.0;

			double max = Math.max(Math.max(rr,gg),bb);
			double min = Math.min(Math.min(rr,gg),bb);
			double hh = 0.0;	// floating point hue, in degrees

			if (max == min)	{
				hh = 0.0;
			}
			else if (max == rr) {
				hh = 60.0 * (0.0 + (gg-bb)/(max-min));
			}
			else if (max == gg)	{
				hh = 60.0 * (2.0 + (bb-rr)/(max-min));
			}
			else { // (max == bb)
				hh = 60.0 * (4.0 + (rr-gg)/(max-min));
			}
			if (hh < 0.0) {
				hh = hh + 360.0;
			}

			double ss = 0.0;
			if (max == 0){
				ss = 0.0;
			}
			else{
				ss = (max-min)/max;
			}
	
			double vv = max;

			// convert to 8-bit integers
			h = (int)(hh/360.0*255.0);
			s = (int)(ss*255.0);
			v = (int)(vv*255.0);
		}

		public int getR() {
			return this.r;
		}

		public int getB() {
			return this.b;
		}

		public int getG() {
			return this.g;
		}

		public int getC() {
			return this.c;
		}

		public int getH() {
			return this.h;
		}

		public int getS() {
			return this.s;
		}

		public int getV() {
			return this.v;
		}

		public String toString() {
			return String.format("[R:% 5d,   G:% 5d,   B:% 5d,   C:% 5d,   H:% 5d,   S:% 5d,   V:% 5d]", r, g, b, c, h, s, v);
		}
	}	
}
