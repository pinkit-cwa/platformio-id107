#include <SPI.h>
#include <KX022.h>
#include <SSD1306Spi.h>
#include <SI114.h>
#include <BLEPeripheral.h>
#include <TimeLib.h>//get this lib here: https://github.com/PaulStoffregen/Time
#include <compile_time.h>//macro for build time as unix time, for setting clock,optional
#define BOOTLOADER_DFU_START 0xBB



BLEPeripheral blePeripheral;  // BLE Peripheral Device
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE Service

// BLE LED Switch to DFU Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedCharCharacteristic switchCharacteristic("00000af6-0000-1000-8000-00805f9b34fb", BLERead | BLEWrite);


KX022<> acc(Wire);

#define OLED_WIDTH 64
#define OLED_HEIGHT 32
SSD1306Spi oled(OLED_RST, OLED_DC, OLED_CS); // (pin_rst, pin_dc, pin_cs)

float xyz[3];
uint32_t tPage, start_meting;
bool B1_isPressed = false;
uint8_t page_num = 1;
const uint8_t page_count = 6;


const int SAMPLES_TO_AVERAGE = 5;
int binOut;     // 1 or 0 depending on state of heartbeat
int teller,perioden;
int BPM;
long meting,minmeting=999999999999,maxmeting,gemiddeldemeting, vorigeamplitude=1, vorigemin;

int signalSize;          // the heartbeat signal minus the offset
int ypos[65];

//heartrate sensor si114
const int portForSI114 = 0; //for SI114 JJ
PortI2C myBus (portForSI114);
PulsePlug pulse (myBus);

void draw_page(uint8_t idx = 0);
String content = "";
int heartsensor_read=0;

void setup()
{
	NRF_WDT->RR[0] = WDT_RR_RR_Reload; //reload watchdogtimer to avoid reset
	setTime(__TIME_UNIX__);//set clock to build time
	Wire.begin();

	//Serial.begin(115200);
	//Serial.println(__FILE__);

	blePeripheral.setLocalName("id107:OpenSource");
	blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

	// add service and characteristics
	blePeripheral.addAttribute(ledService);
	blePeripheral.addAttribute(switchCharacteristic);

	switchCharacteristic.setValue(0);

	// begin initialization
	blePeripheral.begin();


	pinMode(PIN_BUTTON1, INPUT_PULLUP); 
	pinMode(PIN_BUTTON2, INPUT);
	pinMode(PIN_VIBRATE, OUTPUT);

	acc.init();

	oled.setScreenSize(OLED_WIDTH, OLED_HEIGHT);
	oled.init();
	oled.flipScreenVertically();
	oled.setTextAlignment(TEXT_ALIGN_LEFT);
	oled.setFont(ArialMT_Plain_10);
	draw_page(page_num);
	//delay(3000); // show splash for 3s
	tPage = millis();
	start_meting = millis();
	//set gpregret register to some value to enable DFU
	NRF_POWER->GPREGRET = BOOTLOADER_DFU_START;
}

void loop()
{
	NRF_WDT->RR[0] = WDT_RR_RR_Reload; //reload watchdog timer
	//  read out side button -- put the device in DFU mode 
	if (digitalRead(PIN_BUTTON1)==0) // reset the watch into dfu mode 
	{
		//	sd_power_gpregret_set(0xB1); //  necessary 
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		sd_power_gpregret_clr(0xffffffff);
		NRF_POWER->GPREGRET = BOOTLOADER_DFU_START;
		//NVIC_DisableIRQ(GPIOTE_IRQn);
		NVIC_SystemReset();
	}

	blePeripheral.poll();
	if (switchCharacteristic.value()) //if a value is written to the dfu service the device reboots in DFU mode 
	{
		if (switchCharacteristic.written()) 
		{
			//sd_power_gpregret_clr(0xffffffff);
			//		sd_power_gpregret_set(0xB4);
			NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		        //sd_power_gpregret_clr(0xffffffff);
			NRF_POWER->GPREGRET = BOOTLOADER_DFU_START;
			//dfu_app_reset_prepare_set();
			//sd_power_reset_reason_clr();
			//sd_softdevice_disable();
			//SD_POWER_SYSTEM_OFF
			//NVIC_DisableIRQ(GPIOTE_IRQn);
			//NRF_RADIO->TASKS_DISABLE = 1;
			//NRF_RADIO->POWER = 0;
			//SD_SOFTDEVICE_DISABLE;
			NVIC_SystemReset();
		}
	}
	// menu system with touch button
	if (digitalRead(PIN_BUTTON2)){
		page_num = (page_num + 1 < page_count) ? page_num + 1 : 0;
		delay (800);
		digitalWrite(PIN_VIBRATE,1); //vibrate if touched
		delay (40);
		digitalWrite(PIN_VIBRATE,0);
	}



	if (millis() - tPage > 20) // 20ms = 50Hz
	{
		tPage = millis();
		draw_page(page_num);
	}

	//activate si114x sensor
	//only when heartbeatgraph/BPM menu is displayed

	if ((page_num==3) || (page_num==4) || (page_num==5))
	{      
		if (heartsensor_read==0) initPulseSensor();
		readPulseSensorPSO2(); //heart rate sensor
		heartsensor_read=1;
	}
	else
	{
		heartsensor_read=0;
		endPulseSensor(); //stop heartbeat measurement
	}

	yield();
}

void float2chars(float &in, char (&out)[5])
{
	bool sign_bit = (in < 0);
	uint16_t tmp = sign_bit ? (-in * 10) : (in * 10);
	out[0] = (sign_bit) ? '-' : ' ';
	out[1] = char('0' + (tmp / 10));
	out[2] = '.';
	out[3] = char('0' + (tmp % 10));
	out[4] = '\0';
}

void draw_page(uint8_t idx)
{
	switch (idx)
	{
		case 1:
			page_clock(); 
			break;
		case 2:
			page_accelerometer(); 
			break;
		case 3:
			page_graphheartbeat(); 
			break;
		case 4:
			page_bpm(); 
			break;
		case 5:
			page_meting(); 
			break;
		default:
			page_startup();
			break;
	}
}

void page_clock()
{
	char buffer[11];
	time_t time_now = now();
	oled.clear();
	oled.drawString(0, 0, "CLOCK");
	sprintf(buffer,"%02d.%02d.%04d",day(time_now),month(time_now),year(time_now));
	oled.drawString(0, 10, buffer);
	sprintf(buffer,"%02d:%02d:%02d",hour(time_now),minute(time_now),second(time_now));
	oled.drawString(0,20,buffer);
	oled.display();
}

void page_graphheartbeat()
{ 
	int max_y=0, min_y=9999999;
	oled.clear();
	for (int i=0;i<64;i++)
	{
		oled.setPixel(i, int(ypos[i])); 
	}     

	oled.display();
}

void page_bpm()
{
	String stringOne; 
	int steps_start, steps_end, steps_period;
	int crossed=0;
	int bpm;
	oled.clear();
	for (int i=0;i<64;i++)
	{
		if ((ypos[i] > 13) && (ypos[i] < 16) && (crossed==0)) //curve is crossing zero line
		{
			steps_start=i;
			crossed=1;
		}
		if ((ypos[i] > 13) && (ypos[i] < 16) && (crossed==1) && (i > steps_start+4)) //curve is crossing zero line     
		{
			steps_end=i;
			crossed=2;
		}
		if ((ypos[i] > 13) && (ypos[i] < 16) && (crossed==2) && (i > steps_end+4)) //curve is crossing zero line     
		{
			steps_period=i; //third crossing
			crossed=3;
		}

	}
	if (crossed==3){
		bpm=((steps_period-steps_start)*32)*6/100; //each step is 32 milliseconds
		stringOne = String(bpm);
	}
	oled.drawString(0, 0, "BPM"); 
	oled.drawString(0, 10, stringOne); 
	oled.display();
}

void page_meting()
{
	String stringOne; 
	stringOne = String(meting);
	oled.clear();
	oled.drawString(0, 0, "VALUE:"); 
	oled.drawString(0, 10, stringOne); 
	oled.display();
}


void page_startup()
{

	oled.clear();
	oled.drawString(0, 0, "NEW"); //RTC todo
	oled.drawString(0, 10, "BL"); //RTC todo
	oled.drawString(0, 20, "TESTing"); //RTC todo
	oled.display();
}

void page_accelerometer()
{
	char fltBuf[5];
	acc.getAccelXYZ(xyz);
	oled.clear();
	float2chars(xyz[0], fltBuf);
	oled.drawString(0, 0, "X:"); oled.drawString(10, 0, fltBuf);
	float2chars(xyz[1], fltBuf);
	oled.drawString(0, 10, "Y:"); oled.drawString(10, 10, fltBuf);
	float2chars(xyz[2], fltBuf);
	oled.drawString(0, 20, "Z:"); oled.drawString(10, 20, fltBuf);
	oled.display();


}

void endPulseSensor() 
{
	pulse.setReg(PulsePlug::PS_LED21, 0x00);      // LED current for 2 (IR1 - high nibble) & LEDs 1 (red - low nibble) 
	pulse.writeParam(PulsePlug::PARAM_CH_LIST, 0x00);         // all measurements off
}
void initPulseSensor() {

	pulse.setReg(PulsePlug::HW_KEY, 0x17);
	pulse.setReg(PulsePlug::INT_CFG, 0x03);       // turn on interrupts
	pulse.setReg(PulsePlug::IRQ_ENABLE, 0x04);    // turn on interrupt on PS1
	pulse.setReg(PulsePlug::IRQ_MODE1, 0x30);
	pulse.setReg(PulsePlug::MEAS_RATE, 0x99);     //setting jacob 
	pulse.setReg(PulsePlug::ALS_RATE, 0x08);      // see datasheet
	pulse.setReg(PulsePlug::PS_RATE, 0x08);       // see datasheet



	// Current setting for LEDs pulsed while taking readings
	// PS_LED21  Setting for LEDs 1 & 2. LED 2 is high nibble
	// each LED has 16 possible (0-F in hex) possible settings
	// read the

	//pulse.setReg(PulsePlug::PS_LED21, 0xff);      // LED current for 2 (IR1 - high nibble) & LEDs 1 (red - low nibble) 
	pulse.setReg(PulsePlug::PS_LED21, 0xbb);      // setting jacob 
	//when measuring tip of finger 0x11 will do

	pulse.writeParam(PulsePlug::PARAM_CH_LIST, 0x77);         // all measurements on

	// increasing PARAM_PS_ADC_GAIN will increase the LED on time and ADC window
	// you will see increase in brightness of visible LED's, ADC output, & noise
	// datasheet warns not to go beyond 4 because chip or LEDs may be damaged

	//pulse.writeParam(PulsePlug::PARAM_PS_ADC_GAIN, 0x02); // when set to 0 you get good measurements on tip of finger but not around wrist
	pulse.writeParam(PulsePlug::PARAM_PS_ADC_GAIN, 0x04); //setting jacob 
	// You can select which LEDs are energized for each reading.
	// The settings below turn on only the LED that "normally" would be read
	// ie LED1 is pulsed and read first, then LED2 is pulsed and read etc.
	pulse.writeParam(PulsePlug::PARAM_PSLED12_SELECT, 0x21);  // 21 = LED 2 & LED 1 (red) resp.


	pulse.writeParam(PulsePlug::PARAM_PS1_ADCMUX, 0x03);      // PS1 photodiode select
	pulse.writeParam(PulsePlug::PARAM_PS_ADC_COUNTER, B01110000);    // B01110000 is default
	//start an autonomous read loop
	pulse.setReg(PulsePlug::COMMAND, PulsePlug::PS_AUTO_cmd);


}

void readPulseSensorPSO2() {
	meting = (long)pulse.ps1;
	int max_y=0, min_y=99999999999, j;
	pulse.fetchLedData();
	if (meting < 0) meting=0;
	if ((meting > maxmeting) && (meting < 500000000)) maxmeting=meting;
	if ((meting < minmeting) && (meting > 500000)) minmeting=meting;
	//if (meting < minmeting) minmeting=meting;
	//if (meting > maxmeting) maxmeting=meting;

	//	if ((meting > 500000) && (meting < 500000000)){
	//gemiddeldemeting =  meting;
	//	}


	//every 2 seconds update the max and the min value
	//calculate the difference

	if (millis() > start_meting + 2000)
	{
		//
		// measure during 2 seconds to determine max and min

		vorigeamplitude = maxmeting-minmeting;
		vorigemin = minmeting;
		maxmeting = 0; //reset max & min
		minmeting = 9999999999;
		perioden=0;
		start_meting = millis();
		//	gemiddeldemeting=0;
	}

	// display measurement every 32 milliseconds


	if (millis() > (start_meting + 32*perioden)) //take a sample each 32 milliseconds
	{
		//	if (gemiddeldemeting < vorigemin) gemiddeldemeting=vorigemin;
		//	if (vorigeamplitude > 0)
		ypos[perioden] = (((meting-vorigemin)*32)/vorigeamplitude);
		//ypos[perioden] = int(meting);
		perioden++;
		//	gemiddeldemeting=0;

	}

}


