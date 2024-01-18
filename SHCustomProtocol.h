#ifndef __SHCUSTOMPROTOCOL_H__
#define __SHCUSTOMPROTOCOL_H__

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <mcp_can.h>
#include <SPI.h>

const uint8_t pin_ebrake = 4;
const uint8_t pin_abs = 5;
const uint8_t pin_speedo 6;
const uint8_t pin_kbus_tx = 7;
const uint8_t pin_kbus_rx = 8;
const uint8_t pin_cs = 9;
const uint8_t pin_backlight = 10;

MCP_CAN CAN(pin_cs);
SoftwareSerial kbus (pin_kbus_rx, pin_kbus_tx);

//Time variables
uint8_t hour = 0;
uint8_t minute = 0;

// Set Cluster Variables
const unsigned int cluster_max_rpm = 7000;
unsigned int output_rpm = 0.00f;
unsigned int output_speed = 50;
unsigned int output_temp = 50.00f;

// K-Bus Variables
byte kbus_lightbyte_1 = 0x00;
byte kbus_lightbyte_2 = 0x00;

// Can-Bus Variables
byte DME4_Load0 = 0x00;
byte DME4_Load3 = 0x00;
byte DME4_Load5 = 0x00;
byte DME6_Load1 = 0x00;

// SimHub Variables
int sh_speed = 0;
int sh_rpm = 0;
int sh_temp = 0;
int sh_fuel_percent = 0;
int sh_oil_pressure = 0;
uint8_t sh_abs_active = 0;
uint8_t sh_abs_mode = 0;
uint8_t sh_tcs_active = 0;
uint8_t sh_tcs_mode = 0;
String sh_blinker_left;
String sh_blinker_right;
String sh_cruise_control;
String sh_ebrake;
String sh_oil_warn;
String sh_lights;
String sh_highbeam = "False";
String sh_backlight = "False";

String sh_game = "AssetoCorsa";

class SHCustomProtocol {
    private:

    public:
		// Called when starting the arduino (setup method in main sketch)
		void setup() {
			pinMode(pin_abs, OUTPUT);
			pinMode(pin_ebrake, OUTPUT);
			pinMode(pin_backlight, OUTPUT);
			pinMode(pin_speedo, OUTPUT);

			digitalWrite(pin_abs, LOW);
			digitalWrite(pin_ebrake, LOW);
			digitalWrite(pin_backlight, LOW);

			kbus.begin(9600);

			CAN.begin(CAN_500KBPS); // CANBus : Baudrate = 500k

		}

		// Called when new data is coming from computer
		void read() {
			sh_speed = FlowSerialReadStringUntil(';').toInt();
			sh_rpm = FlowSerialReadStringUntil(';').toInt();
			sh_temp = FlowSerialReadStringUntil(';').toInt();
			sh_oil_pressure = FlowSerialReadStringUntil(';').toInt();
			sh_cruise_control = FlowSerialReadStringUntil(';');
			sh_ebrake = FlowSerialReadStringUntil(';');
			sh_oil_warn = FlowSerialReadStringUntil(';');
			sh_lights = FlowSerialReadStringUntil(';');
			sh_abs_active = FlowSerialReadStringUntil(';').toInt();
			sh_abs_mode = FlowSerialReadStringUntil(';').toInt();
			sh_tcs_active = FlowSerialReadStringUntil(';').toInt();
			sh_tcs_mode = FlowSerialReadStringUntil(';').toInt();
			sh_highbeam = FlowSerialReadStringUntil(';');
			sh_blinker_left = FlowSerialReadStringUntil(';');
			sh_blinker_right = FlowSerialReadStringUntil(';');
			sh_backlight = FlowSerialReadStringUntil(';');
			sh_game = FlowSerialReadStringUntil('\n');
		}

		void loop() {

			// Backlight
			if (sh_backlight.equalsIgnoreCase("True") || sh_lights.equalsIgnoreCase("True")) {
				digitalWrite(pin_backlight, HIGH);
			} else {
				digitalWrite(pin_backlight, LOW);
			}

			// Traction Control
			if ((sh_tcs_active == 0) && (sh_tcs_mode == 0)) {
				DME6_Load1 = 0x00;
			} else {
				DME6_Load1 = 0x01;
			}

			// ABS
			if (sh_abs_mode == 0 || sh_abs_active == 1) {
				digitalWrite(pin_abs, HIGH);
			} else{
				digitalWrite(pin_abs, LOW);
			}

			// Speedometer
			write_speedometer();

			// RPM data with maximum enforcement
			if (sh_rpm > cluster_max_rpm) {
				sh_rpm = cluster_max_rpm;
			}
			output_rpm = map(sh_rpm, 0, 7000, 0, 175.00f);

			// RPM
			// L2 = sh_rpm LSB
			// L3 = sh_rpm MSB from 0.00f to 175.00f
			// unsigned char message1[8] = {0x05, 0x62, 0xFF, output_rpm, 0x65, 0x12, 0, 62};
			// CAN.sendMsgBuf(0x316, 0, 8, message1);
			send_canbus(0x316, 0x05, 0x62, 0xFF, output_rpm, 0x65, 0x12, 0, 62);
			
			// DME 4
			// DME4 Load 0: 0x10=EML, 0x08=Cruise Control, 0x02=Motor Light
			// DME4 Load 1: ? Probably L/100km -> Maybe the Change of Rate is needed to calculate in the cluster
			// DME4 Load 3: 0x08=Overheat Light, 0x02=Oillight
			// DME4 Load 4: ? Probably Oiltemp -> not as analog in the cluster, so ignored for now
			// DME4 Load 5: Charging Light 0x00=off, 0x01=on
			send_canbus(0x545, DME4_Load0, 0x01, 0x18, DME4_Load3, 0x00, DME4_Load5, 0x00, 0x00);

			// Coolant Temperature
			// Calculation, maxTemp=260.00f, minTemp=50.00f
			// sh_temp Scaling to Float based on temperaturerange from 50 to 130 Degree Celsius
			// Map engine temp and keep within bounds
			if (sh_temp >= 130) {
				sh_temp = 125;
			}
			output_temp = map(sh_temp, 50, 130, 50.00f, 260.00f);
			send_canbus(0x329, 0x00, output_temp, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

			// Seems to clear the (!) and Traction control light.
			// L1 = 0x01=Traction Control
			send_canbus(0x153, 0x00, DME6_Load1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

			// Time
			send_canbus(0x613, 0x00, 0x00, 0x10, hour, minute, 0x00, 0x00, 0x00);

			byte kbus_message[] = {0xD0, 0x08, 0xBF, 0x5B, kbus_lightbyte_1, 0x00, 0x00, kbus_lightbyte_2, 0x00, 0x58, 0x00};
			send_kbus(kbus_message);

		}

		// Called once between each byte read on arduino, dont put any time consuming stuff in here.
		void idle() {}

		void send_canbus(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h)
		{
			unsigned char DataToSend[8] = {a, b, c, d, e, f, g, h};
			CAN.sendMsgBuf(address, 0, 8, DataToSend);
		};

		void send_kbus(byte *data)
		{
			int end_i = data[1] + 2 ;
			data[end_i - 1] = iso_checksum(data, end_i - 1);
			kbus.write(data, end_i + 1);
		}

		void write_speedometer()
		{
			if (sh_speed == 0){
				noTone(pin_speedo);
			} else {
				output_speed = map(sh_speed, 0, 250, 0, 1680);
				tone(pin_speedo, output_speed); // 250KmH=1680, 0KmH=0
			}
		}

		byte iso_checksum(byte *data, byte len) //len is the number of bytes (not the # of last byte)
		{
			byte crc = 0;

			for (byte i = 0; i < len; i++){
				crc = crc ^ data[i];
			}

			return crc;
		}

};

#endif
