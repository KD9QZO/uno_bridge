/* 
 * SDRuno Bridge plugin mocking firmware v0.1
 */

#include "remote_config.h"
#include "remote.h"

#include <TimerOne.h>


/* --- PCF8575 configuration --- */

#include "lib/PCF8575.cpp"

PCF8575 PCF_1(PCF8575_ADDR);

volatile bool PCF_1_FLAG = false;
const int PCF_1_IRQ_PIN = PIN_PCF1_IRQ;
uint16_t pcf_1_status = 65535;		// all pins high


/* --- TFT_HX8357 configuration --- */

#include "lib/TFT_HX8357.cpp"

TFT_HX8357 tft = TFT_HX8357();


/* --- Rotary Library configuration --- */

#if (defined(INCLUDE_ROTARY_LIB))
#include "lib/rotary.cpp"
Rotary r = Rotary(4, 3);
#endif

/* pinChangeInerrupts routine; used this because of exhausted hardware interrupts */
#include "PinChangeInterrupt.h"





/* --- Rotary encoder routine --- */

/* ------------- *
 *   Encoder-A   *
 * ------------- */

int encoderAPinA = 66;			// A12
int encoderAPinB = 67;			// A13
uint8_t encoderAPinALast = LOW;

/* ------------- *
 *   Encoder-B   *
 * ------------- */

int encoderBPinA = 68;			// A14
int encoderBPinB = 69;			// A15
uint8_t encoderBPinALast = LOW;

/* ------------- *
 *   Encoder-M   *
 * ------------- */

int encoderMPinA = 65;
int encoderMPinB = 64;
uint8_t encoderMPinALast = LOW;




/**
 * \brief Holds the global state of the controller
 */
struct __attribute__((__packed__)) State {
	uint8_t Demodulator = 0;								/*!< Current demodulator */
	wfm_mode_t WfmDeemphasisMode = DeemphasisDefault;		/*!< Wideband FM de-emphasis mode, of type \ref wfm_mode_t */
	nb_mode_t NoiseBlankerMode = NoiseBlankerDefault;		/*!< Noise Blanker mode, of type \ref nb_mode_t */
	agc_mode_t AgcMode = AGCModeDefault;					/*!< AGC mode, of type \ref agc_mode_t */
	int8_t AgcThreshold = 0;
	int8_t NoiseBlankerLevel = 0;
	int8_t NoiseReductionLevel = 0;
	int8_t CwPeakFilterThreshold = 0;
	int AudioVolume = 0;									/*!< Audio output volume level */
	int8_t SP1MinPower = 0;
	uint32_t VfoFrequency = 146520000;						/*!< VFO frequency, in \b Hz */
	uint32_t CenterFrequency = 446000000;					/*!< Center frequency, in \b Hz */
	uint32_t SP1MinFrequency = 0;
	uint32_t SP1MaxFrequency = 0;
	long int MPXLevel = 0;
	int FilterBandwidth = 4300;								/*!< Filter bandwidth, in \b Hz */
	int SquelchLevel = 0;									/*!< Squelch level */
	int SquelchEnable = 0;									/*!< Squelch enable; \b boolean value */
	int FmNoiseReductionEnable = 0;							/*!< FM noise reduction enable; \b boolean value */
	int AudioMute = 0;										/*!< Audio output mute; \b boolean value */
	int BiasTEnable = 0;									/*!< Antenna Bias-Tee enable; \b boolean value */
	/* --- non-transferable params --- */
	uint16_t Step = 16;										// trailing four bits are reserved for aux and does not mapped to step place highlighter
	bool VfoMode = false;									/*!< VFO mode */
	bool Att = false;										/*!< Attenuator mode */
	bool Preamp = false;									/*!< Pre-amplifier mode */
	uint8_t Band = 0;										/*!< Index of current band */
	long int fingerprint;									/*!< hash-like substitute. @todo rework me later */
} state;




state_flags_t state_flags;

#if 0
//@todo: add dynamic state change flag system or just spread a bunch of bool flags
bool state_changed = true;
bool update_display_all = true;
bool update_display_sql = false;
bool update_display_mode = false;
bool update_display_vol = false;
bool update_display_band = false;
bool update_display_att = false;
bool update_display_preamp = false;
#endif

// LED display params
// pin 11 of 74HC595 (SHCP)
const int bit_clock_pin = PIN_74HC595_SCLK;		// SCLK

// pin 12 of 74HC595 (STCP)
const int digit_clock_pin = PIN_74HC595_LOAD;		// LOAD/latch

// pin 14 of 74HC595 (DS)
const int data_pin = PIN_74HC595_SDI;				// SDI pin


// digit pattern for a 7-segment display
const byte digit_pattern[16] = {
	B00111111,  // 0
	B00000110,  // 1
	B01011011,  // 2
	B01001111,  // 3
	B01100110,  // 4
	B01101101,  // 5
	B01111101,  // 6
	B00000111,  // 7
	B01111111,  // 8
	B01101111,  // 9
	B01110111,  // A
	B01111100,  // b
	B00111001,  // C
	B01011110,  // d
	B01111001,  // E
	B01110001   // F
};

/*! \brief temporary substitute for checksum */
long lastRandomSent;

unsigned long timepassed = millis();

unsigned long passed_10 = 0;

void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	Serial.setTimeout(10);						/* 10 ms */
	
	State state;
	
	/* --- Initialize state flags --- */
	state_flags.st_changed = true;
	state_flags.upd_disp_all = false;
	state_flags.upd_disp_sql = false;
	state_flags.upd_disp_mode = false;
	state_flags.upd_disp_vol = false;
	state_flags.upd_disp_band = false;
	state_flags.upd_disp_att = false;
	state_flags.upd_disp_preamp = false;
	
	/* --- LED pins setup --- */
	pinMode(data_pin, OUTPUT);
	pinMode(bit_clock_pin, OUTPUT);
	pinMode(digit_clock_pin, OUTPUT);
	
	/* --- Timer interrupt for display freq --- */
	Timer1.initialize(10000);					/* µs timing value */
	Timer1.attachInterrupt(fillRegisters_isr);
	
	/* --- PCF_1 interrupt for keyboard and buttons --- */
	PCF_1.begin();
	pinMode(PCF_1_IRQ_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PCF_1_IRQ_PIN), pcf_isr, FALLING);

#if 0
	pinMode(encoderAPinA, INPUT_PULLUP);	/* Encoder-A (SQL) */
	pinMode(encoderAPinB, INPUT_PULLUP);	/* Encoder-A (SQL) */
	pinMode(encoderBPinA, INPUT_PULLUP);	/* Encoder-B (Vol) */
	pinMode(encoderBPinB, INPUT_PULLUP);	/* Encoder-B (Vol) */
#endif

	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encoderAPinA), encoder_a_isr, CHANGE);
	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encoderBPinA), encoder_b_isr, CHANGE);
	pinMode(encoderMPinA, INPUT_PULLUP);	/* Main Encoder */
	pinMode(encoderMPinB, INPUT_PULLUP);	/* Main Encoder */
	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encoderMPinA), encoder_m_isr, CHANGE);
	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encoderMPinB), encoder_m_isr, CHANGE);
	
	/* --- TFT init --- */
	tft.init();
	tft.setRotation(1);
	tft.fillScreen(0x0000);
}

void loop() {
	state.fingerprint = random();
	
	if (state_flags.st_changed) {
		echoStruct();
		state_flags.st_changed = false;
	}
	
	if (state_flags.upd_disp_all) {
		updateDisplayAll();
		state_flags.upd_disp_all = false;
	}
	
	if (state_flags.upd_disp_sql) {
		printSQL();
		state_flags.upd_disp_sql = false;
	}
	
	if (state_flags.upd_disp_mode) {
		printMode();
		state_flags.upd_disp_mode = false;
	}
	
	if (state_flags.upd_disp_vol) {
		printVol();
		state_flags.upd_disp_vol = false;
	}
	
	if (state_flags.upd_disp_band) {
		printBand();
		state_flags.upd_disp_band = false;
	}
	
	if (state_flags.upd_disp_att) {
		printAtt();
		state_flags.upd_disp_att = false;  
	}
	
	if (state_flags.upd_disp_preamp) {
		printPreamp();
		state_flags.upd_disp_preamp = false;
	}
	
	if (PCF_1_FLAG) {
		processPCF();
	}
	
	lastRandomSent = state.fingerprint;
	
	String str = Serial.readStringUntil('\n');
	
	if (str.length() > 0) {
		parseStruct(str);
	}

#if 0
	delay(100);
#endif
}

void echoStruct() {
	Serial.print(state.Demodulator);
	Serial.print("\t");
	Serial.print(state.WfmDeemphasisMode);
	Serial.print("\t");
	Serial.print(state.NoiseBlankerMode);
	Serial.print("\t");
	Serial.print(state.AgcMode);
	Serial.print("\t");
	Serial.print(state.AgcThreshold);
	Serial.print("\t");
	Serial.print(state.NoiseBlankerLevel);
	Serial.print("\t");
	Serial.print(state.NoiseReductionLevel);
	Serial.print("\t");
	Serial.print(state.CwPeakFilterThreshold);
	Serial.print("\t");
	Serial.print(state.AudioVolume);
	Serial.print("\t");
	Serial.print(state.SP1MinPower);
	Serial.print("\t");
	Serial.print(state.VfoFrequency);
	Serial.print("\t");
	Serial.print(state.CenterFrequency);
	Serial.print("\t");
	Serial.print(state.SP1MinFrequency);
	Serial.print("\t");
	Serial.print(state.SP1MaxFrequency);
	Serial.print("\t");
	Serial.print(state.MPXLevel);
	Serial.print("\t");
	Serial.print(state.FilterBandwidth);
	Serial.print("\t");
	Serial.print(state.SquelchLevel);
	Serial.print("\t");
	Serial.print(state.SquelchEnable);
	Serial.print("\t");
	Serial.print(state.FmNoiseReductionEnable);
	Serial.print("\t");
	Serial.print(state.AudioMute);
	Serial.print("\t");
	Serial.print(state.BiasTEnable);
	Serial.print("\t");
	Serial.print(state.fingerprint);
	Serial.println("");
}

void parseStruct(String string) {
	const char *str = string.c_str();

	sscanf(str, "%lu %lu %lu %lu", &state.VfoFrequency, &state.CenterFrequency, &state.SP1MinFrequency, &state.SP1MaxFrequency);
	state_flags.upd_disp_all = true;
}

// interrupt service routine by Timer1 interrupt signal
void fillRegisters_isr() {
	if (state_flags.st_changed || state_flags.upd_disp_all) {
		int array[12];
		int temporaryStep = state.Step;
		uint32_t number = state.VfoFrequency;
		
		digitalWrite(digit_clock_pin, LOW);

#if 0
		// highlight mute button
		if (state.AudioMute) {
			bitSet(temporaryStep, 3);
		} else {
			bitClear(temporaryStep, 3);
		}
#endif

		// highlight bias-T button
		if (state.BiasTEnable) {
			bitSet(temporaryStep, 2);
		} else {
			bitClear(temporaryStep, 2);
		}

		// highlight VFO Mode button
		if (state.VfoMode) {
			bitSet(temporaryStep, 1);
		} else {
			bitClear(temporaryStep, 1);
		}
		
		// highlight active step place and buttons
		shiftOut(data_pin, bit_clock_pin, LSBFIRST, temporaryStep);
		shiftOut(data_pin, bit_clock_pin, LSBFIRST, (temporaryStep >> 8));
		
		for (int i = 12; i > 0; i--) {
			byte pattern = digit_pattern[number % 10];
			
			shiftOut(data_pin, bit_clock_pin, MSBFIRST, ~(pattern));
			array[i] = number % 10;
			number /= 10;
		}
		
		digitalWrite(digit_clock_pin, HIGH);
		state_flags.upd_disp_band = true;
	}
}

// interrupt service routine by PCF interrupt signal
void pcf_isr() {
	PCF_1_FLAG = true;
}

// process PCF_1 fetch and modify state accordingly
void processPCF() {
	PCF_1_FLAG = false;
	pcf_1_status = PCF_1.read16();

#if 0
	// process Audio Mute button toggle //Red!
	if (!bitRead(pcf_1_status, 0)) {
		state.AudioMute = !state.AudioMute;
		state_flags.upd_disp_all = true;
	}
#endif

	// process Bias-T button toggle
	if (!bitRead(pcf_1_status, 1)) {
		state.BiasTEnable = !state.BiasTEnable;
	}
	
	// process VfoMode button toggle
	if (!bitRead(pcf_1_status, 14)) {
		state.VfoMode = !state.VfoMode;
		state_flags.upd_disp_mode = true;
	}
	
	// process StepUp button
	if (!bitRead(pcf_1_status, 15)) {
		stepUp();
	}
	
	// process StepDown button
	if (!bitRead(pcf_1_status, 7)) {
		stepDown();
	}
	
	// process SQL On/Off button
	if (!bitRead(pcf_1_status, 8)) {
		state.SquelchEnable = !state.SquelchEnable;
		state_flags.upd_disp_sql = true;
	}
	
	// process Mute On/Off button
	if (!bitRead(pcf_1_status, 11)) {
		state.AudioMute = !state.AudioMute;
		state_flags.upd_disp_vol = true;
	}
	
	// process Attenuator Toggle button
	if (!bitRead(pcf_1_status, 6)) {
		state.Att = !state.Att;
		state_flags.upd_disp_att = true;
	}
	
	// process Pre-Amp Toggle button
	if (!bitRead(pcf_1_status, 5)) {
		state.Preamp = !state.Preamp;
		state_flags.upd_disp_preamp = true;
	}
	
	state_flags.st_changed = true;
}


/**
 * \brief ISR to process squelch encoder increment/decrement
 */
void encoder_a_isr() {
	uint8_t n = digitalRead(encoderAPinA);

	if ((encoderAPinALast == LOW) && (n == HIGH) && state.SquelchEnable) {
		if (digitalRead(encoderAPinB) == LOW) {
			SQLUp();
		} else {
			SQLDown();
		}
		state_flags.st_changed = true; // Changed the value
		state_flags.upd_disp_sql = true;
	}
	encoderAPinALast = n;
}


/**
 * \brief ISR to process volume encoder increment/decrement
 */
void encoder_b_isr() {
	uint8_t n = digitalRead(encoderBPinA);

	if ((encoderBPinALast == LOW) && (n == HIGH) && !state.AudioMute) {
		if (digitalRead(encoderBPinB) == LOW) {
			volDown();
		} else {
			volUp();
		}
		state_flags.st_changed = true; // Changed the value
		state_flags.upd_disp_vol = true;
	}
	encoderBPinALast = n;
}


/**
 * \brief ISR to process main encoder
 */
void encoder_m_isr() {
	uint8_t n = digitalRead(encoderMPinA);

	if ((encoderMPinALast == HIGH) && (n == LOW)) {
		if (digitalRead(encoderMPinB) == HIGH) {
			mainEncInc();
		} else {
			mainEncDec();
		}
		state_flags.st_changed = true; // Changed the value
	}
	encoderMPinALast = n;
}


void updateDisplayAll() {
	printSQL();
	printMode();
	printVol();
	printBand();
	printAtt();
	printPreamp();
}

void printMode() {
	char buffer[50];

	tft.setTextSize(2);
	tft.setTextFont(3);

	sprintf(buffer, "%s", state.VfoMode ? "VFO" : "Memory");
	tft.setTextColor(TFT_WHITE);
	tft.drawRoundRect(399, 0, 80, 34, 5, TFT_WHITE);	// X, Y, W, H, radius, Color
	tft.fillRoundRect(400, 1, 78, 32, 5, TFT_BLACK);	// black mask
	tft.drawString(buffer, 410, 0, 2);					// string, X, Y, FontNumber
}

void printBand(){
	char buffer[50];

	tft.setTextSize(2);
	tft.setTextFont(3);
	tft.setTextColor(TFT_WHITE);  
	tft.drawRoundRect(139, 0, 80, 34, 5, TFT_WHITE);	// X, Y, W, H, radius, Color
	tft.fillRoundRect(140, 1, 78, 32, 5, TFT_BLACK);	// black mask
	for (int i = 0; i < BAND_DEFINITIONS_QTY; i++) {	// size of bands array
		if ((bands[i].band_lower <= state.CenterFrequency) && (state.CenterFrequency <= bands[i].band_upper)) {
//			Serial.print("Band \t");
//			Serial.println(bands[i].band_name);
			sprintf(buffer, "%s", bands[i].band_name.c_str());
		}
	}
	tft.drawString(buffer, 150, 0, 2);					// string, X, Y, FontNumber
}

void printSQL() {
	char buffer[50];
	uint16_t color;
	
	tft.setTextSize(2);
	tft.setTextFont(3);
	
	if (state.SquelchEnable) {
		sprintf(buffer, "SQL: %d", state.SquelchLevel);
		color = TFT_WHITE;
	} else {
		sprintf(buffer, "SQL: OFF");
		color = TFT_RED;
	}

	tft.drawRoundRect(0, 0, 130, 34, 5, color);
	tft.fillRoundRect(1, 1, 128, 32, 5, TFT_BLACK);  // Black mask
	tft.setTextColor(color);
	tft.drawString(buffer, 6, 0, 2); //string, X, Y, Font number
}

void printVol() {
  tft.setTextSize(2);
  tft.setTextFont(3);
  char buffer[50];
  uint16_t color;

  if (!state.AudioMute) {
    sprintf(buffer, "Vol: %d", state.AudioVolume);
    color = TFT_WHITE;
  } else {
    sprintf(buffer, "Vol: OFF");
    color = TFT_RED;
  }

  tft.drawRoundRect(0, 38, 130, 34, 5, color);
  tft.fillRoundRect(1, 39, 128, 32, 5, TFT_BLACK);
  tft.setTextColor(color);
  tft.drawString(buffer, 6, 38, 2);
}

void printAtt() {
  tft.setTextSize(2);
  tft.setTextFont(3);
  char buffer[10];
  uint16_t color;
  sprintf(buffer, "Att");

  if (!state.Att) {
    color = TFT_WHITE;
  } else {
    color = TFT_RED;
  }

  tft.drawRoundRect(0, 286, 70, 34, 5, color);
  tft.fillRoundRect(1, 287, 68, 32, 5, TFT_BLACK);
  tft.setTextColor(color);
  tft.drawString(buffer, 16, 286, 2);
}

void printPreamp() {
	char buffer[10];
	uint16_t color;

	sprintf(buffer, "Preamp");

	tft.setTextSize(2);
	tft.setTextFont(3);

	if (!state.Preamp) {
		color = TFT_WHITE;
	} else {
		color = TFT_RED;
	}
	
	tft.drawRoundRect(76, 286, 120, 34, 5, color);
	tft.fillRoundRect(77, 287, 118, 32, 5, TFT_BLACK);
	tft.setTextColor(color);
	tft.drawString(buffer, 92, 286, 2);
}


// move place highlighter one step upward
void stepUp() {
#if 0
	state.Step = (state.Step << 1);
#else
	state.Step <<= 1;
#endif

	if (state.Step == 0) {
		state.Step = 16;	// 4 bits reserved for buttons
	}
}

// move place highlighter one step downward
void stepDown() {
#if 0
	state.Step = (state.Step >> 1);
#else
	state.Step >>= 1;
#endif

	if (state.Step < 16) {
		state.Step = 32768;	// 4 bits reserved for buttons
	}
}

void SQLUp() {
	if (state.SquelchLevel < 220) /* max value that affects SQL bar */ {
		state.SquelchLevel++;
	}
}

void SQLDown() {
	if (state.SquelchLevel > 0) {
		state.SquelchLevel--;
	}
}

void volUp() {
	if (state.AudioVolume < 220) /* max value that affects Volume bar */ {
		state.AudioVolume++;
	}
}

void volDown() {
	if (state.AudioVolume > 0) {
		state.AudioVolume--;
	}
}

void mainEncInc() {
	uint32_t factor = castStep();
	
	if (state.VfoFrequency + factor <= 3000000000) {
		state.VfoFrequency += factor;
	} else {
		state.VfoFrequency = 3000000000;
	}
}

void mainEncDec() {
	uint32_t factor = castStep();
	if ((state.VfoFrequency - factor > 0) && (state.VfoFrequency - factor < 3000000000)) {
		state.VfoFrequency -= factor;
	}
}

long int castStep() {
	unsigned long long int factor = 0;

	switch (state.Step) {
		case 16:
			factor = 1;
			break;
	
		case 32:
			factor = 10;
			break;
	
		case 64:
			factor = 100;
			break;
	
		case 128:
			factor = 1000;
			break;
	
		case 256:
			factor = 10000;
			break;
	
		case 512:
			factor = 100000;
			break;
	
		case 1024:
			factor = 1000000;
			break;
			
		case 2048:
			factor = 10000000;
			break;

		case 4096:
			factor = 100000000;
			break;

		case 8192:
			factor = 1000000000;
			break;

		case 16384:
			factor = 10000000000;
			break;

		case 32768:
			factor = 100000000000;
			break;

		default:
			break;
	}
	
	return (factor);
}


//--------------service helper methods--------------

//print value in binary-way.
void print_binary(int v, int num_places) {
	int mask = 0;
	int n;

	for (n = 1; n <= num_places; n++) {
		mask = ((mask << 1) | 0x0001);
	}
	v = (v & mask);  // truncate v to specified number of places

	while (num_places) {
		if ((v & (0x0001 << num_places - 1))) {
			Serial.print("1");
		} else {
			Serial.print("0");
		}

		--num_places;
		if (((num_places % 4) == 0) && (num_places != 0)) {
			Serial.print("_");
		}
	}
}
