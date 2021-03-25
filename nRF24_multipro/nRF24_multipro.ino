#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

// ##########################################
// #####   MultiProtocol nRF24L01 Tx   ######
// ##########################################
// #        by goebish on rcgroups          #
// #                                        #
// #   Parts of this project are derived    #
// #     from existing work, thanks to:     #
// #                                        #
// #   - PhracturedBlue for DeviationTX     #
// #   - victzh for XN297 emulation layer   #
// #   - Hasi for Arduino PPM decoder       #
// #   - hexfet, midelic, closedsink ...    #
// #   - SamGold for analog input           #
// ##########################################
//
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License.
// If not, see <http://www.gnu.org/licenses/>.


// ############ Wiring ################
#define PPM_pin   2  // PPM in
//SPI Comm.pins with nRF24L01
#define MOSI_pin    11  // MOSI
#define SCK_pin     13  // SCK
#define CE_pin      7  // CE
#define MISO_pin    12 // MISO
#define CS_pin      8 // CS

#define ledPin      4 // LED

// SPI outputs
#define MOSI_on PORTB |= _BV(3)
#define MOSI_off PORTB &= ~_BV(3)
#define SCK_on PORTB |= _BV(5)
#define SCK_off PORTB &= ~_BV(5)
#define CE_on PORTD |= _BV(7)
#define CE_off PORTD &= ~_BV(7)
#define CS_on PORTB |= _BV(0)
#define CS_off PORTB &= ~_BV(0)
// SPI input
#define  MISO_on (PINB & _BV(4))

#define RF_POWER TX_POWER_80mW 

// tune ppm input for "special" transmitters
// #define SPEKTRUM // TAER, 1100-1900, AIL & RUD reversed

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7), emergency stop (Bayang, Silverware)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)
#define ANALOG_CHANNELS 10
#define ANALOG_THRESH 20

// supported protocols
enum {
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,           // EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
    PROTO_SYMAXOLD,     // Syma X5C, X2
    PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
    PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    PROTO_FQ777124,     // FQ777-124 pocket drone
    PROTO_E010,         // EAchine E010, NiHui NH-010, JJRC H36 mini
    PROTO_BAYANG_SILVERWARE, // Bayang for Silverware with frsky telemetry
    PROTO_END
};

// EEPROM locationss
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

struct {
    uint16_t volt1;
    uint16_t rssi;
    uint8_t updated;
    uint32_t lastUpdate;
} telemetry_data;

typedef struct{
  uint8_t  input;
  uint16_t min_val;
  uint16_t max_val;
  bool     button;
  uint16_t bind_to;
} analog_channel_t;

uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};


//A0 ELEVATOR        MIN 0 MAX 719 CENTER 363
//A1 AILERON         MIN 0 MAX 719 CENTER 358
//A2 THROTTLE        MIN 0 MAX 719 CENTER 361
//A3 RUDDER          MIN 0 MAX 714 CENTER 350

//A4 0
//   DOWN_LEFT_BTN1 652
//   DOWN_LEFT_BTN2 357

//A5 RELEASED 0
//   DOWN_RIGHT_BTN1 652
//   DOWN_RIGHT_BTN2 357

//A6 FRONT_RIGHT_BTN RELEASED 720 PRESSED 0
//A7 FRONT_LEFT_BTN  RELEASED 720 PRESSED 0

analog_channel_t a_channels[ANALOG_CHANNELS] = {
  {.input = 0, .min_val = 0,   .max_val = 719, .button = false, .bind_to = ELEVATOR},
  {.input = 1, .min_val = 719, .max_val = 0,   .button = false, .bind_to = AILERON},
  {.input = 2, .min_val = 719, .max_val = 0,   .button = false, .bind_to = THROTTLE},
  {.input = 3, .min_val = 0,   .max_val =714,  .button = false, .bind_to = RUDDER},
  {.input = 4, .min_val = 0, .max_val = 652, .button = true, .bind_to = AUX3},
  {.input = 4, .min_val = 0, .max_val = 357, .button = true, .bind_to = AUX4},
  {.input = 5, .min_val = 0, .max_val = 652, .button = true, .bind_to = AUX5},
  {.input = 5, .min_val = 0, .max_val = 357, .button = true, .bind_to = AUX8},
  {.input = 6, .min_val = 720, .max_val = 0, .button = true, .bind_to = AUX1},
  {.input = 7, .min_val = 720, .max_val = 0, .button = true, .bind_to = AUX2}
};

void setup()
{
    Serial.begin(115200);
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(PPM_pin, INPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    set_txid(false);
}

void loop()
{
    uint32_t timeout=0;
    // reset / rebind
    
    if(reset || ppm[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        selectProtocol();
        if (NRF24L01_Reset())
          Serial.println("NRF24L01 OK");
        else
          Serial.println("NRF24L01 FAILED");

        NRF24L01_Initialize();
        init_protocol();
    }
    telemetry_data.updated = 0;
    // process protocol
    switch(current_protocol) {
        case PROTO_CG023:
        case PROTO_YD829:
            timeout = process_CG023();
            break;
        case PROTO_V2X2:
            timeout = process_V2x2();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            timeout = process_CX10();
            break;
        case PROTO_H7:
            timeout = process_H7();
            break;
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            timeout = process_Bayang();
            break;
        case PROTO_SYMAX5C1:
        case PROTO_SYMAXOLD:
            timeout = process_SymaX();
            break;
        case PROTO_H8_3D:
            timeout = process_H8_3D();
            break;
        case PROTO_MJX:
        case PROTO_E010:
            timeout = process_MJX();
            break;
        case PROTO_HISKY:
            timeout = process_HiSky();
            break;
        case PROTO_KN:
            timeout = process_KN();
            break;
        case PROTO_YD717:
            timeout = process_YD717();
            break;
        case PROTO_FQ777124:
            timeout = process_FQ777124();
            break;
    }
    // updates ppm values out of ISR
    update_ppm();
    
    while(micros() < timeout) {
        if(telemetry_data.updated) {
            frskyUpdate();
        }            
    }
    telemetry_data.updated = 0;
}

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

void selectProtocol()
{
    update_ppm();

    Serial.println("selectProtocol");
    Serial.print("ppm[RUDDER]:");
    Serial.println(ppm[RUDDER]);
    Serial.print("ppm[AILERON]:");
    Serial.println(ppm[AILERON]);
    Serial.print("ppm[ELEVATOR]:");
    Serial.println(ppm[ELEVATOR]);
    Serial.print("ppm[THROTTLE]:");
    Serial.println(ppm[THROTTLE]);
    
    // startup stick commands (protocol selection / renew transmitter ID)
    
    if(ppm[RUDDER] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND) // rudder left + aileron left
        current_protocol = PROTO_BAYANG_SILVERWARE; // Bayang protocol for Silverware with frsky telemetry
        
    else if(ppm[RUDDER] < PPM_MIN_COMMAND)   // Rudder left
        set_txid(true);                      // Renew Transmitter ID
    
    // Rudder right + Aileron right + Elevator down
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_E010; // EAchine E010, NiHui NH-010, JJRC H36 mini
    
    // Rudder right + Aileron right + Elevator up
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_FQ777124; // FQ-777-124

    // Rudder right + Aileron left + Elevator up
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD717; // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    
    // Rudder right + Aileron left + Elevator down
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_KN; // KN (WLToys variant) V930/931/939/966/977/988
    
    // Rudder right + Elevator down
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_HISKY; // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    
    // Rudder right + Elevator up
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_SYMAXOLD; // Syma X5C, X2 ...
    
    // Rudder right + Aileron right
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_MJX; // MJX X600, other sub protocols can be set in code
    
    // Rudder right + Aileron left
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_H8_3D; // H8 mini 3D, H20 ...
    
    // Elevator down + Aileron right
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD829; // YD-829, YD-829C, YD-822 ...
    
    // Elevator down + Aileron left
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_SYMAX5C1; // Syma X5C-1, X11, X11C, X12
    
    // Elevator up + Aileron right
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_BAYANG;    // EAchine H8(C) mini, BayangToys X6/X7/X9, JJRC JJ850 ...
    
    // Elevator up + Aileron left
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND) 
        current_protocol = PROTO_H7;        // EAchine H7, MT99xx
    
    // Elevator up  
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_V2X2;       // WLToys V202/252/272, JXD 385/388, JJRC H6C ...
        
    // Elevator down
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND) 
        current_protocol = PROTO_CG023;      // EAchine CG023/CG031/3D X4, (todo :ATTOP YD-836/YD-836C) ...
    
    // Aileron right
    else if(ppm[AILERON] > PPM_MAX_COMMAND)  
        current_protocol = PROTO_CX10_BLUE;  // Cheerson CX10(blue pcb, newer red pcb)/CX10-A/CX11/CX12 ... 
    
    // Aileron left
    else if(ppm[AILERON] < PPM_MIN_COMMAND)  
        current_protocol = PROTO_CX10_GREEN;  // Cheerson CX10(green pcb)... 
    
    // read last used protocol from eeprom
    else 
        current_protocol = constrain(EEPROM.read(ee_PROTOCOL_ID),0,PROTO_END-1);      
    // update eeprom 
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    // wait for safe throttle
    while(ppm[THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        update_ppm();
    }
}

void init_protocol()
{
    switch(current_protocol) {
        case PROTO_CG023:
        case PROTO_YD829:
            Serial.println("Init PROTO_CG023 PROTO_YD829");
            CG023_init();
            CG023_bind();
            break;
        case PROTO_V2X2:
            Serial.println("Init PROTO_V2X2");
            V2x2_init();
            V2x2_bind();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            Serial.println("Init PROTO_CX10_GREEN PROTO_CX10_BLUE");
            CX10_init();
            CX10_bind();
            break;
        case PROTO_H7:
            Serial.println("Init PROTO_H7");
            H7_init();
            H7_bind();
            break;
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            Serial.println("Init PROTO_BAYANG PROTO_BAYANG_SILVERWARE");
            Bayang_init();
            Bayang_bind();
            break;
        case PROTO_SYMAX5C1:
        case PROTO_SYMAXOLD:
            Serial.println("Init PROTO_SYMAX5C1 PROTO_SYMAXOLD");
            Symax_init();
            break;
        case PROTO_H8_3D:
            Serial.println("Init PROTO_H8_3D");
            H8_3D_init();
            H8_3D_bind();
            break;
        case PROTO_MJX:
        case PROTO_E010:
            Serial.println("Init PROTO_MJX PROTO_E010");
            MJX_init();
            MJX_bind();
            break;
        case PROTO_HISKY:
            Serial.println("Init PROTO_HISK");
            HiSky_init();
            break;
        case PROTO_KN:
            Serial.println("Init PROTO_KN");
            kn_start_tx(true); // autobind
            break;
        case PROTO_YD717:
            Serial.println("Init PROTO_YD717");
            YD717_init();
            break;
        case PROTO_FQ777124:
            Serial.println("Init PROTO_FQ777124");
            FQ777124_init();
            FQ777124_bind();
            break;
    }
}

// update ppm values out of analog inputs    
void update_ppm()
{
    for(uint8_t ch=0; ch<ANALOG_CHANNELS; ch++) {
        int a_value;
        
        a_value = analogRead(a_channels[ch].input);

        if (a_channels[ch].button) {
          int comp_value;
          
          comp_value = ((int) a_channels[ch].max_val) - ((int) a_channels[ch].min_val);
          
          if (abs(comp_value - a_value) < ANALOG_THRESH)
            ppm[a_channels[ch].bind_to] = PPM_MAX;
          else
            ppm[a_channels[ch].bind_to] = PPM_MIN;
        } else {
            ppm[a_channels[ch].bind_to] = constrain(map(a_value,a_channels[ch].min_val,a_channels[ch].max_val,PPM_MIN,PPM_MAX),PPM_MIN,PPM_MAX);
        }
    }
}
