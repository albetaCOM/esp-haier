/**
* Create by Miguel Ángel López on 20/07/19
* Modified by Alba Prades on 21/07/20
* Modified by Alba Prades on 13/08/20 
*      Added modes
**/

#ifndef HAIER_ESP_HAIER_H
#define HAIER_ESP_HAIER_H

#include "esphome.h"
#include <string>

using namespace esphome;
using namespace esphome::climate;

// Updated read offset

#define MODE_OFFSET 			14
#define MODE_MSK				0xF0
	#define MODE_AUTO       	0x00
	#define MODE_DRY			0x40
	#define MODE_COOL			0x20
	#define MODE_HEAT			0x80
	#define MODE_FAN			0xC0
#define FAN_MSK					0x0F
	#define FAN_LOW	    		0x03
	#define FAN_MID		  		0x02
	#define FAN_HIGH	     	0x01
	#define FAN_AUTO	   		0x05

#define TEMPERATURE_OFFSET   	22

#define POWER_OFFSET       		17
#define POWER_MSK				0x01

#define PURIFY_OFFSET			17
#define PURIFY_MSK				0x02
	
#define SET_POINT_OFFSET 		12	

// Another byte
	#define SWING        		27
	#define SWING_OFF          	0
	#define SWING_VERTICAL     	1
	#define SWING_HORIZONTAL   	2
	#define SWING_BOTH

	#define LOCK        		28
	#define LOCK_ON     		80
	#define LOCK_OFF    		00

// Updated read offset



#define FRESH       			31
	#define FRESH_ON    		1
	#define FRESH_OFF   		0

// Updated read offset


#define COMMAND_OFFSET			9
	#define RESPONSE_POLL		2

#define CRC_OFFSET(message)		(2 + message[2])

// Control commands
#define CTR_POWER_OFFSET		13
	#define CTR_POWER_ON		0x01
	#define CTR_POWER_OFF		0x00
	
#define POLY 0xa001


// temperatures supported by AC system
#define MIN_SET_TEMPERATURE 16
#define MAX_SET_TEMPERATURE 30

//if internal temperature is outside of those boundaries, message will be discarded
#define MIN_VALID_INTERNAL_TEMP 10
#define MAX_VALID_INTERNAL_TEMP 50

class Haier : public Climate, public PollingComponent {

private:

    byte lastCRC;
    byte status[47];
	
	byte initialization_1[13] = {0xFF,0xFF,0x0A,0x0,0x0,0x0,0x0,0x0,0x00,0x61,0x00,0x07,0x72};
	byte initialization_2[13] = {0xFF,0xFF,0x08,0x40,0x0,0x0,0x0,0x0,0x0,0x70,0xB8,0x86,0x41};
 	byte poll[15] = {0xFF,0xFF,0x0A,0x40,0x00,0x00,0x00,0x00,0x00,0x01,0x4D,0x01,0x99,0xB3,0xB4};
    byte power_command[17]     = {0xFF,0xFF,0x0C,0x40,0x00,0x00,0x00,0x00,0x00,0x01,0x5D,0x01,0x00,0x01,0xAC,0xBD,0xFB};
	byte control_command[25] = {0xFF,0xFF,0x14,0x40,0x00,0x00,0x00,0x00,0x00,0x01,0x60,0x01,0x09,0x08,0x25,0x00,0x02,0x03,0x00,0x06,0x00,0x0C,0x03,0x0B,0x70};


	void SetMode(byte mode)
	{
		control_command[MODE_OFFSET] &= ~MODE_MSK;
		control_command[MODE_OFFSET] |= mode;
	}	
	
	void SetFan(byte fan_mode)
	{
		control_command[MODE_OFFSET] &= ~FAN_MSK;
		control_command[MODE_OFFSET] |= fan_mode;
	}


public:

    Haier() : PollingComponent(5 * 1000) {
        lastCRC = 0;
    }


    
    void setup() override {
        
        Serial.begin(9600);
		delay(1000);
		Serial.write(initialization_1, sizeof(initialization_1));
        auto raw = getHex(initialization_1, sizeof(initialization_1));
        ESP_LOGD("Haier", "initialization_1: %s ", raw.c_str());
		delay(1000);
		Serial.write(initialization_2, sizeof(initialization_2));
        raw = getHex(initialization_2, sizeof(initialization_2));
        ESP_LOGD("Haier", "initialization_2: %s ", raw.c_str());
    }

    void loop() override  {
		byte data[47];
        if (Serial.available() > 0) {
			if (Serial.read() != 255) return;
			if (Serial.read() != 255) return;
			
			data[0] = 255;
			data[1] = 255;

            Serial.readBytes(data+2, sizeof(data)-2);
			
			// If is a status response
			if (data[COMMAND_OFFSET] == RESPONSE_POLL) {
				// Update the status frame
				memcpy(status, data, sizeof(status));
				parseStatus();
			}
		}
    }

    void update() override {
            
        Serial.write(poll, sizeof(poll));
        auto raw = getHex(poll, sizeof(poll));
        ESP_LOGD("Haier", "POLL: %s ", raw.c_str());
    }

protected:
    ClimateTraits traits() override {
        auto traits = climate::ClimateTraits();
        traits.set_supports_away(false);
        traits.set_supports_auto_mode(true);
        traits.set_supports_heat_mode(true);
        traits.set_supports_cool_mode(true);
        traits.set_visual_min_temperature(MIN_SET_TEMPERATURE);
        traits.set_visual_max_temperature(MAX_SET_TEMPERATURE);
        traits.set_visual_temperature_step(1.0f);
        traits.set_supports_current_temperature(true);
        //traits.set_supports_action(true);// Cal identificar el byte
        return traits;
    }

public:

    void parseStatus() {


        auto raw = getHex(status, sizeof(status));
        ESP_LOGD("Haier", "Readed message ALBA: %s ", raw.c_str());

        byte check = getChecksum(status, sizeof(status));

        if (check != status[CRC_OFFSET(status)]) {
            ESP_LOGW("Haier", "Invalid checksum (%d vs %d)", check, status[CRC_OFFSET(status)]);
            //return;
        }


        lastCRC = check;

        current_temperature = status[TEMPERATURE_OFFSET]/2;
        target_temperature = status[SET_POINT_OFFSET] + 16;

        if(current_temperature < MIN_VALID_INTERNAL_TEMP || current_temperature > MAX_VALID_INTERNAL_TEMP 
            || target_temperature < MIN_SET_TEMPERATURE || target_temperature > MAX_SET_TEMPERATURE){
            ESP_LOGW("Haier", "Invalid temperatures");
            return;
        }


        if (!(status[POWER_OFFSET]&POWER_MSK)) {
            mode = CLIMATE_MODE_OFF;

        } else {
			ESP_LOGW("Haier", "Current mode = %X", status[MODE_OFFSET] & MODE_MSK);
            switch (status[MODE_OFFSET] & MODE_MSK) {
                case MODE_COOL:
                    mode = CLIMATE_MODE_COOL;
                    break;
                case MODE_HEAT:
                    mode = CLIMATE_MODE_HEAT;
                    break;
                case MODE_AUTO:
                case MODE_DRY:
				case MODE_FAN:
                default:
                    mode = CLIMATE_MODE_AUTO;
            }


        }

        this->publish_state();

    }


    void control(const ClimateCall &call) override {
        ClimateMode new_mode;
		bool new_control_cmd = false;
		
		
		ESP_LOGD("Control", "Control call");

        if (call.get_mode().has_value()) {
            // User requested mode change
            new_mode = *call.get_mode();
        
			ESP_LOGD("Control", "*call.get_mode() = %d", new_mode);
			//ESP_LOGD("Control", "abans: 0x%X ", control_command[MODE_OFFSET]);
			
			if((new_mode != CLIMATE_MODE_OFF) && (!(status[POWER_OFFSET]&POWER_MSK))){
				ESP_LOGD("Control", "abans: 0x%X ", control_command[MODE_OFFSET]);

				// if the current mode is off -> we need to power on
				sendData(power_command, sizeof(power_command));  
				delay(1000);	
			}
			
			
			// PURIFY ON
			//control_command[PURIFY_OFFSET] |= PURIFY_MSK;
			// PURIFY OFF
			//control_command[PURIFY_OFFSET] &= ~PURIFY_MSK;
			
			
			
            switch (new_mode) {
                case CLIMATE_MODE_OFF:
					if(status[POWER_OFFSET]&POWER_MSK){				
						power_command[CTR_POWER_OFFSET] = CTR_POWER_OFF;
						sendData(power_command, sizeof(power_command)); 
					}
                    break;
                case CLIMATE_MODE_AUTO:
					if((status[MODE_OFFSET] & MODE_MSK) != MODE_DRY){
						power_command[CTR_POWER_OFFSET] = CTR_POWER_ON;
						SetMode(MODE_FAN);
						SetFan(FAN_HIGH);
						control_command[SET_POINT_OFFSET] = status[SET_POINT_OFFSET];
						new_control_cmd = true;
						//sendData(control_command, sizeof(control_command)); 
					}
                    break;
                case CLIMATE_MODE_HEAT:
					if((status[MODE_OFFSET] & MODE_MSK) != MODE_HEAT){
						power_command[CTR_POWER_OFFSET] = CTR_POWER_ON;				
						SetMode(MODE_HEAT);
						SetFan(FAN_AUTO);
						control_command[SET_POINT_OFFSET] = status[SET_POINT_OFFSET];
						new_control_cmd = true;
						//sendData(control_command, sizeof(control_command));
					}
					 
                    break;
                case CLIMATE_MODE_COOL:
					if((status[MODE_OFFSET] & MODE_MSK) != MODE_COOL){
						power_command[CTR_POWER_OFFSET] = CTR_POWER_ON;
						SetMode(MODE_COOL);
						SetFan(FAN_AUTO);
						control_command[SET_POINT_OFFSET] = status[SET_POINT_OFFSET];
						new_control_cmd = true;
						//sendData(control_command, sizeof(control_command)); 
					}
                    break;
            }
			//ESP_LOGD("Haier", "despres: 0x%X ", control_command[MODE_OFFSET]);
            // Publish updated state
            mode = new_mode;
            this->publish_state();
		}
		
		if (call.get_target_temperature().has_value()) {
		    float temp = *call.get_target_temperature();
			ESP_LOGD("Control", "*call.get_target_temperature() = %f", temp);
			control_command[SET_POINT_OFFSET] = (uint16) temp - 16;
			//sendData(control_command, sizeof(control_command));
			new_control_cmd = true;
			
			target_temperature = temp;
            this->publish_state();
		}
		
		if(new_control_cmd == true){
			ESP_LOGD("Haier", "Enviando nueva INFO ");
			sendData(control_command, sizeof(control_command));
			new_control_cmd = false;
		}
   }


    void sendData(byte * message, byte size) {
        byte crc_offset = CRC_OFFSET(message);
        byte crc = getChecksum(message, size);
        word crc_16 = crc16(0, &(message[2]), crc_offset-2);
        
        // Updates the crc
        message[crc_offset] = crc;
        message[crc_offset+1] = (crc_16>>8)&0xFF;
        message[crc_offset+2] = crc_16&0xFF;

        Serial.write(message, size);

        auto raw = getHex(message, size);
        ESP_LOGD("Haier", "Message sent: %s  - CRC: %X - CRC16: %X", raw.c_str(), crc, crc_16);

    }

    String getHex(byte * message, byte size) {

		
        String raw;

        for (int i=0; i < size; i++){
			raw += " " + String(message[i]);

        }
        raw.toUpperCase();

        return raw;


    }

    byte getChecksum(const byte * message, size_t size) {
        byte position = CRC_OFFSET(message);
        byte crc = 0;
        
        if (size < ( position)) {
        	ESP_LOGE("Control", "frame format error (size = %d vs length = %d)", size, message[2]);
        	return 0;
        }

        for (int i = 2; i < position; i++)
            crc += message[i];

        return crc;
    }


    unsigned crc16(unsigned crc, unsigned char *buf, size_t len)
    {
        while (len--) {
            crc ^= *buf++;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        }
        return crc;
    }


};


#endif //HAIER_ESP_HAIER_H