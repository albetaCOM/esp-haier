/**
* Create by Miguel Ángel López on 20/07/19
* Modified by Alba Prades on 21/07/20
* Modified by Alba Prades on 13/08/20 
* Modified by Alba Prades on 25/08/20: Added fan, dry and swing features
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
	
#define HORIZONTAL_SWING_OFFSET		19
	#define HORIZONTAL_SWING_CENTER 		0x00
	#define HORIZONTAL_SWING_MAX_LEFT 		0x03
	#define HORIZONTAL_SWING_LEFT 			0x04
	#define HORIZONTAL_SWING_MAX_RIGHT 		0x06
	#define HORIZONTAL_SWING_RIGHT 			0x05
	#define HORIZONTAL_SWING_AUTO 			0x07
	
#define VERTICAL_SWING_OFFSET			13
	#define VERTICAL_SWING_MAX_UP			0x02
	#define VERTICAL_SWING_UP				0x04
	#define VERTICAL_SWING_CENTER				0x06
	#define VERTICAL_SWING_DOWN				0x08
	#define VERTICAL_SWING_HEALTH_UP			0x01
	#define VERTICAL_SWING_HEALTH_DOWN		0x03
	#define VERTICAL_SWING_AUTO 				0x0C

#define TEMPERATURE_OFFSET   	22

#define STATUS_DATA_OFFSET			17 // Purify/Quiet mode/OnOff/...
	#define POWER_BIT				(0)	
	#define PURIFY_BIT				(1)	
	#define QUIET_BIT				(3)	
	#define AUTO_FAN_MAX_BIT		(4)
	
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
#define CTRL_POWER_OFFSET		13
	#define CTRL_POWER_ON		0x01
	#define CTRL_POWER_OFF		0x00
	
	
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
	byte control_command[25] = {0xFF,0xFF,0x14,0x40,0x00,0x00,0x00,0x00,0x00,0x01,0x60,0x01,0x09,0x08,0x25,0x00,0x02,0x00,0x00,0x06,0x00,0x00,0x03,0x0B,0x70};

	byte climate_mode_fan_speed = FAN_AUTO;
	byte climate_mode_setpoint = 0x0A;
	
	byte fan_mode_fan_speed = FAN_HIGH;
	byte fan_mode_setpoint = 0x08;
	
	bool first_status_received = false;
	
	// Some vars for debuging purposes	
	byte previous_status[47];
	bool previous_status_init = false;
	
	
	// Functions

	void SetHvacModeControl(byte mode)
	{
		control_command[MODE_OFFSET] &= ~MODE_MSK;
		control_command[MODE_OFFSET] |= mode;
	}	
	
	byte GetHvacModeStatus()
	{
		return status[MODE_OFFSET] & MODE_MSK;
	}	
	
	void SetTemperatureSetpointControl(byte temp)
	{
		control_command[SET_POINT_OFFSET] = temp;
	}	
	
	byte GetTemperatureSetpointStatus()
	{
		return status[SET_POINT_OFFSET];
	}	
	
	void SetFanSpeedControl(byte fan_mode)
	{
		control_command[MODE_OFFSET] &= ~FAN_MSK;
		control_command[MODE_OFFSET] |= fan_mode;
	}
	
	byte GetFanSpeedStatus()
	{
		return status[MODE_OFFSET] & FAN_MSK;
	}		
	
	void SetHorizontalSwingControl(byte swing_mode)
	{
		control_command[HORIZONTAL_SWING_OFFSET] = swing_mode;
	}
	
	byte GetHorizontalSwingStatus()
	{
		return status[HORIZONTAL_SWING_OFFSET];
	}	
	
	void SetVerticalSwingControl(byte swing_mode)
	{
		control_command[VERTICAL_SWING_OFFSET] = swing_mode;
	}
	
	byte GetVerticalSwingStatus()
	{
		return status[VERTICAL_SWING_OFFSET];
	}	
	
	void SetQuietModeControl(bool quiet_mode)
	{
		byte tmp;
		byte msk;
		
		msk = (0x01 << QUIET_BIT);		
		
		if(quiet_mode == true){
			control_command[STATUS_DATA_OFFSET] |= msk;
		}
		else{
			msk = ~msk;
			control_command[STATUS_DATA_OFFSET] &= msk;
		}
	}
	
	bool GetQuietModeStatus( void )
	{
		bool ret = false;		
		byte tmp;
		byte msk;
		
		msk = (0x01 << QUIET_BIT);
		tmp = status[STATUS_DATA_OFFSET] & msk;
		
		if(tmp != 0) ret = true;
		
		return ret;
	}
	
	void SetPurifyControl(bool purify_mode)
	{
		byte tmp;
		byte msk;
		
		msk = (0x01 << PURIFY_BIT);		
		
		if(purify_mode == true){
			control_command[STATUS_DATA_OFFSET] |= msk;
		}
		else{
			msk = ~msk;
			control_command[STATUS_DATA_OFFSET] &= msk;
		}
	}
	
	bool GetPurifyStatus( void )
	{
		bool ret = false;		
		byte tmp;
		byte msk;
		
		msk = (0x01 << PURIFY_BIT);
		tmp = status[STATUS_DATA_OFFSET] & msk;
		
		if(tmp != 0) ret = true;
		
		return ret;
	}
	
	void SetPowerControl(bool power_mode)
	{

		byte tmp;
		byte msk;
		
		msk = (0x01 << POWER_BIT);		
		
		if(power_mode == true){
			control_command[STATUS_DATA_OFFSET] |= msk;
		}
		else{
			msk = ~msk;
			control_command[STATUS_DATA_OFFSET] &= msk;
		}
	}
	
	bool GetPowerStatus( void )
	{
		bool ret = false;		
		byte tmp;
		byte msk;
		
		msk = (0x01 << POWER_BIT);
		tmp = status[STATUS_DATA_OFFSET] & msk;
		
		if(tmp != 0) ret = true;
		
		return ret;
	}
	
	
	bool GetFastModeStatus( void )
	{
		bool ret = false;		
		byte tmp;
		byte msk;
		
		msk = (0x01 << AUTO_FAN_MAX_BIT);
		tmp = status[STATUS_DATA_OFFSET] & msk;
		
		if(tmp != 0) ret = true;
		
		return ret;
	}
	
	void SetFastModeControl(bool fast_mode)
	{
		byte tmp;
		byte msk;
		
		msk = (0x01 << AUTO_FAN_MAX_BIT);		
		
		if(fast_mode == true){
			control_command[STATUS_DATA_OFFSET] |= msk;
		}
		else{
			msk = ~msk;
			control_command[STATUS_DATA_OFFSET] &= msk;
		}
	}
	
	
	void CompareStatusByte()
	{
		int i;
		
		if(previous_status_init == false){
			for (i=0;i<sizeof(status);i++){
				previous_status[i] = status[i];
			}
			previous_status_init = true;
		}
		
		for (i=0;i<sizeof(status);i++)
		{
			if(status[i] != previous_status[i]){
				ESP_LOGD("Debug", "Status byte %d: 0x%X --> 0x%X ", i, previous_status[i],status[i]);
			}
			previous_status[i] = status[i];
		}
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
		traits.set_supported_modes({climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_DRY, climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_OFF});

		traits.set_supported_fan_modes({climate::CLIMATE_FAN_ON, climate::CLIMATE_FAN_OFF, climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_MIDDLE, climate::CLIMATE_FAN_HIGH});

        traits.set_visual_min_temperature(MIN_SET_TEMPERATURE);
        traits.set_visual_max_temperature(MAX_SET_TEMPERATURE);
        traits.set_visual_temperature_step(1.0f);
        traits.set_supports_current_temperature(true);

		traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_BOTH, climate::CLIMATE_SWING_VERTICAL, climate::CLIMATE_SWING_HORIZONTAL});
        return traits;
    }

public:

    void parseStatus() {


        auto raw = getHex(status, sizeof(status));
        ESP_LOGD("Haier", "Readed message ALBA: %s ", raw.c_str());

        byte check = getChecksum(status, sizeof(status));

        if (check != status[CRC_OFFSET(status)]) {
            ESP_LOGW("Haier", "Invalid checksum (%d vs %d)", check, status[CRC_OFFSET(status)]);
            return;
        }

        lastCRC = check;

        current_temperature = status[TEMPERATURE_OFFSET]/2;
        target_temperature = status[SET_POINT_OFFSET] + 16;

        if(current_temperature < MIN_VALID_INTERNAL_TEMP || current_temperature > MAX_VALID_INTERNAL_TEMP 
            || target_temperature < MIN_SET_TEMPERATURE || target_temperature > MAX_SET_TEMPERATURE){
            ESP_LOGW("Haier", "Invalid temperatures");
            return;
        }
		
		// Read all the info from the status message and update values in control message
		// so the next message is updated
		// This is usefull if there are manual changes with the remote control
		SetHvacModeControl(GetHvacModeStatus());
		
		// All flags on STATUS_DATA_OFFSET just toggle the corresponding status, like a push button
		//SetPowerControl(GetPowerStatus());
		//SetPurifyControl(GetPurifyStatus());
		//SetQuietModeControl(GetQuietModeStatus());
		//SetFastModeControl(GetFastModeStatus());
		
		SetFanSpeedControl(GetFanSpeedStatus());
		SetHorizontalSwingControl(GetHorizontalSwingStatus());
		SetVerticalSwingControl(GetVerticalSwingStatus());
		SetTemperatureSetpointControl(GetTemperatureSetpointStatus());
		
		if(GetHvacModeStatus() == MODE_FAN){
			fan_mode_fan_speed = GetFanSpeedStatus();
			fan_mode_setpoint = GetTemperatureSetpointStatus();
		}
		else{
			climate_mode_fan_speed = GetFanSpeedStatus();
			climate_mode_setpoint = GetTemperatureSetpointStatus();
		}
		
		// Flag to enable modifications from UI as we now know the status of the A/C
		first_status_received = true;

		
		// DEBUG DATA, uncomment what's needed
		//ESP_LOGW("Debug", "HVAC Mode = 0x%X", GetHvacModeStatus());

		ESP_LOGW("Debug", "Power Status = 0x%X", GetPowerStatus());
		ESP_LOGW("Debug", "Purify status = 0x%X", GetPurifyStatus());
		ESP_LOGW("Debug", "Quiet mode Status = 0x%X", GetQuietModeStatus());
		ESP_LOGW("Debug", "Fast mode Status = 0x%X", GetFastModeStatus());

		//ESP_LOGW("Debug", "Fan speed Status = 0x%X", GetFanSpeedStatus());
		//ESP_LOGW("Debug", "Horizontal Swing Status = 0x%X", GetHorizontalSwingStatus());
		//ESP_LOGW("Debug", "Vertical Swing Status = 0x%X", GetVerticalSwingStatus());
		//ESP_LOGW("Debug", "Set Point Status = 0x%X", GetTemperatureSetpointStatus());
		CompareStatusByte();
		
		
		// Update home assistant component
		
        if (GetPowerStatus() == false) {
            mode = CLIMATE_MODE_OFF;
		} else {
			// Check current hvac mode
            switch (GetHvacModeStatus()) {
                case MODE_COOL:
                    mode = CLIMATE_MODE_COOL;
                    break;
                case MODE_HEAT:
                    mode = CLIMATE_MODE_HEAT;
                    break;
                case MODE_DRY:
				    mode = CLIMATE_MODE_DRY;
					break;
				case MODE_FAN:
                    mode = CLIMATE_MODE_FAN_ONLY;
                    break;
                case MODE_AUTO:
                default:
                    mode = CLIMATE_MODE_HEAT_COOL;
            }
					
			// Get fan speed
			// If "quiet mode" is set we will read it as "fan low"
			if ( GetQuietModeStatus() == true) {
                fan_mode = CLIMATE_FAN_LOW;
            }
			// If we detect that fast mode is on the we read it as "fan high"
			else if( GetFastModeStatus() == true) {
				fan_mode = CLIMATE_FAN_HIGH;
			}			
			else {				
				// No quiet or fast so we read the actual fan speed.
                switch (GetFanSpeedStatus()) {
                    case FAN_AUTO:
                        fan_mode = CLIMATE_FAN_AUTO;
                        break;
                    case FAN_MID:
                        fan_mode = CLIMATE_FAN_MEDIUM;
                        break;
                    //case FAN_MIDDLE:
                    //    fan_mode = CLIMATE_FAN_MIDDLE;
                    //    break;
					case FAN_LOW:
						fan_mode = CLIMATE_FAN_LOW;
                        break;
                    case FAN_HIGH:
                        fan_mode = CLIMATE_FAN_HIGH;
                        break;
                    default:
                        fan_mode = CLIMATE_FAN_AUTO;
						
                }
            }				


			// Check the status of the swings (vertical and horizontal and translate according component configuration
			if( (GetHorizontalSwingStatus() == HORIZONTAL_SWING_AUTO) && (GetVerticalSwingStatus() == VERTICAL_SWING_AUTO) ){
				swing_mode = CLIMATE_SWING_BOTH;				
			}
			else if(GetHorizontalSwingStatus() == HORIZONTAL_SWING_AUTO){
				swing_mode = CLIMATE_SWING_HORIZONTAL;
			}
			else if(GetVerticalSwingStatus() == VERTICAL_SWING_AUTO){
				swing_mode = CLIMATE_SWING_VERTICAL;
			}
			else{
				swing_mode = CLIMATE_SWING_OFF;
			}
		}

        this->publish_state();

    }


    void control(const ClimateCall &call) override {
        ClimateMode new_mode;
		bool new_control_cmd = false;
		
		
		ESP_LOGD("Control", "Control call");
		
		if(first_status_received == false){
			ESP_LOGD("Control", "No action, first poll answer not received");
			return;
		}

        if (call.get_mode().has_value()) {
            // User requested mode change
            new_mode = *call.get_mode();
        
			ESP_LOGD("Control", "*call.get_mode() = %d", new_mode);
			
			// It seems that this message is no needed, we keep it here commented
						
			//if((new_mode != CLIMATE_MODE_OFF) && (GetPowerStatus() == false)){
				// if the current mode is off -> we need to power on
			//	sendData(power_command, sizeof(power_command));  
			//	delay(1000);	
			//}
			
            switch (new_mode) {
                case CLIMATE_MODE_OFF:
					SetPowerControl(false);
					sendData(control_command, sizeof(control_command)); 
                    break;
					
                case CLIMATE_MODE_HEAT_COOL:
					SetPowerControl(true);
					SetHvacModeControl(MODE_AUTO);
					
					// Recover fan_speed and setpoint (when switching to fan_only they are "lost")
					SetFanSpeedControl(climate_mode_fan_speed);
					SetTemperatureSetpointControl(climate_mode_setpoint);	
						
					sendData(control_command, sizeof(control_command));
                    break;
					
                case CLIMATE_MODE_HEAT:	
					SetPowerControl(true);
					SetHvacModeControl(MODE_HEAT);
					
					// Recover fan_speed and setpoint (when switching to fan_only they are "lost")
					SetFanSpeedControl(climate_mode_fan_speed);
					SetTemperatureSetpointControl(climate_mode_setpoint);	
					
					sendData(control_command, sizeof(control_command));
                    break;
					
                case CLIMATE_MODE_DRY:		
					SetPowerControl(true);
					SetHvacModeControl(MODE_DRY);
					
					// Recover fan_speed and setpoint (when switching to fan_only they are "lost")
					SetFanSpeedControl(climate_mode_fan_speed);
					SetTemperatureSetpointControl(climate_mode_setpoint);	
					
					sendData(control_command, sizeof(control_command));
                    break;
					
                case CLIMATE_MODE_FAN_ONLY:				
					SetPowerControl(true);
					SetHvacModeControl(MODE_FAN);				
					
					// Recover fan_speed and setpoint (fan_only values are "special")
					SetFanSpeedControl(fan_mode_fan_speed);
					SetTemperatureSetpointControl(fan_mode_setpoint);	
					
					sendData(control_command, sizeof(control_command));
                    break;

                case CLIMATE_MODE_COOL:
					SetPowerControl(true);
					SetHvacModeControl(MODE_COOL);
					
					// Recover fan_speed and setpoint (when switching to fan_only they are "lost")
					SetFanSpeedControl(climate_mode_fan_speed);
					SetTemperatureSetpointControl(climate_mode_setpoint);
					
					sendData(control_command, sizeof(control_command));
                    break;
					
				case CLIMATE_MODE_AUTO:
				default:
					break;
					
            }
		
            // Publish updated state
            mode = new_mode;
            this->publish_state();
		}
		
		        //Set fan speed
        if (call.get_fan_mode().has_value()) {
            switch(call.get_fan_mode().value()) {
                case CLIMATE_FAN_LOW:
					SetFanSpeedControl(FAN_LOW);
                    break;
                case CLIMATE_FAN_MIDDLE:
					SetFanSpeedControl(FAN_MID);
                    break;
                case CLIMATE_FAN_MEDIUM:
					SetFanSpeedControl(FAN_MID);
                    break;
                case CLIMATE_FAN_HIGH:
					SetFanSpeedControl(FAN_HIGH);
                    break;
                case CLIMATE_FAN_AUTO:
                    SetFanSpeedControl(FAN_AUTO);
                    break;
				case CLIMATE_FAN_ON:
				case CLIMATE_FAN_OFF:
				case CLIMATE_FAN_FOCUS:
				case CLIMATE_FAN_DIFFUSE:
				default:
				    break;
                    
			}
			sendData(control_command, sizeof(control_command)); 
		}

        //Set swing mode
        if (call.get_swing_mode().has_value()){
            switch(call.get_swing_mode().value()) {
                case CLIMATE_SWING_OFF:
					// When not auto we decide to set it to the center
					SetHorizontalSwingControl(HORIZONTAL_SWING_CENTER);
					// When not auto we decide to set it to the center
					SetVerticalSwingControl(VERTICAL_SWING_CENTER);
                    break;
                case CLIMATE_SWING_VERTICAL:
					// When not auto we decide to set it to the center
                    SetHorizontalSwingControl(HORIZONTAL_SWING_CENTER);
					SetVerticalSwingControl(VERTICAL_SWING_AUTO);
                    break;
                case CLIMATE_SWING_HORIZONTAL:
                    SetHorizontalSwingControl(HORIZONTAL_SWING_AUTO);
					// When not auto we decide to set it to the center
					SetVerticalSwingControl(VERTICAL_SWING_CENTER);
                    break;
                case CLIMATE_SWING_BOTH:
                    SetHorizontalSwingControl(HORIZONTAL_SWING_AUTO);
					SetVerticalSwingControl(VERTICAL_SWING_AUTO);
                    break;
			}
			sendData(control_command, sizeof(control_command)); 
        }
		
		
		if (call.get_target_temperature().has_value()) {
		    float temp = *call.get_target_temperature();
			ESP_LOGD("Control", "*call.get_target_temperature() = %f", temp);
			control_command[SET_POINT_OFFSET] = (uint16) temp - 16;
			sendData(control_command, sizeof(control_command));			
			target_temperature = temp;
            this->publish_state();
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