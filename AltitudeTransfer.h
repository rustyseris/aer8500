#ifndef ALTITUDE_TRANSFER_H
#define ALTITUDE_TRANSFER_H

#include <iostream>
#include <string>

static const std::string plane_state_strings[] = {
	"GROUND", "CHANGING_ALTITUDE", "CRUISE_FLIGHT", "STALL"
};

enum AltitudeTransferType {
	ASCENDING,
	DESCENDING,
	NO_TRANSFER
};

enum AltitudeTransferState {
	TRANSFER_START,
	TRANSFER_SPEED_TRANSFER_START,
	TRANSFER_SPEED_TRANSFER_END,
	TRANSFER_STEADY,
	TRANSFER_DONE
};

class AltitudeTransfer {
public:
	AltitudeTransfer() = default;
	AltitudeTransfer(
			int current_altitude,
			int altitude,
			float angle,
			float climb_rate)
		: desired_altitude(altitude),
		  desired_angle(angle)
	{
		state = TRANSFER_SPEED_TRANSFER_START;	
		if(current_altitude > desired_altitude) {
			desired_climb_rate = - climb_rate;
			type = DESCENDING;
		} else if(current_altitude < desired_altitude) {
			desired_climb_rate = climb_rate;
			type = ASCENDING;
		} else {
			type = NO_TRANSFER;
			state = TRANSFER_DONE;
		}
	}

	int get_desired_altitude() { return desired_altitude; }
	float get_desired_angle() { return desired_angle; }
	float get_desired_climb_rate() { return desired_climb_rate; }	

	bool is_done() {
		return get_state() == TRANSFER_DONE;
	}

	bool next_state() {
		bool result = true;
		switch(state) {
			case TRANSFER_START:
				state = TRANSFER_SPEED_TRANSFER_START;
				break;

			case TRANSFER_SPEED_TRANSFER_START:
				state = TRANSFER_STEADY;
				break;

			case TRANSFER_STEADY:
				state = TRANSFER_SPEED_TRANSFER_END;
				break;

			case TRANSFER_SPEED_TRANSFER_END:
				state = TRANSFER_DONE;
				break;

			case TRANSFER_DONE:
				result = false;
		}

		return result;
	}

	AltitudeTransferState get_state() {
		return state;	
	}

	AltitudeTransferType get_type() {
		return type;
	}

	void dump() {
		if(type == NO_TRANSFER) {
			std::cout << "no altitude transfer" << std::endl;
			return;
		}

		std::cout << std::fixed << "desired_altitude: " << desired_altitude << "meters, desired_angle: " << desired_angle << "deg, desired_climb_rate: " << desired_climb_rate << "m/min" << std::endl;
	}

private:
	int desired_altitude = 0;
	float desired_angle = 0;
	float desired_climb_rate = 0;

	AltitudeTransferType type = NO_TRANSFER;
	AltitudeTransferState state = TRANSFER_DONE;
};


#endif // ALTITUDE_TRANSFER_H

