#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread> 
#include <cmath>
#include "AltitudeTransfer.h"

using namespace std;
using namespace std::chrono;

enum PlaneState {
    GROUND = 0,
    CHANGING_ALTITUDE = 1,
    CRUISE_FLIGHT = 2,
    STALL = 3
};

class Plane {
public:
    Plane(milliseconds time_step, float engine_power = 10, PlaneState state = GROUND)
		: time_step(time_step), engine_power(engine_power), plane_state(state) {}

	void dump() {
		cout << fixed << "current status: " << plane_state_strings[plane_state] << ", altitude: " << altitude << "m" << endl;
		cout << fixed << "speed: " << get_current_speed() << "m/min, rate of climb: " << vertical_speed << "m/min, horizontal speed: " << horizontal_speed << "m/min" << endl;
		cout << fixed << "engine_power: " << engine_power << "%" << endl;

		current_transfer.dump();

		cout << endl;
	}

	void tick();

	int get_altitude() { return round(altitude); }

	void set_altitude_transfer(AltitudeTransfer transfer) {
		current_transfer = transfer;	
	}

	AltitudeTransfer get_current_transfer() {
		return current_transfer;
	}

private:
	const float ACCELERATION_PER_PERCENT_OF_ENGINE_POWER = 10.0f; // m/min

	milliseconds time_step;

	AltitudeTransfer current_transfer;

    // outputs
    float engine_power;
    PlaneState plane_state;

    float vertical_speed = 0;
	float horizontal_speed = 0;
    float altitude = 0;

	float get_acceleration() {
		return (engine_power / 100.0f) * ACCELERATION_PER_PERCENT_OF_ENGINE_POWER;
	}

	float get_current_speed() {
		return sqrt(pow(vertical_speed, 2) + pow(horizontal_speed, 2));
	}

	void update_altitude();
};


void Plane::tick() {
	auto altitude = get_altitude();

	switch(current_transfer.get_state()) {
		case TRANSFER_START: {
			current_transfer.next_state();
			break;
		}

		case TRANSFER_SPEED_TRANSFER_START: {
			plane_state = CHANGING_ALTITUDE;
			horizontal_speed = current_transfer.get_desired_climb_rate() / tan(current_transfer.get_desired_angle());
			vertical_speed = current_transfer.get_desired_climb_rate();
			current_transfer.next_state();
			break;
		}

		case TRANSFER_STEADY: {
			auto type = current_transfer.get_type();
			if(type == ASCENDING && altitude >= current_transfer.get_desired_altitude()) {
				current_transfer.next_state();
			} else if(type == DESCENDING && altitude <= current_transfer.get_desired_altitude()) {
				current_transfer.next_state();
			} else {
				break;
			}
		}

		case TRANSFER_SPEED_TRANSFER_END: {
			vertical_speed = 0;
			if(altitude == 0) {
				horizontal_speed = 0;
			} else {
				horizontal_speed = 800;
			}

			current_transfer.next_state();
		}

		case TRANSFER_DONE: {
			if(altitude == 0) plane_state = GROUND;
			else plane_state = CRUISE_FLIGHT;
			break;
		}	
	}

	update_altitude();
}

void Plane::update_altitude() {
	altitude += (float) time_step.count() / duration_cast<milliseconds>(1min).count() * vertical_speed;
}

int main() {
	auto time_step = 1ms;
	Plane plane(time_step);

	cout << "initializing plane" << endl;
	plane.dump();

	auto altitude_transfer = AltitudeTransfer(plane.get_altitude(), 200/*m*/, 10/*deg*/, 500/*m/min*/);
	plane.set_altitude_transfer(altitude_transfer);

	uint64_t number_of_ticks = 0;
	while(true) {
		plane.tick();

		if(number_of_ticks % 1000 == 0) {
			auto elapsed_time = number_of_ticks * time_step;
			cout << "[" << elapsed_time.count() << "ms]" << endl;
			plane.dump();
		}

		this_thread::sleep_for(1us);
		number_of_ticks++;

		if(plane.get_current_transfer().is_done() ) {
			plane.set_altitude_transfer(AltitudeTransfer(
				plane.get_altitude(), 0, 10, 500			
			));
		}
	}

    return EXIT_SUCCESS;
}

