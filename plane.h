#ifndef PLANE_H
#define PLANE_H

#include <stdexcept>
#include <iostream>
#include <chrono>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <cassert>

using namespace std;
using namespace chrono;

#define M1_IN_FT 3.28084f
#define FT1_IN_M 0.3048f
#define METER_TO_FOOT(___meters) (___meters * M1_IN_FT)
#define FOOT_TO_METER(___feets) (___feets * FT1_IN_M)
#define DEG_TO_RAD(___deg) (___deg * M_PI / 180.0f)
#define RAD_TO_DEG(___rad) (___rad * 180.0f / M_PI)
#define ENGINE_POWER_PER_PERCENT 10.0f
#define US_IN_1MIN (1000 * 1000 * 60)


struct AltitudeTransfer {
	typedef enum {
		TRANSFER_ASCENDING,
		TRANSFER_DESCENDING,
		TRANSFER_NONE
	} Type;

	typedef enum {
		TRANSFER_START,
		TRANSFER_SPEED_TRANSFER_BEGINNING,
		TRANSFER_STEADY,
		TRANSFER_SPEED_TRANSFER_ENDING,
		TRANSFER_FINISHED
	} State;

	State state = TRANSFER_FINISHED;
	Type type = TRANSFER_NONE;
	double altitude = 0;   // meters
	double angle = 0;      // rad, [-15, 15]
	double climb_rate = 0; // m/min, always positive, max value dependent on the angle

	AltitudeTransfer() = default;
	AltitudeTransfer(Type type, double altitude, double angle, double climb_rate)
		: state(TRANSFER_START), type(type), altitude(altitude), angle(angle), climb_rate(climb_rate) {}

	void dump_status()
	{
		cout << "transfer state: " << transfer_state_str[state]; 
		if(state != TRANSFER_FINISHED) {
			auto altitude_ft = METER_TO_FOOT(altitude);
			auto angle_deg = RAD_TO_DEG(angle);
			cout << ", type: " << transfer_type_str[type] << endl;
			cout << fixed << "altitude: " << altitude_ft << "ft (" << altitude <<  "m), angle: " << angle_deg << "deg, climb_rate: " << climb_rate << "m/min";
		}
		cout << endl;
	}

private:
	static const std::string transfer_state_str[];
	static const std::string transfer_type_str[];
};

const std::string AltitudeTransfer::transfer_state_str[] = {
	"TRANSFER_START",
	"TRANSFER_SPEED_TRANSFER_BEGINNING",
	"TRANSFER_STEADY",
	"TRANSFER_SPEED_TRANSFER_ENDING",
	"TRANSFER_FINISHED"
};

const std::string AltitudeTransfer::transfer_type_str[] = {
	"TRANSFER_ASCENDING",
	"TRANSFER_DESCENDING",
	"TRANSFER_NONE"
};


class Plane {
public:
	typedef enum {
		GROUND = 0,
		CHANGING_ALTITUDE = 1,
		CRUISE_FLIGHT = 2,
		STALL = 3
	} State;

	typedef struct {
		double pos_x_ft;
		double altitude_ft;
		double speed_m_min;
		double computed_angle_deg;
		State state;
	} Data;

	const std::string plane_state_str[4] = {
		"GROUND",
		"CHANGING_ALTITUDE",
		"CRUISE_FLIGHT",
		"STALL"
	};
	
	Plane(microseconds timestep, double engine_power = 100)
		: timestep(timestep), engine_power(engine_power) {};

	void dump_status()
	{
		auto plane_data = get_data();
		auto acceleration = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
		
		auto computed_angle	= compute_angle();
		auto computed_angle_deg = RAD_TO_DEG(computed_angle);
		auto speed_m_s = plane_data.speed_m_min / 60.0f;

		cout << fixed << "state: " << plane_state_str[state] << endl;
		cout << fixed << "altitude: " << plane_data.altitude_ft << "ft (" << altitude << "m)" << endl;
		cout << fixed << "computed angle: " << computed_angle << "rad (" << computed_angle_deg << "deg)" << endl;
		cout << fixed << "speed: " << plane_data.speed_m_min << "m/min (" << speed_m_s << "m/s) (x=" << speed_x << "m/min, y=" << speed_y << "m/min)" << endl;
		cout << fixed << "acc: "<< acceleration << "m/min² " << "(x=" << acc_x << "m/min², y=" << acc_y << "m/min²)" << endl;
		cout << fixed << "engine_power: " << engine_power << "%" << endl;
		if(!is_altitude_transfer_done()) {
			cout << fixed << "speed transfer: beginning(" << altitude_speed_transfer_beginning_end << "m), "
			     << "ending(" << altitude_speed_transfer_ending_start << "m, acc_x: " << speed_transfer_ending_acc_x << "m/min², acc_y: "
				 << speed_transfer_ending_acc_y << "m/min²)" << endl;
			altitude_transfer.dump_status();
		}
	}

	Data get_data() const
	{
		auto altitude_ft = METER_TO_FOOT(altitude);
		auto altitude_rounded = round(altitude_ft);

		auto pos_x_ft = METER_TO_FOOT(pos_x);
		auto pos_x_rounded = round(pos_x_ft * 10.0f) / 10.0f;

		// 1m/min 
		auto speed_rounded = round(sqrt(pow(speed_x, 2) + pow(speed_y, 2)) * 10.0f ) / 10.0f;

		auto computed_angle_deg = RAD_TO_DEG(compute_angle());

		return {
			pos_x_rounded,
			altitude_rounded,
			speed_rounded,
			computed_angle_deg,
			state
		};
	}

	void set_altitude_transfer(double target_altitude_ft)
	{
		auto plane_data = get_data();

		double target_altitude_m = max(min(target_altitude_ft * FT1_IN_M, MAX_ALTITUDE), GROUND_LEVEL);
		double angle = MAX_STABLE_ANGLE / 2.0f;

		auto transfer_type = AltitudeTransfer::TRANSFER_NONE;
		if(target_altitude_ft > plane_data.altitude_ft) {
			transfer_type = AltitudeTransfer::TRANSFER_ASCENDING;
		} else if(target_altitude_ft < plane_data.altitude_ft) {
			transfer_type = AltitudeTransfer::TRANSFER_DESCENDING;
			angle *= -1;
		} else {
			altitude_transfer = AltitudeTransfer();
			return;
		}

		double climb_rate = MAX_SPEED * sin(angle);
		altitude_transfer = AltitudeTransfer(transfer_type, target_altitude_m, angle, climb_rate);
	}

	void set_altitude_transfer(double attack_angle_deg, double climb_rate_m_min, double max_altitude)
	{
		double target_altitude_m = FOOT_TO_METER(max_altitude);
		double angle_rad = DEG_TO_RAD(attack_angle_deg);
		angle_rad = max(min(angle_rad, MAX_STABLE_ANGLE), - MAX_STABLE_ANGLE);

		climb_rate_m_min = copysign(climb_rate_m_min, angle_rad);
		double max_climb_rate = MAX_SPEED * sin(angle_rad);
		double climb_rate = min(climb_rate_m_min, max_climb_rate);

		auto transfer_type = AltitudeTransfer::TRANSFER_NONE;
		if(angle_rad > 0)
			transfer_type = AltitudeTransfer::TRANSFER_ASCENDING;
		else if(angle_rad < 0)
			transfer_type = AltitudeTransfer::TRANSFER_DESCENDING;

		altitude_transfer = AltitudeTransfer(transfer_type, target_altitude_m, abs(angle_rad), climb_rate);
	}

	bool is_altitude_transfer_done() {
		return altitude_transfer.state == AltitudeTransfer::TRANSFER_FINISHED;
	}

	void tick()
	{
		compute_speed_transfers();
		switch(altitude_transfer.state) {
			case AltitudeTransfer::TRANSFER_START: {
				state = State::CHANGING_ALTITUDE;	
				altitude_transfer.state = AltitudeTransfer::TRANSFER_SPEED_TRANSFER_BEGINNING;
				break;
			}

			case AltitudeTransfer::TRANSFER_SPEED_TRANSFER_BEGINNING: {
				bool speed_transfer_done =
					abs(speed_y - altitude_transfer.climb_rate) < 0.01 ||
					abs(altitude_speed_transfer_ending_start - altitude) < MIN_ALTITUDE_DELTA;

				if(!speed_transfer_done) {
					acc_y = speed_transfer_beginning_acc_y;
					acc_x = speed_transfer_beginning_acc_x;
					break;
				} else {
					acc_y = 0;
					acc_x = 0;
					altitude_transfer.state = AltitudeTransfer::TRANSFER_STEADY;
				}
			}

			case AltitudeTransfer::TRANSFER_STEADY: {
				bool steady_transfer_done = abs(altitude_speed_transfer_ending_start - altitude) < MIN_ALTITUDE_DELTA;

				if(!steady_transfer_done) {
					break;
				} else {
					altitude_transfer.state = AltitudeTransfer::TRANSFER_SPEED_TRANSFER_ENDING;
				}
			}

			case AltitudeTransfer::TRANSFER_SPEED_TRANSFER_ENDING: {
				bool end_speed_transfer_done =
					abs(altitude_transfer.altitude - altitude) < MIN_ALTITUDE_DELTA ||
					abs(speed_y) < 0.1;

				if(!end_speed_transfer_done) {
					acc_y = speed_transfer_ending_acc_y;
					acc_x = speed_transfer_ending_acc_x;
					break;
				} else {
					acc_y = 0;
					acc_x = 0;
					speed_x = MAX_SPEED;
					speed_y = 0;
					altitude_transfer.state = AltitudeTransfer::TRANSFER_FINISHED;
				}
			}

			case AltitudeTransfer::TRANSFER_FINISHED: {
				auto plane_data = get_data();
				if(plane_data.altitude_ft == 0) {
					speed_x = 0;
					state = GROUND;
				} else {
					state = CRUISE_FLIGHT;
				}

				break;
			}
		};

		update_speed();
		update_position();
	}

private:
	typedef struct {
		double acceleration;

		double speed_y;
		double speed_x;

		double desired_speed_y;
		double desired_speed_x;
	} SpeedTransferInfo;

	typedef struct {
		double acc_x;
		double acc_y;
		double distance_y;
	} SpeedTransferData;
	
	const double MAX_ALTITUDE = FOOT_TO_METER(40000.0f); // 40000ft in meters
	const double MAX_STABLE_ANGLE = DEG_TO_RAD(15.0f); // 15 deg in radian
	const double GROUND_LEVEL = 0.0f; // meters
	const double MAX_SPEED = 780.0f; // m/min
	const double PRECISION_ALTITUDE = FOOT_TO_METER(0.1f);
	const double MIN_ALTITUDE_DELTA = PRECISION_ALTITUDE / 10.0f;
	
	AltitudeTransfer altitude_transfer;
	microseconds timestep;

	State state = GROUND;
	double pos_x = 0;    // m
	double altitude = 0; // m
	double speed_x = 0;  // m/min
	double speed_y = 0;  // m/min
	double acc_x = 0;    // m/min²
	double acc_y = 0;    // m/min²

	double speed_transfer_ending_acc_x = 0;
	double speed_transfer_ending_acc_y = 0;
	double speed_transfer_beginning_acc_x = 0;
	double speed_transfer_beginning_acc_y = 0;

	double altitude_speed_transfer_beginning_end = 0;
	double altitude_speed_transfer_ending_start = 0;

	double engine_power = 0;

	void compute_speed_transfers()
	{
		if(altitude_transfer.state == AltitudeTransfer::TRANSFER_SPEED_TRANSFER_BEGINNING) {
			auto speed_transfer_beginning_result = compute_speed_transfer({
				engine_power * ENGINE_POWER_PER_PERCENT,
				speed_y,
				speed_x,
				altitude_transfer.climb_rate,
				altitude_transfer.climb_rate / tan(altitude_transfer.angle),
			});

			altitude_speed_transfer_beginning_end = altitude + speed_transfer_beginning_result.distance_y;
			speed_transfer_beginning_acc_x = speed_transfer_beginning_result.acc_x;
			speed_transfer_beginning_acc_y = speed_transfer_beginning_result.acc_y;
		}
	
		if(altitude_transfer.state == AltitudeTransfer::TRANSFER_SPEED_TRANSFER_BEGINNING ||
		   altitude_transfer.state == AltitudeTransfer::TRANSFER_SPEED_TRANSFER_ENDING) {
			auto speed_transfer_end_result = compute_speed_transfer({
				engine_power * ENGINE_POWER_PER_PERCENT,
				speed_y,
				speed_x,
				0.0f,
				MAX_SPEED
			});

			altitude_speed_transfer_ending_start = altitude_transfer.altitude - speed_transfer_end_result.distance_y;
			speed_transfer_ending_acc_x = speed_transfer_end_result.acc_x;
			speed_transfer_ending_acc_y = speed_transfer_end_result.acc_y;
		}
	}

	SpeedTransferData compute_speed_transfer(const SpeedTransferInfo& info) const {
		SpeedTransferData result;

		auto desired_speed_x = info.desired_speed_x;
		auto desired_speed_y = info.desired_speed_y;

		// compute the share of the available acceleration that goes to each component (x & y)
		auto delta_speed_y = desired_speed_y - info.speed_y;
		auto delta_speed_x = desired_speed_x - info.speed_x;

		auto desired_speed = sqrt(pow(desired_speed_x, 2) + pow(desired_speed_y, 2));
		if(desired_speed > MAX_SPEED) {
			auto speed_ratio = MAX_SPEED / desired_speed;
			delta_speed_x *= speed_ratio;
			delta_speed_y *= speed_ratio;
		}

		auto delta_speed_angle = atan(delta_speed_y / delta_speed_x);
		delta_speed_angle = max(min(delta_speed_angle, MAX_STABLE_ANGLE), -MAX_STABLE_ANGLE);

		auto share_acc_y = sin(delta_speed_angle);
		auto share_acc_x = cos(delta_speed_angle);

		// set the acceleration for each component
		result.acc_y = copysign(info.acceleration * share_acc_y, delta_speed_y);
		result.acc_x = copysign(info.acceleration * share_acc_x, delta_speed_x);

		// compute the distance that we will have to travel in the y direction (altitude)
		// to complete that first speed transfer
		auto average_speed_y = (desired_speed_y + info.speed_y) / 2.0f;
		auto speed_transfer_beginning_duration = delta_speed_y / result.acc_y;
	
		result.distance_y = speed_transfer_beginning_duration * average_speed_y;

		return result;
	}

	double compute_angle() const
	{
		return (speed_x != 0) ? atan(speed_y / speed_x) : 0.0f;
	}

	void update_speed()
	{
		speed_x += (acc_x / US_IN_1MIN) * timestep.count();
		speed_y += (acc_y / US_IN_1MIN) * timestep.count();
	}

	void update_position()
	{
		altitude += speed_y * timestep.count() / US_IN_1MIN;
		pos_x += speed_x * timestep.count() / US_IN_1MIN;
	}
};

void assert_constraints(const Plane& plane)
{
	auto plane_data = plane.get_data();

	if(plane_data.altitude_ft >= FOOT_TO_METER(40000.0f)) {
		throw std::runtime_error("too high");
	}

	if(plane_data.altitude_ft < 0) {
		throw std::runtime_error("it's a plane, not a tunnel boring machine");
	}
	
	if(abs(plane_data.computed_angle_deg) > 15.0f) {
		throw std::runtime_error("angle is not within [-15, 15]deg bounds");
	}
	
	if(plane_data.speed_m_min > 800.0f) {
		throw std::runtime_error("too faaaaaaaaaaaaast");
	}
}

#endif // PLANE_H

