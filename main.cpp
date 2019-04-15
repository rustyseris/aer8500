#include <stdexcept>
#include <iostream>
#include <chrono>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <unistd.h>

using namespace std;
using namespace chrono;

#define M1_IN_FT 3.28084f
#define FT1_IN_M 0.3048f
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
			auto altitude_ft = altitude * M1_IN_FT;
			auto angle_deg = angle * 180 / M_PI;
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

	const double MAX_ALTITUDE = 40000.0f * FT1_IN_M; // 40000ft in meters
	const double MAX_STABLE_ANGLE = 15.0f * M_PI / 180.0f; // 15 deg in radian
	const double GROUND_LEVEL = 0.0f; // meters
	const double MAX_SPEED = 800.0f; // m/min

	Plane(microseconds tickrate, double engine_power = 100)
		: tickrate(tickrate), engine_power(engine_power) {};

	void dump_status()
	{
		auto plane_data = get_data();
		auto acceleration = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
		
		auto computed_angle	= compute_angle();
		auto computed_angle_deg = computed_angle * 180.0f / M_PI;
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

	Data get_data()
	{
		// 0.1ft precision
		auto altitude_ft = altitude * M1_IN_FT;
		auto altitude_rounded = round(altitude_ft * 10) / 10.0f;

		// 1m/min 
		auto speed_rounded = round(sqrt(pow(speed_x, 2) + pow(speed_y, 2)));

		auto computed_angle_deg = compute_angle() * 180.0f / M_PI;

		return {
			altitude_rounded,
			speed_rounded,
			computed_angle_deg,
			state
		};
	}

	void set_altitude_transfer(double target_altitude_ft)
	{
		auto plane_data = get_data();

		auto transfer_type = AltitudeTransfer::TRANSFER_NONE;
		if(target_altitude_ft > plane_data.altitude_ft) {
			transfer_type = AltitudeTransfer::TRANSFER_ASCENDING;
		} else if(target_altitude_ft < plane_data.altitude_ft) {
			transfer_type = AltitudeTransfer::TRANSFER_DESCENDING;
		} else {
			altitude_transfer = AltitudeTransfer();
			return;
		}

		double target_altitude_m = max(min(target_altitude_ft * FT1_IN_M, MAX_ALTITUDE), GROUND_LEVEL);
		double angle = MAX_STABLE_ANGLE / 2;
		double climb_rate = MAX_SPEED * sin(angle);

		altitude_transfer = AltitudeTransfer(transfer_type, target_altitude_m, angle, climb_rate);
	}

	void set_altitude_transfer(double attack_angle_deg, double climb_rate_m_min, double max_altitude)
	{
		double target_altitude_m = max_altitude;
		double angle_rad = attack_angle_deg * M_PI / 180.0f;
		angle_rad = max(min(angle_rad, MAX_STABLE_ANGLE), - MAX_STABLE_ANGLE);
		
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

	static SpeedTransferData compute_speed_transfer(const SpeedTransferInfo& info) {
		SpeedTransferData result;

		auto desired_speed_x = info.desired_speed_x;
		auto desired_speed_y = info.desired_speed_y;

		// compute the share of the available acceleration that goes to each component (x & y)
		auto delta_speed_y = desired_speed_y - info.speed_y;
		auto delta_speed_x = desired_speed_x - info.speed_x;
		auto delta_speed_sum = abs(delta_speed_y) + abs(delta_speed_x);

		auto share_acc_y = abs(delta_speed_y) / delta_speed_sum;
		auto share_acc_x = abs(delta_speed_x) / delta_speed_sum;

		// set the acceleration for each component
		result.acc_y = copysign(info.acceleration * share_acc_y, delta_speed_y);
		result.acc_x = info.acceleration * share_acc_x;

		// compute the distance that we will have to travel in the y direction (altitude)
		// to complete that first speed transfer
		auto average_speed_y = (desired_speed_y + info.speed_y) / 2.0f;
		auto speed_transfer_beginning_duration = delta_speed_y / result.acc_y;
	
		result.distance_y = speed_transfer_beginning_duration * average_speed_y;

		return result;
	}

	void tick()
	{
		switch(altitude_transfer.state) {
			case AltitudeTransfer::TRANSFER_START: {
				state = State::CHANGING_ALTITUDE;

				auto total_acceleration = engine_power * ENGINE_POWER_PER_PERCENT;

				double desired_speed_x = altitude_transfer.climb_rate / tan(altitude_transfer.angle);
				double desired_speed_y =  altitude_transfer.climb_rate;
				if(altitude_transfer.type == AltitudeTransfer::TRANSFER_DESCENDING)
					desired_speed_y *= -1;

				auto speed_transfer_beginning_result = compute_speed_transfer({
					total_acceleration,
					speed_y,
					speed_x,
					desired_speed_y,
					desired_speed_x,
				});

				altitude_speed_transfer_beginning_end = altitude + speed_transfer_beginning_result.distance_y;
				acc_x = speed_transfer_beginning_result.acc_x;
				acc_y = speed_transfer_beginning_result.acc_y;

				altitude_transfer.state = AltitudeTransfer::TRANSFER_SPEED_TRANSFER_BEGINNING;
				break;
			}

			case AltitudeTransfer::TRANSFER_SPEED_TRANSFER_BEGINNING: {
				bool speed_transfer_done = false;

				auto speed_transfer_end_result = compute_speed_transfer({
					engine_power * ENGINE_POWER_PER_PERCENT,
					speed_y,
					speed_x,
					0.0f,
					MAX_SPEED
				});

				altitude_speed_transfer_ending_start = altitude_transfer.altitude - speed_transfer_end_result.distance_y;

				switch(altitude_transfer.type) {
					case AltitudeTransfer::TRANSFER_ASCENDING:
						speed_transfer_done =
							(altitude >= altitude_speed_transfer_beginning_end) ||
							(altitude >= altitude_speed_transfer_ending_start);
						break;

					case AltitudeTransfer::TRANSFER_DESCENDING:
						speed_transfer_done =
							(altitude <= altitude_speed_transfer_beginning_end) ||
							(altitude <= altitude_speed_transfer_ending_start);
						break;

					case AltitudeTransfer::TRANSFER_NONE:
						throw std::runtime_error("shouldn't encounter TRANSFER_NONE in TRANSFER_SPEED_TRANSFER_BEGINNING handler");
				};

				if(speed_transfer_done) {
					acc_y = 0;
					acc_x = 0;
					speed_transfer_ending_acc_x = speed_transfer_end_result.acc_x;
					speed_transfer_ending_acc_y = speed_transfer_end_result.acc_y;
					altitude_transfer.state = AltitudeTransfer::TRANSFER_STEADY;
				} else {
					break;
				}
			}

			case AltitudeTransfer::TRANSFER_STEADY: {
				bool steady_transfer_done = false;

				switch(altitude_transfer.type) {
					case AltitudeTransfer::TRANSFER_ASCENDING:
						steady_transfer_done = altitude >= altitude_speed_transfer_ending_start;
						break;

					case AltitudeTransfer::TRANSFER_DESCENDING:
						steady_transfer_done = altitude <= altitude_speed_transfer_ending_start;
						break;

					case AltitudeTransfer::TRANSFER_NONE:
						throw std::runtime_error("shoudln't encounter TRANSFER_NONE in TRANSFER_STEADY handler");
				}

				if(steady_transfer_done) {
					acc_y = speed_transfer_ending_acc_y;
					acc_x = speed_transfer_ending_acc_x;
					altitude_transfer.state = AltitudeTransfer::TRANSFER_SPEED_TRANSFER_ENDING;
				} else {
					break;
				}
			}

			case AltitudeTransfer::TRANSFER_SPEED_TRANSFER_ENDING: {
				bool end_speed_transfer_done = false;

				switch(altitude_transfer.type) {
					case AltitudeTransfer::TRANSFER_ASCENDING:
						// end_speed_transfer_done = altitude >= altitude_transfer.altitude;
						end_speed_transfer_done = speed_x >= 800;
						break;

					case AltitudeTransfer::TRANSFER_DESCENDING:
						end_speed_transfer_done = speed_x >= MAX_SPEED;
						// end_speed_transfer_done = altitude <= altitude_transfer.altitude;
						break;

					case AltitudeTransfer::TRANSFER_NONE:
						throw std::runtime_error("shoudln't encounter TRANSFER_NONE in TRANSFER_SPEED_TRANSFER_ENDING handler");
				}

				if(end_speed_transfer_done) {
					acc_y = 0;
					acc_x = 0;
					speed_x = MAX_SPEED;
					speed_y = 0;
					altitude_transfer.state = AltitudeTransfer::TRANSFER_FINISHED;
				} else {
					break;
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
			}
		};

		update_speed();
		update_altitude();
	}

private:
	AltitudeTransfer altitude_transfer;
	microseconds tickrate;

	State state = GROUND;
	double altitude = 0; // m
	double speed_x = 0;  // m/min
	double speed_y = 0;  // m/min
	double acc_x = 0;    // m/min²
	double acc_y = 0;    // m/min²

	double speed_transfer_ending_acc_x = 0;
	double speed_transfer_ending_acc_y = 0;

	double altitude_speed_transfer_beginning_end = 0;
	double altitude_speed_transfer_ending_start = 0;

	double engine_power = 0;

	double compute_angle() {
		return (speed_x != 0) ? atan(speed_y / speed_x) : 0.0f;
	}

	void update_speed()
	{
		speed_x += (acc_x / US_IN_1MIN) * tickrate.count();
		speed_y += (acc_y / US_IN_1MIN) * tickrate.count();
	}

	void update_altitude()
	{
		altitude += speed_y * tickrate.count() / US_IN_1MIN;
	}
};

int main()
{
	ofstream plane_altitude("./plane_altitude", ios::out | ios::trunc);
	ofstream plane_speed("./plane_speed", ios::out | ios::trunc);

	// every minute in simulation time
	auto tickrate = 100us;
	auto tick_count_before_dump = 10 * 1000; 

	Plane plane(tickrate);
	plane.set_altitude_transfer(1);

	cout << "plane initial status:" << endl;
	plane.dump_status();
	cout << endl;

	uint64_t tick_count = 0;
	while(true) {	
		tick_count++;
		plane.tick();
		
		// every minute
		if(tick_count % tick_count_before_dump == 0) {
			auto time_elapsed = duration_cast<seconds>(tickrate * tick_count).count();
			auto plane_data = plane.get_data();

			plane_altitude << time_elapsed << '\t' << plane_data.altitude_ft << "ft" << endl;
			plane_speed << time_elapsed << '\t' << plane_data.speed_m_min << "m/min" << endl;

			cout << '[' << time_elapsed << "s]" << endl;
			plane.dump_status();
			cout << endl;

			usleep(duration_cast<microseconds>(0.5s).count());
		}
	}

	return EXIT_SUCCESS;
}

