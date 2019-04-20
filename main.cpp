#include "plane.h"

int main(int argc, char* argv[])
{
	if(argc < 2 || string(argv[1]) != "-v" ) {
		cout.setstate(ios::failbit);
	}
	
	ofstream plane_altitude("./plane_altitude", ios::out | ios::trunc);
	ofstream plane_speed("./plane_speed", ios::out | ios::trunc);
	ofstream plane_angle("./plane_angle", ios::out | ios::trunc);

	auto timestep = 100us;
	// every minute in simulation time
	auto tick_count_before_dump = 10 * 1000 + 1; 

	Plane plane(timestep, 10);

	cout << "plane initial status:" << endl;
	plane.dump_status();
	cout << endl;

	static auto wait_until = 0;
	static auto done = false;

	uint64_t tick_count = 0;
	while(true) {	
		tick_count++;
		plane.tick();
		
		auto time_elapsed = duration_cast<seconds>(timestep * tick_count).count();

		// every minute
		if(tick_count % tick_count_before_dump == 0) {
			auto plane_data = plane.get_data();
			auto x = plane_data.pos_x_ft;

			plane_altitude << x << '\t' << plane_data.altitude_ft << "ft" << endl;
			plane_speed << x << '\t' << plane_data.vertical_speed << "m/min" << endl;
			plane_angle << x << '\t' << plane_data.computed_angle_deg << "deg" << endl;

			cout << '[' << time_elapsed << "s]" << endl;
			plane.dump_status();
			cout << endl;


			if(plane_data.altitude_ft == 500 && !done) {
				done = true;
				plane.set_altitude_transfer(plane_data.altitude_ft);
			} else if(done && plane.is_altitude_transfer_done()) {
				done = false;
			}

			usleep(duration_cast<microseconds>(50ms).count());	
		}
	
		if((wait_until == 0 && plane.is_altitude_transfer_done()) || (wait_until != 0 && time_elapsed > wait_until)) {
			plane.dump_status();
			cout << endl;

			wait_until = 0;

			int new_altitude = 0;
			cout << "new altitude: ";
			cin >> new_altitude;
			cin.ignore();

			if(new_altitude >= 0) {
				plane.set_altitude_transfer(new_altitude);
				cout << "going to altitude " << new_altitude << endl;
			} else {
				auto waiting_time = -new_altitude;
				wait_until = time_elapsed + waiting_time;	
				cout << "waiting for " << waiting_time << "s" << endl;
			}
		}


		try {
			assert_constraints(plane);	
		} catch(const std::exception& err) {
			cerr << "at tick " << tick_count << endl;
			cerr << endl << "PLANE FAILED TO MEET CONSTRAINTS" << endl;
			cerr << err.what() << endl << endl;
			cerr << "plane status while error: " << endl;
			plane.dump_status();
			return EXIT_FAILURE;
		}
	}

	return EXIT_SUCCESS;
}
