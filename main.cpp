#include "plane.h"

int main(int argc, char* argv[])
{
	ofstream plane_altitude("./plane_altitude", ios::out | ios::trunc);
	ofstream plane_speed("./plane_speed", ios::out | ios::trunc);
	ofstream plane_angle("./plane_angle", ios::out | ios::trunc);

	auto timestep = 100us;
	// every minute in simulation time
	auto tick_count_before_dump = 10 * 1000 + 27; 

	Plane plane(timestep);
	plane.set_altitude_transfer(500);

	cout << "plane initial status:" << endl;
	plane.dump_status();
	cout << endl;

	if(argc < 2 || string(argv[1]) != "-v" ) {
		cout.setstate(ios::failbit);
	}

	uint64_t tick_count = 0;
	while(true) {	
		tick_count++;
		plane.tick();
		
		// every minute
		if(tick_count % tick_count_before_dump == 0) {
			auto time_elapsed = duration_cast<seconds>(timestep * tick_count).count();
			auto plane_data = plane.get_data();

			plane_altitude << time_elapsed << '\t' << plane_data.altitude_ft << "ft" << endl;
			plane_speed << time_elapsed << '\t' << plane_data.speed_m_min << "m/min" << endl;
			plane_angle << time_elapsed << '\t' << plane_data.computed_angle_deg << "deg" << endl;

			cout << '[' << time_elapsed << "s]" << endl;
			plane.dump_status();
			cout << endl;

			if(time_elapsed == 150){
				plane.set_altitude_transfer(0);
			}

			if(time_elapsed == 200) {
				plane.set_altitude_transfer(300);
			}

			if(time_elapsed == 224) {
				plane.set_altitude_transfer(700);
			}

			if(time_elapsed == 400) {
				plane.set_altitude_transfer(0);
			}

			if(time_elapsed == 500) {
				plane.set_altitude_transfer(15, 5000, 40000);
			}
			
			if(time_elapsed == 600) {
				return EXIT_SUCCESS;
			}

//			usleep(duration_cast<microseconds>(50ms).count());
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
