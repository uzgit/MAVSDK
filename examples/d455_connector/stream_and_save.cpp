#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <librealsense2/rs.hpp>
#include <unistd.h>
#include <string>
#include <sstream>
#include <experimental/filesystem>

#include <mavsdk/mavsdk.h>

#define WIDTH  640
#define HEIGHT 480
#define FRAMERATE 15

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param_server/param_server.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry_server/telemetry_server.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.h>
#include <mavsdk/plugins/mission/mission.h>

using namespace mavsdk;
using std::chrono::seconds;

bool prev_record = false;
bool record = false;

std::string bag_directory = "/home/joshua/Documents/";
int boot_index = 0;
int bag_index  = 0;
std::string bag_filename = "";

uint64_t channel_11;
float gimbal_pitch_control;
uint64_t channel_15;
uint64_t channel_16;

float interpolate(float input, float input_min, float input_max, float output_min, float output_max)
{
	return output_min + ((output_max - output_min) / (input_max - input_min)) * (input - input_min);
}

void subscribe_armed(Telemetry& telemetry)
{
    telemetry.subscribe_armed(
        [](bool is_armed) { std::cout << (is_armed ? "armed" : "disarmed") << '\n'; });
}

void rc_channels_callback( const mavlink_message_t & message )
{
	channel_11 = mavlink_msg_rc_channels_get_chan11_raw(&message);
	channel_15 = mavlink_msg_rc_channels_get_chan15_raw(&message);
	channel_16 = mavlink_msg_rc_channels_get_chan16_raw(&message);

	prev_record = record;
	record = channel_15 > 1500;

	// std::cerr << "Channel 15: " << channel_15 << ", record: " << record << std::endl;
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system([&mavsdk, &prom, &handle]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.unsubscribe_on_new_system(handle);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char** argv)
{
	if( argc < 2 )
	{
		std::cout << "Please supply a serial URL for the FCU." << std::endl;
		return -1;
	}

	std::stringstream format;
	format << bag_directory << "boot_%d_bag_%d.bag";

	int max_existing_boot_index = 0;
	for (const auto & entry : std::experimental::filesystem::v1::__cxx11::directory_iterator(bag_directory))
	{

		int existing_boot_index;
		int existing_bag_index = 0;
		
		std::stringstream path;
		path << entry.path().string();
	       	
		sscanf( entry.path().c_str(), format.str().c_str(), &existing_boot_index, &existing_bag_index );

		std::cout << entry.path() << ": " << existing_boot_index << std::endl;

		if( existing_boot_index < 5000 && existing_boot_index > max_existing_boot_index )
		{
			max_existing_boot_index = existing_boot_index;
		}
	}
	boot_index = max_existing_boot_index + 1;

	Mavsdk mavsdk;

	ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);
	
	if (connection_result != ConnectionResult::Success) {
	    std::cout << "Adding connection failed: " << connection_result << '\n';
	    return -1;
	}
	else
	{
		std::cout << "We have connected...?" << std::endl;
	}
	
	channel_11 = 69;
	gimbal_pitch_control = 0;
	channel_16 = 69;

	auto system = get_system(mavsdk);
	// Instantiate plugins.
	auto telemetry = Telemetry{system};
	auto gimbal = Gimbal{system};
	auto mavlink_passthrough = MavlinkPassthrough{system};

	mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS, rc_channels_callback);

	std::cout << "Start controlling gimbal...\n";
	Gimbal::Result gimbal_result = gimbal.take_control(Gimbal::ControlMode::Primary);
	if (gimbal_result != Gimbal::Result::Success) {
		std::cerr << "Could not take gimbal control: " << gimbal_result << '\n';
		return 1;
	}

	// say hello
	std::cout << "Starting D455 depth stream..." << std::endl;

	// set up the streams
	rs2::config config;
	config.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FRAMERATE);
	config.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FRAMERATE);
	config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
	config.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);

	// create a pipeline
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile = pipeline.start(config);

	// get the depth sensor and its scaling factor
	rs2::device device = pipeline_profile.get_device();
	rs2::depth_sensor depth_sensor = device.query_sensors().front().as<rs2::depth_sensor>();
	float depth_scale = depth_sensor.get_depth_scale();

	// set up the windows
	cv::String window_name = "D455 Stream";
	cv::namedWindow(window_name, cv::WINDOW_NORMAL);
	cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	while(true)
	{
		// detect edge
		if( record != prev_record )
		{
			pipeline.stop();

			rs2::config new_config;
			new_config.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FRAMERATE);
			new_config.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FRAMERATE);
			new_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
			new_config.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);

			if( record )
			{
				bag_index ++;

				std::stringstream filename;
				filename << bag_directory
					 << "boot_"
					 << boot_index
					 << "_bag_"
					 << bag_index
					 << ".bag";
				
				new_config.enable_record_to_file( filename.str() );
				
				bag_filename = "boot_" + std::to_string(boot_index) + "_bag_" + std::to_string(bag_index) + ".bag";
			}
			else
			{
				bag_filename = "";
			}
			// std::cout << "recording: " << record << std::endl;

			// create a pipeline
			pipeline_profile = pipeline.start(new_config);

			// get the depth sensor and its scaling factor
			device = pipeline_profile.get_device();
			depth_sensor = device.query_sensors().front().as<rs2::depth_sensor>();
			depth_scale = depth_sensor.get_depth_scale();
		}

		// get a depth frame
	
		if( channel_16 < 1500 )
		{
			auto frame_data = (pipeline.wait_for_frames()).first(RS2_STREAM_DEPTH).get_data();

			uint16_t* depth_data = (uint16_t*)frame_data;
			float depth_at_edge = depth_data[0] * depth_scale;

			// create opencv matrix representations of the images
			cv::Mat depth_image(cv::Size(WIDTH, HEIGHT), CV_16UC1, (void*)frame_data, cv::Mat::AUTO_STEP);

			depth_image.convertTo(depth_image, CV_64F);

			// colorize the depth image (for display only)
			cv::Mat depth_image_colorized(depth_image);
			cv::convertScaleAbs(depth_image_colorized, depth_image_colorized, 0.03);
			cv::applyColorMap(depth_image_colorized, depth_image_colorized, cv::COLORMAP_JET);
			//cv::putText(depth_image_colorized, "depth image", cv::Point(200, 200), cv::FONT_HERSHEY_SIMPLEX, 5, true);
			//cv::putText(depth_image_colorized, "depth image", cv::Point(200, 200), cv::FONT_HERSHEY_SIMPLEX, cv::Scalar(255, 0, 0));
			//cv::putText(depth_image_colorized, "depth image test",cv::Point(50,50),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,255),2,false);
			cv::putText(depth_image_colorized, bag_filename,cv::Point(50,50),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,255),2,false);
			cv::imshow(window_name, depth_image_colorized);
		}
		else
		{
			auto frame_data_color = (pipeline.wait_for_frames()).first(RS2_STREAM_COLOR).get_data();
			cv::Mat color_image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)frame_data_color, cv::Mat::AUTO_STEP);
			cv::imshow(window_name, color_image);
		}

		gimbal_pitch_control = interpolate( (float)channel_11, 1000, 2000, -1, 1);
		if( abs(gimbal_pitch_control) > 0.05 )
		{
			gimbal.set_pitch_rate_and_yaw_rate( 75 * (float)gimbal_pitch_control , 0.0f);
		}
		else
		{
			gimbal.set_pitch_rate_and_yaw_rate( 0.0f , 0.0f);
		}

		// wait a bit so the images actually appear
		cv::waitKey(1);
	}

	pipeline.stop();

	return 0;
}
