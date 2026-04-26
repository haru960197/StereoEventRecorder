#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <metavision/hal/device/device.h>
#if __has_include(<metavision/hal/device/device_config.h>)
#include <metavision/hal/device/device_config.h>
#define HAS_HAL_DEVICE_CONFIG 1
#else
#define HAS_HAL_DEVICE_CONFIG 0
#endif
#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_event_decoder.h>
#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/hal/facilities/i_events_stream_decoder.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/error_utils.h>

namespace {

std::atomic<bool> g_stop_requested{false};

void signal_handler(int) {
	g_stop_requested.store(true);
}

struct ProgramOptions {
	bool master_mode = false;
	bool slave_mode  = false;
	std::string serial;
	std::string output_raw_path;
};

void print_help(const char *program_name) {
	std::cout << "Stereo event recorder with synchronization support\n\n"
			  << "Usage:\n"
			  << "  " << program_name << " --master [--serial <id>] [--output-raw-file <path>]\n"
			  << "  " << program_name << " --slave  [--serial <id>] [--output-raw-file <path>]\n\n"
			  << "Options:\n"
			  << "  --master                  Run camera in synchronization master mode.\n"
			  << "  --slave                   Run camera in synchronization slave mode.\n"
			  << "  -s, --serial <id>         Camera serial. If omitted, first available camera is used.\n"
			  << "  -o, --output-raw-file     Output RAW path. Default is events_master.raw or events_slave.raw.\n"
			  << "  -h, --help                Show this help.\n";
}

bool parse_args(int argc, char *argv[], ProgramOptions &options) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--master") == 0) {
			options.master_mode = true;
		} else if (std::strcmp(argv[i], "--slave") == 0) {
			options.slave_mode = true;
		} else if (std::strcmp(argv[i], "-s") == 0 || std::strcmp(argv[i], "--serial") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for --serial option." << std::endl;
				return false;
			}
			options.serial = argv[++i];
		} else if (std::strcmp(argv[i], "-o") == 0 || std::strcmp(argv[i], "--output-raw-file") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for --output-raw-file option." << std::endl;
				return false;
			}
			options.output_raw_path = argv[++i];
		} else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
			print_help(argv[0]);
			return false;
		} else {
			std::cerr << "Unknown option: '" << argv[i] << "'" << std::endl;
			return false;
		}
	}

	if (options.master_mode == options.slave_mode) {
		std::cerr << "Please specify exactly one of --master or --slave." << std::endl;
		return false;
	}

	if (options.output_raw_path.empty()) {
		options.output_raw_path = options.master_mode ? "events_master.raw" : "events_slave.raw";
	}

	return true;
}

} // namespace

int main(int argc, char *argv[]) {
	ProgramOptions options;
	if (!parse_args(argc, argv, options)) {
		return 1;
	}

	std::signal(SIGINT, signal_handler);

	try {
		std::cout << "Opening camera..." << std::endl;

		std::unique_ptr<Metavision::Device> device;

#if HAS_HAL_DEVICE_CONFIG
		Metavision::DeviceConfig config;
		config.set_format("EVT3");

		if (options.serial.empty()) {
			device = Metavision::DeviceDiscovery::open("", config);
		} else {
			device = Metavision::DeviceDiscovery::open(options.serial, config);
		}
#else
		if (options.serial.empty()) {
			device = Metavision::DeviceDiscovery::open("");
		} else {
			device = Metavision::DeviceDiscovery::open(options.serial);
		}
#endif

		if (!device) {
			std::cerr << "Camera opening failed." << std::endl;
			return 1;
		}
		std::cout << "Camera open." << std::endl;

		auto *i_eventsstream = device->get_facility<Metavision::I_EventsStream>();
		if (!i_eventsstream) {
			std::cerr << "Could not initialize events stream." << std::endl;
			return 2;
		}

		auto *i_camera_synchronization = device->get_facility<Metavision::I_CameraSynchronization>();
		if (!i_camera_synchronization) {
			std::cerr << "Could not initialize camera synchronization facility." << std::endl;
			return 3;
		}

		if (options.master_mode) {
			if (!i_camera_synchronization->set_mode_master()) {
				std::cerr << "Could not set Master mode. Synchronization might not be supported." << std::endl;
				return 3;
			}
			std::cout << "Set mode Master successful." << std::endl;
		} else {
			if (!i_camera_synchronization->set_mode_slave()) {
				std::cerr << "Could not set Slave mode. Synchronization might not be supported." << std::endl;
				return 3;
			}
			std::cout << "Set mode Slave successful. Waiting for master before recording..." << std::endl;
		}

		auto *i_stream_decoder = device->get_facility<Metavision::I_EventsStreamDecoder>();
		auto *i_cddecoder      = device->get_facility<Metavision::I_EventDecoder<Metavision::EventCD>>();
		if (!i_stream_decoder || !i_cddecoder) {
			std::cerr << "Could not initialize decoder facilities." << std::endl;
			return 4;
		}

		std::atomic<bool> slave_synced{options.master_mode};
		std::atomic<bool> slave_wait_msg_printed{false};

		i_cddecoder->add_event_buffer_callback([&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
			if (begin == end) {
				return;
			}

			if (!options.slave_mode || slave_synced.load()) {
				return;
			}

			const bool waiting_for_master = (begin->t == 0 && (end - 1)->t == 0);
			if (waiting_for_master) {
				if (!slave_wait_msg_printed.exchange(true)) {
					std::cout << "Slave is waiting for master startup..." << std::endl;
				}
				return;
			}

			slave_synced.store(true);
			std::cout << "Master detected. Slave recording can start." << std::endl;
		});

		i_eventsstream->start();
		std::cout << "Camera stream started." << std::endl;

		std::atomic<bool> stop_decoding{false};
		std::thread decoding_loop([&]() {
			while (!stop_decoding.load() && !g_stop_requested.load()) {
				const short poll_status = i_eventsstream->poll_buffer();
				if (poll_status < 0) {
					break;
				}

				auto raw_data = i_eventsstream->get_latest_raw_data();
				if (raw_data) {
					i_stream_decoder->decode(raw_data.begin(), raw_data.data() + raw_data.size());
				}
			}
		});

		bool recording_started = false;
		while (!g_stop_requested.load()) {
			if (!recording_started && slave_synced.load()) {
				i_eventsstream->log_raw_data(options.output_raw_path);
				recording_started = true;
				std::cout << "Recording started: " << options.output_raw_path << std::endl;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		std::cout << "Ctrl-C received, stopping camera..." << std::endl;

		stop_decoding.store(true);
		i_eventsstream->stop();
		if (decoding_loop.joinable()) {
			decoding_loop.join();
		}

		std::cout << "Camera stopped." << std::endl;
		return 0;
	} catch (const Metavision::BaseException &e) {
		std::cerr << "Metavision error: " << e.what() << std::endl;
		return 10;
	} catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 11;
	}
}
