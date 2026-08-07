#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <sstream>
#include <cctype>

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
#include <metavision/hal/facilities/i_geometry.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/error_utils.h>

#ifdef __linux__
#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cstdint>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <cerrno>
#endif

namespace {

std::atomic<bool> g_stop_requested{false};
std::atomic<bool> g_q_pressed{false};

void signal_handler(int) {
	g_stop_requested.store(true);
}

struct ProgramOptions {
	bool master_mode = false;
	bool slave_mode  = false;
	std::string serial;
	std::string output_raw_path;
	float recording_duration = 15.0f;
	std::string hot_pixels_path;
	std::string subdev_path;
};

void print_help(const char *program_name) {
	std::cout << "Stereo event recorder with synchronization support\n\n"
			  << "Usage:\n"
			  << "  " << program_name << " --master [options]\n"
			  << "  " << program_name << " --slave  [options]\n\n"
			  << "Options:\n"
			  << "  --master                  Run camera in synchronization master mode.\n"
			  << "  --slave                   Run camera in synchronization slave mode.\n"
			  << "  -s, --serial <id>         Camera serial. If omitted, first available camera is used.\n"
			  << "  -o, --output-raw-file     Output RAW path. Default is events_master.raw or events_slave.raw.\n"
			  << "  -t, --time <seconds>      Recording duration in seconds (default: 15.0).\n"
			  << "                            Total time = 5.0s (warm-up) + duration + 5.0s (cool-down)\n"
			  << "  -c, --hot-pixels <path>   Hot pixels config JSON file path.\n"
			  << "  -d, --subdev <path>       V4L2 subdevice path (e.g. /dev/v4l-subdev0).\n"
			  << "  -h, --help                Show this help.\n";
}

int parse_args(int argc, char *argv[], ProgramOptions &options) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--master") == 0) {
			options.master_mode = true;
		} else if (std::strcmp(argv[i], "--slave") == 0) {
			options.slave_mode = true;
		} else if (std::strcmp(argv[i], "-s") == 0 || std::strcmp(argv[i], "--serial") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for --serial option." << std::endl;
				return -1;
			}
			options.serial = argv[++i];
		} else if (std::strcmp(argv[i], "-o") == 0 || std::strcmp(argv[i], "--output-raw-file") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for --output-raw-file option." << std::endl;
				return -1;
			}
			options.output_raw_path = argv[++i];
		} else if (std::strcmp(argv[i], "-t") == 0 || std::strcmp(argv[i], "--time") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for --time option." << std::endl;
				return -1;
			}
			char *end_ptr = nullptr;
			options.recording_duration = std::strtof(argv[++i], &end_ptr);
			if (end_ptr == argv[i] || *end_ptr != '\0' || options.recording_duration <= 0.0f) {
				std::cerr << "Invalid duration: '" << argv[i] << "'. Please provide a positive number." << std::endl;
				return -1;
			}
		} else if (std::strcmp(argv[i], "-c") == 0 || std::strcmp(argv[i], "--hot-pixels") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for --hot-pixels option." << std::endl;
				return -1;
			}
			options.hot_pixels_path = argv[++i];
		} else if (std::strcmp(argv[i], "-d") == 0 || std::strcmp(argv[i], "--subdev") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "Missing value for --subdev option." << std::endl;
				return -1;
			}
			options.subdev_path = argv[++i];
		} else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
			print_help(argv[0]);
			return 0;
		} else {
			std::cerr << "Unknown option: '" << argv[i] << "'" << std::endl;
			return -1;
		}
	}

	if (options.master_mode == options.slave_mode) {
		std::cerr << "Please specify exactly one of --master or --slave." << std::endl;
		return -1;
	}

	if (options.output_raw_path.empty()) {
		options.output_raw_path = options.master_mode ? "events_master.raw" : "events_slave.raw";
	}

	return 1;
}

#ifdef __linux__
// V4L2 pixel mask structures for GenX320
struct PixelRow {
	bool dirty;
	uint32_t vectors[10]; // 320 bits = 32 bits * 10
};

struct PixelGrid {
	uint32_t width;
	uint32_t height;
	PixelRow rows[320];
};

static constexpr uint32_t PSEE_ROI_CLASS = 0x4000;
static constexpr uint32_t PSEE_CID_ROI_CTRL = V4L2_CID_USER_BASE | PSEE_ROI_CLASS;
static constexpr uint32_t GENX320_CID_ROI_PIXEL_ARRAY = PSEE_CID_ROI_CTRL + 8;

static unsigned long make_iowr(unsigned char type, unsigned int nr, size_t size) {
	return (3UL << 30) | ((unsigned long)size << 16) | ((unsigned long)type << 8) | nr;
}

// Find the GenX320 sensor V4L2 sub-device path dynamically
static std::string find_sensor_subdev() {
	const std::string base = "/sys/class/video4linux";
	DIR *dir = opendir(base.c_str());
	if (!dir)
		return "";

	struct dirent *entry;
	while ((entry = readdir(dir)) != nullptr) {
		std::string name_str(entry->d_name);
		if (name_str.find("v4l-subdev") == std::string::npos)
			continue;

		std::string name_path = base + "/" + name_str + "/name";
		std::ifstream ifs(name_path);
		if (!ifs.is_open())
			continue;

		std::string sensor_name;
		std::getline(ifs, sensor_name);
		ifs.close();

		std::string lower = sensor_name;
		for (auto &c : lower)
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

		if (lower.find("genx320") != std::string::npos ||
			lower.find("psee") != std::string::npos ||
			lower.find("imx636") != std::string::npos) {
			closedir(dir);
			std::string dev_path = "/dev/" + name_str;
			std::cout << "[INFO] Discovered sensor subdevice: " << dev_path
					  << " (" << sensor_name << ")" << std::endl;
			return dev_path;
		}
	}
	closedir(dir);
	return "";
}

// Apply V4L2 pixel mask to disable hot pixels
static bool apply_hotpixel_mask(const std::string &dev_path, const std::vector<std::pair<int, int>> &hot_pixels) {
	PixelGrid grid{};
	grid.width = 320;
	grid.height = 320;

	for (int y = 0; y < 320; ++y) {
		grid.rows[y].dirty = false;
		for (int x = 0; x < 10; ++x) {
			grid.rows[y].vectors[x] = 0xFFFFFFFF;
		}
	}

	int masked_count = 0;
	for (const auto &[px, py] : hot_pixels) {
		if (px < 0 || px >= 320 || py < 0 || py >= 320) {
			std::cerr << "[WARN] Hot pixel (" << px << ", " << py
					  << ") is out of range [0, 320). Skipping." << std::endl;
			continue;
		}
		int vector_idx = px / 32;
		int bit_idx = px % 32;
		grid.rows[py].vectors[vector_idx] &= ~(1u << bit_idx);
		grid.rows[py].dirty = true;
		++masked_count;
	}

	std::cout << "[INFO] Masking " << masked_count << " hot pixel(s) via V4L2 device: " << dev_path << std::endl;

	struct v4l2_ext_control ctrl{};
	ctrl.id = GENX320_CID_ROI_PIXEL_ARRAY;
	ctrl.size = sizeof(grid);
	ctrl.ptr = &grid;

	struct v4l2_ext_controls ctrls{};
	ctrls.which = 0;
	ctrls.count = 1;
	ctrls.controls = &ctrl;

	unsigned long VIDIOC_S_EXT_CTRLS_CMD = make_iowr('V', 72, sizeof(ctrls));

	int fd = open(dev_path.c_str(), O_RDWR);
	if (fd < 0) {
		std::cerr << "[ERROR] Failed to open V4L2 device: " << dev_path
				  << " (errno=" << errno << ")" << std::endl;
		return false;
	}

	int ret = ioctl(fd, VIDIOC_S_EXT_CTRLS_CMD, &ctrls);
	close(fd);

	if (ret < 0) {
		std::cerr << "[ERROR] V4L2 ioctl VIDIOC_S_EXT_CTRLS failed (errno=" << errno << ")." << std::endl;
		return false;
	}

	std::cout << "[INFO] Hot pixel mask applied successfully." << std::endl;
	return true;
}

// RAII terminal restorer to guarantee terminal settings are reset on exit
class TerminalRestorer {
	struct termios old_t;
	bool active;
public:
	TerminalRestorer() : active(false) {
		if (tcgetattr(STDIN_FILENO, &old_t) >= 0) {
			active = true;
		}
	}
	~TerminalRestorer() {
		restore();
	}
	void restore() {
		if (active) {
			tcsetattr(STDIN_FILENO, TCSADRAIN, &old_t);
			active = false;
		}
	}
};

void input_loop(std::atomic<bool> &stop_flag) {
	struct termios old_t, new_t;
	if (tcgetattr(STDIN_FILENO, &old_t) < 0) {
		char ch;
		while (!stop_flag.load() && std::cin >> ch) {
			if (ch == 'q') {
				g_q_pressed.store(true);
				break;
			}
		}
		return;
	}

	new_t = old_t;
	new_t.c_lflag &= ~(ICANON | ECHO);
	new_t.c_cc[VMIN] = 1;
	new_t.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &new_t);

	while (!stop_flag.load() && !g_q_pressed.load()) {
		struct timeval tv = {0, 100000}; // 100ms timeout
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
		if (ret > 0) {
			char c;
			if (read(STDIN_FILENO, &c, 1) > 0) {
				if (c == 'q') {
					g_q_pressed.store(true);
					break;
				}
			}
		}
	}
	tcsetattr(STDIN_FILENO, TCSADRAIN, &old_t);
}
#endif

// Resolve the path to config/hot_pixels.json relative to the executable or role
static std::string resolve_config_path(const char *argv0, const std::string &role, const std::string &custom_path) {
	if (!custom_path.empty()) {
		std::ifstream test(custom_path);
		if (test.good()) return custom_path;
		std::cerr << "[WARN] Custom hot pixel config not found: " << custom_path << std::endl;
	}

	std::vector<std::string> candidates;
	if (!role.empty()) {
		candidates.push_back("config/hot_pixels_" + role + ".json");
		candidates.push_back("../config/hot_pixels_" + role + ".json");
	}
	candidates.push_back("config/hot_pixels.json");
	candidates.push_back("../config/hot_pixels.json");

	std::string exe_dir;
	{
		std::string argv0_str(argv0);
		auto pos = argv0_str.rfind('/');
		if (pos != std::string::npos) {
			exe_dir = argv0_str.substr(0, pos);
			if (!role.empty()) {
				candidates.push_back(exe_dir + "/../config/hot_pixels_" + role + ".json");
			}
			candidates.push_back(exe_dir + "/../config/hot_pixels.json");
		}
	}

	for (const auto &path : candidates) {
		std::ifstream test(path);
		if (test.good())
			return path;
	}
	return "";
}

// Load and parse config/hot_pixels.json
static bool load_hot_pixels(const std::string &path, std::vector<std::pair<int, int>> &out) {
	std::ifstream ifs(path);
	if (!ifs.is_open()) {
		std::cerr << "[ERROR] Cannot open hot pixel config: " << path << std::endl;
		return false;
	}

	std::string content((std::istreambuf_iterator<char>(ifs)),
						 std::istreambuf_iterator<char>());
	ifs.close();

	out.clear();

	size_t pos = 0;
	while (pos < content.size()) {
		pos = content.find('{', pos);
		if (pos == std::string::npos)
			break;

		size_t end = content.find('}', pos);
		if (end == std::string::npos)
			break;

		std::string obj = content.substr(pos, end - pos + 1);
		pos = end + 1;

		auto extract_int = [&](const std::string &key) -> std::pair<bool, int> {
			std::string pattern = "\"" + key + "\"";
			size_t kp = obj.find(pattern);
			if (kp == std::string::npos)
				return {false, 0};
			kp += pattern.size();
			while (kp < obj.size() && (obj[kp] == ' ' || obj[kp] == ':' || obj[kp] == '\t' || obj[kp] == '\n' || obj[kp] == '\r'))
				++kp;
			if (kp >= obj.size())
				return {false, 0};
			char *endp = nullptr;
			long val = std::strtol(obj.c_str() + kp, &endp, 10);
			if (endp == obj.c_str() + kp)
				return {false, 0};
			return {true, static_cast<int>(val)};
		};

		auto [xok, xv] = extract_int("x");
		auto [yok, yv] = extract_int("y");

		if (xok && yok) {
			out.emplace_back(xv, yv);
		}
	}

	std::cout << "[INFO] Loaded " << out.size() << " hot pixel(s) from: " << path << std::endl;
	return !out.empty() || content.find("\"hot_pixels\"") != std::string::npos;
}

} // namespace

int main(int argc, char *argv[]) {
	ProgramOptions options;
	int parse_res = parse_args(argc, argv, options);
	if (parse_res < 0) {
		return 1;
	} else if (parse_res == 0) {
		return 0;
	}

	std::signal(SIGINT, signal_handler);

#ifdef __linux__
	TerminalRestorer restorer;
	std::atomic<bool> stop_input{false};
	std::thread input_thread;
#endif

	static constexpr float MARGIN_BEFORE_SEC = 5.0f;
	static constexpr float MARGIN_AFTER_SEC = 5.0f;
	const float total_duration = MARGIN_BEFORE_SEC + options.recording_duration + MARGIN_AFTER_SEC;
	const float mask_apply_time = MARGIN_BEFORE_SEC;
	const float mask_clear_time = MARGIN_BEFORE_SEC + options.recording_duration;

	std::cout << "[INFO] Recording duration: " << options.recording_duration << "s (masked)" << std::endl;
	std::cout << "[INFO] Total recording time: " << total_duration << "s "
			  << "(warm-up " << MARGIN_BEFORE_SEC << "s + recording " << options.recording_duration << "s + cool-down " << MARGIN_AFTER_SEC << "s)" << std::endl;

	try {
		std::cout << "Opening camera..." << std::endl;

		std::unique_ptr<Metavision::Device> device;

#if HAS_HAL_DEVICE_CONFIG
		Metavision::DeviceConfig config;
		config.set_format("EVT3");
		config.enable_biases_range_check_bypass(true);

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

		// Load hot pixels config
		std::vector<std::pair<int, int>> hot_pixels;
		std::string role_str = options.master_mode ? "master" : "slave";
#ifdef __linux__
		std::string config_path = resolve_config_path(argv[0], role_str, options.hot_pixels_path);
		if (!config_path.empty()) {
			if (load_hot_pixels(config_path, hot_pixels)) {
				if (hot_pixels.empty()) {
					std::cout << "[INFO] Hot pixel config loaded, but it is empty." << std::endl;
				}
			} else {
				std::cerr << "[WARN] Failed to load hot pixel data from: " << config_path << std::endl;
			}
		} else {
			std::cerr << "[WARN] Hot pixel config not found. Phase 2 will not apply any mask." << std::endl;
		}

		// Find V4L2 subdevice
		std::string dev_path = options.subdev_path;
		if (dev_path.empty()) {
			dev_path = find_sensor_subdev();
		}
		if (dev_path.empty()) {
			dev_path = "/dev/v4l-subdev0";
			std::cerr << "[WARN] Sensor subdevice not found. Falling back to: " << dev_path << std::endl;
		} else {
			std::cout << "[INFO] Using sensor subdevice: " << dev_path << std::endl;
		}
#else
		std::cerr << "[WARN] Hot pixel masking is only supported on Linux (V4L2)." << std::endl;
#endif

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

#ifdef __linux__
		input_thread = std::thread(input_loop, std::ref(stop_input));
#endif

		bool recording_started = false;
		bool mask_applied = false;
		bool mask_cleared = false;
		bool q_pressed_action = false;
		float cool_down_start_time = 0.0f;
		auto start_time = std::chrono::steady_clock::now();

		while (!g_stop_requested.load()) {
			const short poll_status = i_eventsstream->poll_buffer();
			if (poll_status < 0) {
				std::cerr << "[WARN] Event stream ended unexpectedly." << std::endl;
				break;
			}

			if (poll_status > 0) {
				auto raw_data = i_eventsstream->get_latest_raw_data();
				if (raw_data) {
					i_stream_decoder->decode(raw_data.begin(), raw_data.data() + raw_data.size());
				}
			}

			if (!recording_started) {
				if (slave_synced.load()) {
					i_eventsstream->log_raw_data(options.output_raw_path);
					recording_started = true;
					start_time = std::chrono::steady_clock::now();
					std::cout << "[INFO] Recording started: " << options.output_raw_path << std::endl;
					std::cout << "[PHASE 1] 0-" << mask_apply_time << "s: Warm-up (No Mask)" << std::endl;
				}
			} else {
				const auto elapsed = std::chrono::steady_clock::now() - start_time;
				float seconds = std::chrono::duration<float>(elapsed).count();

				// Check for q key press
				if (g_q_pressed.load() && !q_pressed_action) {
					std::cout << "\n[INFO] 'q' key pressed. Ending recording and entering cool-down..." << std::endl;
					q_pressed_action = true;
					if (!mask_cleared) {
						std::cout << "[PHASE 3] Cool-down (Hot Pixel Mask OFF)" << std::endl;
#ifdef __linux__
						if (!hot_pixels.empty()) {
							std::vector<std::pair<int, int>> empty_list;
							apply_hotpixel_mask(dev_path, empty_list);
						}
#endif
						mask_cleared = true;
						cool_down_start_time = seconds;
					}
				}

				if (!q_pressed_action) {
					// Phase 2: Apply hot pixel mask after warm-up margin
					if (seconds >= mask_apply_time && !mask_applied) {
						std::cout << "\n[PHASE 2] " << mask_apply_time << "-" << mask_clear_time
								  << "s: Masked Recording (Hot Pixel Mask ON)" << std::endl;
#ifdef __linux__
						if (!hot_pixels.empty()) {
							apply_hotpixel_mask(dev_path, hot_pixels);
						} else {
							std::cout << "[WARN] Masking skipped (no hot pixels loaded)" << std::endl;
						}
#endif
						mask_applied = true;
					}

					// Phase 3: Clear mask before cool-down margin
					if (seconds >= mask_clear_time && !mask_cleared) {
						std::cout << "\n[PHASE 3] " << mask_clear_time << "-" << total_duration
								  << "s: Cool-down (Hot Pixel Mask OFF)" << std::endl;
#ifdef __linux__
						if (!hot_pixels.empty()) {
							std::vector<std::pair<int, int>> empty_list;
							apply_hotpixel_mask(dev_path, empty_list);
						} else {
							std::cout << "[INFO] Mask clear skipped (no mask was applied)" << std::endl;
						}
#endif
						mask_cleared = true;
					}

					if (seconds >= total_duration) {
						break;
					}
				} else {
					if (seconds >= cool_down_start_time + MARGIN_AFTER_SEC) {
						break;
					}
				}
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		std::cout << "Stopping camera..." << std::endl;

#ifdef __linux__
		stop_input.store(true);
		if (input_thread.joinable()) {
			input_thread.join();
		}
		restorer.restore();
#endif

		i_eventsstream->stop();
		i_eventsstream->stop_log_raw_data();
		std::cout << "Camera stopped." << std::endl;
		return 0;
	} catch (const Metavision::BaseException &e) {
		std::cerr << "Metavision error: " << e.what() << std::endl;
#ifdef __linux__
		stop_input.store(true);
		if (input_thread.joinable()) {
			input_thread.join();
		}
#endif
		return 10;
	} catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
#ifdef __linux__
		stop_input.store(true);
		if (input_thread.joinable()) {
			input_thread.join();
		}
#endif
		return 11;
	}
}
