## Get Started

### 1. Loading the sensor driver and formats

```bash
cd ~
sudo dtoverlay genx320,cam0
./rp5_setup_v4l.sh
```

### 2. Build

```bash
cd StereoEventRecorder
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

### 3. Run (Synchronized Stereo Recording)

Start the slave camera first, then start the master camera.

```bash
# Terminal 1 (slave)
./stereo_event_recorder --slave

# Terminal 2 (master)
./stereo_event_recorder --master
```

Optional arguments:

```bash
# specify serial
./stereo_event_recorder --slave --serial <slave_serial>
./stereo_event_recorder --master --serial <master_serial>

# specify output RAW path
./stereo_event_recorder --slave --output-raw-file slave.raw
./stereo_event_recorder --master --output-raw-file master.raw
```

Default output files:

- Slave: `events_slave.raw`
- Master: `events_master.raw`

Both outputs are recorded as RAW event files.

EVT3 note:

- On HAL versions that provide `metavision/hal/device/device_config.h`, the program explicitly requests EVT3.
- On HAL versions that do not provide this header, the program falls back to the compatible open API and records with the device/SDK default format (typically EVT3 on GenX320 setups).

To stop recording, press Ctrl-C in each terminal.

### 4. Decode the output file

```bash
metavision_evt3_raw_file_decoder /path/to/input.raw /path/to/output.csv
```

If you get like "Unknown command: metavision_evt3_raw_file_decoder", you have to compile the code first.

[Compiling C++ code samples](https://docs.prophesee.ai/stable/samples/compilation/compilation.html#chapter-samples-cpp-compilation)

1. `$ cd ~`
2. `$ cp -r /usr/local/share/metavision/standalone_samples/metavision_evt3_raw_file_decoder ~/`
3. `$ mkdir build && cd build`
4. `$ cmake .. -DCMAKE_BUILD_TYPE=Release`
5. `$ cmake --build . --config Release`

Finally, you can move the exe file to bin directory.
`$ sudo cp ./metavision_evt3_raw_file_decoder /usr/local/bin/`

Other decoders are here. [PROPHESEE Encoder/Decoder samples](https://docs.prophesee.ai/stable/samples/standalone.html)

## Notes

This repository is developed with reference to PROPHESEE's SDK, official documentation, and related public resources.

Referenced resources:

- https://docs.prophesee.ai/stable/get_started/get_started_cpp.html
- https://docs.prophesee.ai/stable/samples/modules/hal/hal_sync.html#chapter-samples-hal-hal-sync
- https://github.com/prophesee-ai/openeb
