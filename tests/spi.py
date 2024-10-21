#!/usr/bin/env python3

import os
from saleae import automation

with automation.Manager.connect(port=10430) as manager:

    device_configuration = automation.LogicDeviceConfiguration(
        enabled_digital_channels=[2, 3, 4, 5, 6, 7],
        digital_sample_rate=50_000_000,
    )

    capture_configuration = automation.CaptureConfiguration(
        capture_mode=automation.DigitalTriggerCaptureMode(
                        trigger_type=automation.DigitalTriggerType.RISING,
                        trigger_channel_index=6,
                        trim_data_seconds=0.01,
                        after_trigger_seconds=1)
    )

    with manager.start_capture(
            device_configuration=device_configuration,
            capture_configuration=capture_configuration) as capture:

        capture.wait()

        spi_analyzer = capture.add_analyzer('SPI', label=f'Test Analyzer', settings={
            'MOSI': 2,
            'MISO': 3,
            'Clock': 5,
            'Enable': 4,
            'Bits per Transfer': '8 Bits per Transfer (Standard)'
        })

        output_dir = '/Users/jcw/Desktop'

        #analyzer_export_filepath = os.path.join(output_dir, 'spi_export.csv')
        #capture.export_data_table(
        #    filepath=analyzer_export_filepath,
        #    analyzers=[spi_analyzer]
        #)

        # Finally, save the capture to a file
        capture_filepath = os.path.join(output_dir, 'example_capture.sal')
        capture.save_capture(filepath=capture_filepath)
