#!/bin/python3

import pathlib
import sys

from resim.metrics.default_report_metrics import main as default_report_metrics_main

REPORT_METRICS_CONFIG_PATH = "/tmp/resim/inputs/report_config.json"


async def main() -> None:
    report_metrics_config = pathlib.Path(REPORT_METRICS_CONFIG_PATH)
    is_report_metrics_mode = report_metrics_config.is_file()
    if is_report_metrics_mode:
        # Call the default_report_metrics main function:
        await default_report_metrics_main()
    else:
        # Otherwise, we can't do anything: exit 1
        print("Metrics Build only compatible with reports")
        sys.exit(1)


if __name__ == "__main__":
    main()
