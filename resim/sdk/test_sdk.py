from pathlib import Path
import tempfile

import resim.sdk.sdk as resim


def main() -> None:
    
    with resim.init(batch="example_batch", project="SDK-Test", system="example_system") as batch:
    test = Test("example_test", emissions_path)

    # Emit a couple of example metrics (validation disabled if no config present)
    test.emit("demo_metric", {"int_value": 1, "float_value": 3.14, "label": "ok"})

    # Attach a small log file
    log_path = emissions_path.parent / "example_log.txt"
    log_path.write_text("hello from test_sdk example")
    log = test.add_file(log_path)

    print(
        f"Created test '{test.name}' with emissions at {test.emissions_file}. "
        f"Added log '{log.filename}' ({log.size} bytes, type={log.log_type.value})."
    )


if __name__ == "__main__":
    main()
