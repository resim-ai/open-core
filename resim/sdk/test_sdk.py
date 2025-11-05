from pathlib import Path
import tempfile
import uuid
import resim.sdk.sdk as resim


def main() -> None:
    print("Initializing batch")
    batch_name = f"example_batch_{uuid.uuid4().hex[:8]}"
    with resim.init(batch=batch_name, project="SDK-Test", system="TestSystem", branch="main", version="1.0.0") as batch:
        
        with batch.run_test("example_test") as test:
            # Emit a couple of example metrics (validation disabled if no config present)
            test.emit("demo_metric", {"int_value": 1, "float_value": 3.14, "label": "ok"})

            # Attach a small log file
            log_path = test.emissions_file.parent / "example_log.txt"
            log_path.write_text("hello from test_sdk example")
            log = test.add_file(log_path)

        print(
            f"Created test '{test.name}' with emissions at {test.emissions_file}. "
            # f"Added log '{log.filename}' ({log.size} bytes, type={log.log_type.value})."
        )

    
if __name__ == "__main__":
    main()
