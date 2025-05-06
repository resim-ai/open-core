from pathlib import Path
import uuid
from resim.metrics.python.metrics_writer import ResimMetricsWriter
from resim.metrics.python.metrics_utils import MetricImportance, MetricStatus
from resim.metrics.python.plotly_helpers import create_state_timeline_chart


def add_state_timeline_metric(writer: ResimMetricsWriter) -> None:
    system_states = {
        "AUTONOMY_MODE": [
            "False",
            "False",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
            "True",
        ],
        "CHANGING_LANES": [
            "False",
            "False",
            "False",
            "False",
            "False",
            "False",
            "False",
            "False",
            "False",
            "False",
            "True",
            "True",
            "True",
            "False",
            "False",
            "False",
        ],
    }
    timestamps = [
        1743814243602717674,
        1743814244102717674,
        1743814244602717674,
        1743814245102717674,
        1743814245602717674,
        1743814246102717674,
        1743814246602717674,
        1743814247102717674,
        1743814247602717674,
        1743814248102717674,
        1743814248602717674,
        1743814249102717674,
        1743814249602717674,
        1743814250102717674,
        1743814250602717674,
        1743814251102717674,
    ]

    plotly_json = create_state_timeline_chart(system_states, timestamps)
    assert plotly_json is not None, "Failed to create plotly json for system states"

    (
        writer.add_plotly_metric(name="System States")
        .with_description("System states of interest over time")
        .with_status(MetricStatus.PASSED_METRIC_STATUS)
        .with_importance(MetricImportance.HIGH_IMPORTANCE)
        .with_should_display(True)
        .with_blocking(True)
        .with_plotly_data(plotly_json)
    )


def main() -> None:
    writer = ResimMetricsWriter(job_id=uuid.uuid4())

    add_state_timeline_metric(writer)

    metrics_proto = writer.write()
    out_path = Path("./demo_metrics.binproto").absolute()
    with out_path.open("wb") as f:
        f.write(metrics_proto.metrics_msg.SerializeToString())

    print(f"Example metrics written to {out_path}")


if __name__ == "__main__":
    main()
