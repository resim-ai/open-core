# Copyright 2024 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from typing import Any

from plotly.colors import qualitative
from plotly.graph_objects import Figure
from resim_python_client.models.conflated_job_status import ConflatedJobStatus

RESIM_BLUE = "#93B1FE"
RESIM_RED = "#FCA6A6"
RESIM_YELLOW = "#FCF39E"
RESIM_PINK = "#FCB5EC"
RESIM_GREEN = "#D3FAA3"
RESIM_PURPLE = "#D6B9FC"
RESIM_ORANGE = "#FFC195"
RESIM_TURQUOISE = "#ACFCEE"
ERROR_RED = "#AD4143"  # roughly equivalent to #CE4747 80% on #3A3A44
PASSED_BLUE = "#7CA0FB"
WARNING_ORANGE = "#FFAC70"
GRIDCOLOR_GRAY = "#3A3A44"
BLOCKING_RED = "#FA7070"


def resim_plotly_style(fig: Figure, **kwargs: Any) -> None:
    fig.update_layout(
        template="plotly_dark",
        plot_bgcolor="rgba(0, 0, 0, 0)",
        paper_bgcolor="rgba(0, 0, 0, 0)",
        margin={"l": 0, "r": 0, "t": 0, "b": 0},
        font_family="Foundry Gridnik Medium",
        title_font_family="Foundry Gridnik Medium",
        modebar_orientation="v",
        modebar_remove="lasso",
        modebar_activecolor=RESIM_GREEN,
        yaxis_gridcolor=GRIDCOLOR_GRAY,
        xaxis_gridcolor=GRIDCOLOR_GRAY,
        # Use invisible ticks since ticklabelstandoff doesn't appear to work
        xaxis={
            "ticks": "outside",
            "tickcolor": "rgba(0, 0, 0, 0)",
            "ticklen": 10,
            "tickwidth": 0.1,
        },
        yaxis={
            "ticks": "outside",
            "tickcolor": "rgba(0, 0, 0, 0)",
            "ticklen": 10,
            "tickwidth": 0.1,
        },
        **kwargs,
    )


resim_colors = [
    RESIM_PURPLE,
    RESIM_GREEN,
    RESIM_BLUE,
    RESIM_RED,
    RESIM_YELLOW,
    RESIM_PINK,
    RESIM_ORANGE,
    RESIM_TURQUOISE,
] + qualitative.Alphabet


resim_status_color_map = {
    ConflatedJobStatus.BLOCKER: BLOCKING_RED,
    ConflatedJobStatus.CANCELLED: RESIM_PINK,
    ConflatedJobStatus.ERROR: ERROR_RED,
    ConflatedJobStatus.PASSED: PASSED_BLUE,
    ConflatedJobStatus.QUEUED: GRIDCOLOR_GRAY,
    ConflatedJobStatus.RUNNING: RESIM_GREEN,
    ConflatedJobStatus.WARNING: WARNING_ORANGE,
}
