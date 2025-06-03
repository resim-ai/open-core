# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import itertools
import logging
import pandas as pd
import plotly.graph_objects as go
from typing import Optional

from resim.metrics.resim_style import resim_colors, resim_plotly_style, GRIDCOLOR_GRAY

logger = logging.getLogger(__name__)


def create_state_timeline_chart(
    system_states: dict[str, list[str]],
    timestamps: list[int],
    *,
    colors: Optional[dict[str, str]] = None,
    xaxis_title: str = "Elapsed time (s)",
) -> go.Figure | None:
    """Create a state timeline chart.

    Args:
        system_states: A dictionary mapping system names to their list of states
        timestamps: List of timestamps in nanoseconds
        colors: Optional dictionary mapping states to their colors
        xaxis_title: Optional title for the x-axis

    Returns:
        A plotly figure, or None if no data
    """
    if not timestamps:
        logger.warning("No timestamps found for state timeline chart")
        return None

    # Get all unique states
    all_states: set[str] = set()
    for states in system_states.values():
        all_states.update(states)

    start_time_secs = timestamps[0] / 1e9
    elapsed_times = []
    for ts in timestamps:
        seconds = ts / 1e9 - start_time_secs
        elapsed_times.append(seconds)

    df_list = []
    for system_name, states in system_states.items():
        if not states:
            continue

        current_state = states[0]
        start_idx = 0
        for j in range(1, len(states)):
            if states[j] != current_state:
                df_list.append(
                    {
                        "System": system_name,
                        "State": current_state,
                        "Start": elapsed_times[start_idx],
                        "End": elapsed_times[j],
                    }
                )
                current_state = states[j]
                start_idx = j

        # Add the final state
        if start_idx < len(elapsed_times):
            df_list.append(
                {
                    "System": system_name,
                    "State": current_state,
                    "Start": elapsed_times[start_idx],
                    "End": elapsed_times[-1],
                }
            )

    if not df_list:
        logger.warning("No state changes found for timeline chart")
        return None

    df = pd.DataFrame(df_list)

    # Create the figure
    fig = go.Figure()

    # Define colors for all states
    colors = {
        state: color
        for state, color in zip(sorted(all_states), itertools.cycle(resim_colors))
    }

    already_on_legend = set()

    # Add a trace for each system and state
    for system in df["System"].unique():
        system_df = df[df["System"] == system]

        for state in all_states:
            state_df = system_df[system_df["State"] == state]

            for i, row in state_df.iterrows():
                fig.add_trace(
                    go.Bar(
                        x=[row["End"] - row["Start"]],
                        y=[system],
                        orientation="h",
                        base=[row["Start"]],
                        marker_color=colors[state],
                        name=state,
                        showlegend=True if state not in already_on_legend else False,
                        legendgroup=state,
                        hovertemplate=f"<b style='color:{colors[state]};'>{state}</b>: {format(row['Start'], '.3f')} - {format(row['End'], '.3f')}",
                        hoverlabel=dict(namelength=0),
                    )
                )
                already_on_legend.add(state)

    # Update layout
    fig.update_layout(
        xaxis_title=xaxis_title,
        barmode="stack",
        hovermode="closest",
        hoverlabel=dict(bgcolor=GRIDCOLOR_GRAY),
        showlegend=True,
        legend=dict(orientation="h", y=1.0, x=0.5, xanchor="center", yanchor="bottom"),
    )
    fig.update_xaxes(rangemode="tozero")
    resim_plotly_style(fig)

    return fig
