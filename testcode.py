"""

Meaningless test code

"""

import plotly.graph_objects as go
import pandas as pd
import numpy as np
import plotly.io as pio
pio.renderers.default = "browser"

df = pd.read_csv('last_test.csv')

fig = go.Figure(data=go.Scatter3d(
    x=df["X Position"], y=df["Y Position"], z=df["Z Position"],
    marker=dict(
        size=1,
        color=df["Time"],
        colorscale='thermal',
    ),
    line=dict(
        color='darkblue',
        width=2
    )
))

fig.update_layout(
    width=800,
    height=700,
    autosize=False,
    scene=dict(
        camera=dict(
            up=dict(
                x=0,
                y=0,
                z=1
            ),
            eye=dict(
                x=0,
                y=1.0707,
                z=1,
            )
        ),
        aspectratio = dict( x=1, y=1, z=0.7 ),
        aspectmode = 'manual'
    ),
)

fig.show()