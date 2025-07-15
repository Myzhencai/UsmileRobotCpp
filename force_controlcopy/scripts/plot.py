import numpy as np

from plotly.offline import init_notebook_mode, iplot
from plotly.subplots import make_subplots
import plotly.graph_objs as go
import plotly.io as pio
import plotly.express as px

pio.templates.default = "plotly_dark"
pio.renderers.default = "browser"
# pio.renderers.default = "vscode"


# Load data from file
timestamp = []
pos_WTref = []
pos_WT = []
pos_WTadj = []
pos_WT_cmd = []
wrench_T_fb = []
wrench_Tr_All = []
with open(
    "/tmp/admittance_controller.log"
) as file_object:  # open file in a with statement
    for line in file_object:  # iterate line by line
        numbers = [
            float(e) for e in line.split()
        ]  # split line and convert string elements into int
        timestamp.append(numbers[0])
        pos_WTref.append(np.array(numbers[1:4]))
        pos_WT.append(np.array(numbers[4:7]))
        pos_WTadj.append(np.array(numbers[7:10]))
        pos_WT_cmd.append(np.array(numbers[10:13]))
        wrench_T_fb.append(np.array(numbers[13:19]))
        wrench_Tr_All.append(np.array(numbers[19:25]))

timestamp = np.array(timestamp)
pos_WTref = np.array(pos_WTref)
pos_WT = np.array(pos_WT)
pos_WTadj = np.array(pos_WTadj)
pos_WT_cmd = np.array(pos_WT_cmd)
wrench_T_fb = np.array(wrench_T_fb)
wrench_Tr_All = np.array(wrench_Tr_All)

fig = make_subplots(
    rows=9,
    cols=1,
    shared_xaxes="all",
    subplot_titles=("X", "Y", "Z", "w1", "w2", "w3", "w4", "w5", "w6"),
)
marker = dict(
    size=3,
    line=dict(width=1),
    opacity=0.5,
)
# fmt: off
fig.add_trace(go.Scatter(x=timestamp, y=pos_WTref[:,0], name='pos_WTref0'),row=1, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WTref[:,1], name='pos_WTref1'),row=2, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WTref[:,2], name='pos_WTref2'),row=3, col=1)

fig.add_trace(go.Scatter(x=timestamp, y=pos_WT[:,0], name='pos_WT0'),row=1, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WT[:,1], name='pos_WT1'),row=2, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WT[:,2], name='pos_WT2'),row=3, col=1)

fig.add_trace(go.Scatter(x=timestamp, y=pos_WTadj[:,0], name='pos_WTadj0'),row=1, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WTadj[:,1], name='pos_WTadj1'),row=2, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WTadj[:,2], name='pos_WTadj2'),row=3, col=1)

fig.add_trace(go.Scatter(x=timestamp, y=pos_WT_cmd[:,0], name='pos_WT_cmd0'),row=1, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WT_cmd[:,1], name='pos_WT_cmd1'),row=2, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=pos_WT_cmd[:,2], name='pos_WT_cmd2'),row=3, col=1)


fig.add_trace(go.Scatter(x=timestamp, y=wrench_T_fb[:,0], name='wrench_T_fb0'),row=4, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=wrench_T_fb[:,1], name='wrench_T_fb1'),row=5, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=wrench_T_fb[:,2], name='wrench_T_fb2'),row=6, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=wrench_T_fb[:,3], name='wrench_T_fb3'),row=7, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=wrench_T_fb[:,4], name='wrench_T_fb4'),row=8, col=1)
fig.add_trace(go.Scatter(x=timestamp, y=wrench_T_fb[:,5], name='wrench_T_fb5'),row=9, col=1)

# fmt: on
fig.update_layout(height=1200, width=800, title_text="Force Control")
fig.update_layout(hovermode="x unified")

fig.show()
