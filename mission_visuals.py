"""Plot and replay recorded SITL telemetry; no invented or interpolated physics."""
import csv
import json
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR',str(Path(__file__).parent/'.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def export(output):
    output = Path(output)
    with (output/'telemetry.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return
    summary = json.loads((output/'summary.json').read_text())
    t = np.array([float(row['time_s']) for row in rows])
    position = np.array([[float(row[key]) for key in ('north_m','east_m','down_m')] for row in rows])
    origin = summary.get('origin_ned') or position[0].tolist()
    relative = position-np.array(origin)
    height = -relative[:,2]
    speed = np.array([np.linalg.norm([float(row[key]) for key in ('vn_mps','ve_mps','vd_mps')]) for row in rows])
    phases = [row['phase'] for row in rows]
    fig = plt.figure(figsize=(12,7),facecolor='#f4f7fb')
    grid = fig.add_gridspec(2,2,left=.08,right=.96,top=.79,bottom=.17,wspace=.32,hspace=.4)
    path_ax = fig.add_subplot(grid[:,0])
    height_ax = fig.add_subplot(grid[0,1])
    speed_ax = fig.add_subplot(grid[1,1])
    passed = summary['passed']
    fig.suptitle('SITL MISSION  /  '+('PASS' if passed else 'INCOMPLETE'),x=.08,y=.96,ha='left',fontsize=20,fontweight='bold',color='#142d4e')
    fig.text(.08,.88,'ArduCopter 4.6.3 • simulated quad • recorded MAVLink telemetry',fontsize=11,color='#4a5d73')
    fig.text(.08,.055,'SIMULATION ONLY • Default SITL aircraft; not the calibrated project airframe.\nReplay is accelerated. Displayed times are measured wall-clock seconds.',fontsize=10,color='#4a5d73')
    path_ax.plot(relative[:,1],relative[:,0],color='#087f8c',lw=2,label='Recorded path')
    path_ax.scatter([0],[0],color='#142d4e',marker='o',s=45,label='Takeoff')
    plan = summary['plan']
    path_ax.scatter([plan['east']],[plan['north']],color='#ec7c35',marker='x',s=70,label='Waypoint')
    path_ax.set(xlabel='East from takeoff (m)',ylabel='North from takeoff (m)',title='Top-down flight path')
    path_ax.set_aspect('equal',adjustable='datalim')
    path_ax.margins(.25)
    path_ax.legend(loc='best',fontsize=8)
    height_ax.plot(t,height,color='#087f8c')
    height_ax.axhline(plan['altitude'],color='#ec7c35',ls='--',lw=1)
    height_ax.set(ylabel='Height above start (m)',xlabel='Elapsed time (s)')
    speed_ax.plot(t,speed,color='#087f8c')
    speed_ax.set(ylabel='Speed (m/s)',xlabel='Elapsed time (s)')
    for axis in [path_ax,height_ax,speed_ax]: axis.grid(alpha=.2)
    fig.savefig(output/'mission.png',dpi=150)
    fig.set_dpi(100)
    fig.canvas.draw()
    base = Image.fromarray(np.asarray(fig.canvas.buffer_rgba()).copy()).convert('RGB')
    width,h = base.size
    try: font=ImageFont.truetype('DejaVuSans.ttf',14)
    except OSError: font=ImageFont.load_default(size=14)
    frames=[]
    indices=np.unique(np.linspace(0,len(t)-1,min(80,len(t))).astype(int))
    for index in indices:
        frame=base.copy()
        draw=ImageDraw.Draw(frame)
        x,y=path_ax.transData.transform((relative[index,1],relative[index,0]))
        y=h-y
        draw.line([(x-10,y-10),(x+10,y+10)],fill='#142d4e',width=4)
        draw.line([(x-10,y+10),(x+10,y-10)],fill='#ec7c35',width=4)
        for axis,value in [(height_ax,height[index]),(speed_ax,speed[index])]:
            x,y=axis.transData.transform((t[index],value)); box=axis.get_window_extent()
            draw.line([(x,h-box.y0),(x,h-box.y1)],fill='#8393a5',width=1)
            draw.ellipse((x-4,h-y-4,x+4,h-y+4),fill='#ec7c35')
        label = phases[index] if index != indices[-1] else ('END / LANDED & DISARMED' if passed else 'END / INCOMPLETE')
        draw.text((width*.08,h*.115),f't = {t[index]:.1f} s  |  {label}  |  height {height[index]:.2f} m',fill='#142d4e',font=font)
        frames.append(frame.quantize(colors=256))
    frames[0].save(output/'mission.gif',save_all=True,append_images=frames[1:],duration=120,disposal=2,optimize=False)
    plt.close(fig)
    print(f'Mission chart and replay: {output}',flush=True)


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    export(parser.parse_args().output)
