"""Shareable plots and GIF from actual rigid-body simulation samples."""
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).parent / '.mplconfig'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from flight_dynamics import rotation


def export_visuals(rows, quad, scenario, output):
    data = np.asarray(rows)
    t, states = data[:,0], data[:,1:]
    # Display north/east/up; model and CSV retain NED.
    centers = states[:,:3] * [1,1,-1]
    matrices = np.array([rotation(q) for q in states[:,6:10]])
    roll = np.degrees(np.arctan2(matrices[:,2,1],matrices[:,2,2]))
    pitch = np.degrees(np.arcsin(np.clip(-matrices[:,2,0],-1,1)))
    fig = plt.figure(figsize=(12,7),facecolor='#f4f7fb')
    grid = fig.add_gridspec(3,2,width_ratios=[1.2,1],left=.06,right=.97,bottom=.13,top=.83,wspace=.25,hspace=.55)
    ax = fig.add_subplot(grid[:,0],projection='3d')
    charts = [fig.add_subplot(grid[i,1]) for i in range(3)]
    fig.suptitle('DRONE FLIGHT MODEL  /  '+scenario.upper(),x=.06,ha='left',fontsize=20,fontweight='bold',color='#142d4e')
    fig.text(.06,.88,'Simulated data • 9 in props • 900 KV • 7000 or 10000 mAh • provisional 2.2 kg • no flight-controller feedback',fontsize=11,color='#4a5d73')
    fig.text(.06,.035,'Uncalibrated model — not measured flight data.\nHeight is relative to the starting point; ground contact is not modeled.',fontsize=10,color='#4a5d73')
    ax.plot(*centers.T,color='#b9c9da',lw=2,label='Full trajectory')
    path, = ax.plot([],[],[],color='#087f8c',lw=2)
    arms = [ax.plot([],[],[],lw=4,color=color)[0] for color in ['#087f8c','#ec7c35']]
    rotors, = ax.plot([],[],[],linestyle='',marker='o',ms=8,color='#142d4e')
    center = (centers.min(axis=0)+centers.max(axis=0))/2
    half = max(float(np.ptp(centers,axis=0).max())/2+quad.p.arm*1.5,.4)
    ax.set(xlim=(center[0]-half,center[0]+half),ylim=(center[1]-half,center[1]+half),zlim=(center[2]-half,center[2]+half),xlabel='North (m)',ylabel='East (m)',zlabel='Height (m)')
    ax.set_box_aspect((1,1,1)); ax.view_init(22,-55)
    ax.set_title('Aircraft pose and path',loc='left',fontsize=12)
    charts[0].plot(t,centers[:,2],color='#087f8c',label='Height')
    charts[0].set_ylabel('Height (m)')
    charts[1].plot(t,roll,color='#ec7c35',label='Roll')
    charts[1].plot(t,pitch,color='#087f8c',label='Pitch')
    charts[1].set_ylabel('Angle (deg)'); charts[1].legend(loc='upper left',fontsize=8,ncol=2)
    charts[2].plot(t,states[:,13:].sum(axis=1),color='#087f8c',label='Total thrust')
    charts[2].axhline(quad.p.mass*9.81,color='#ec7c35',ls='--',label='Weight')
    charts[2].set_ylabel('Force (N)'); charts[2].set_xlabel('Simulation time (s)')
    charts[2].set_ylim(0, max(quad.p.mass*9.81, states[:,13:].sum(axis=1).max())*1.15)
    charts[2].legend(loc='upper left',fontsize=8,ncol=2)
    cursors = []
    for chart in charts:
        chart.grid(alpha=.2); chart.set_xlim(0,t[-1]); cursors.append(chart.axvline(0,color='#142d4e',alpha=.65))
    clock = fig.text(.06,.08,'',fontsize=11,color='#142d4e')

    def draw(index):
        motors = (quad.positions @ matrices[index].T + states[index,:3])*[1,1,-1]
        for line,pair in zip(arms,[[0,2],[1,3]]):
            line.set_data_3d(*motors[pair].T)
        rotors.set_data_3d(*motors.T)
        path.set_data_3d(*centers[:index+1].T)
        for cursor in cursors: cursor.set_xdata([t[index],t[index]])
        status = 'END' if index == len(t)-1 else 'PLAYING'
        clock.set_text(f'{scenario.upper()} / {status}   |   t = {t[index]:.2f} s   |   Speed = {np.linalg.norm(states[index,3:6]):.2f} m/s   |   Orange arm: RR–FL')
        return arms+[rotors,path,clock]+cursors

    output = Path(output)
    draw(len(t)-1)
    png = output/f'{scenario}.png'
    fig.savefig(png,dpi=150,facecolor=fig.get_facecolor())
    indices = np.unique(np.linspace(0,len(t)-1,min(40,len(t))).astype(int))
    frames = []
    fig.set_dpi(80)
    for index in indices:
        draw(index)
        fig.canvas.draw()
        rgb = Image.fromarray(np.asarray(fig.canvas.buffer_rgba()).copy()).convert('RGB')
        frames.append(rgb.convert('P', palette=Image.Palette.WEB, dither=Image.Dither.NONE))
    gif = output/f'{scenario}.gif'
    frames[0].save(gif, save_all=True, append_images=frames[1:], duration=100, disposal=2, optimize=False)
    plt.close(fig)
    return png,gif
