"""Side-by-side replay of saved numerical samples; no animation physics."""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR',str(Path(__file__).parent/'.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from flight_control import angles
from flight_dynamics import Quad, rotation


def export(datasets, output):
    output = Path(output)
    fig,axes = plt.subplots(4,2,figsize=(15,12),gridspec_kw={'height_ratios':[1.2,1,1,1]})
    fig.patch.set_facecolor('#f4f7fb')
    fig.subplots_adjust(left=.075,right=.975,bottom=.12,top=.84,hspace=.7,wspace=.25)
    fig.suptitle('DRONE STABILIZATION / FEEDBACK COMPARISON',x=.075,ha='left',y=.98,fontsize=19,fontweight='bold',color='#142d4e')
    fig.text(.075,.935,'0–10 s hover  •  10–14 s: 10° roll request  •  14 s: level request  •  17–17.15 s: torque pulse',fontsize=10)
    fig.text(.075,.904,'9 in props • 900 KV • 7000 or 10000 mAh • provisional 2.2 kg. Same initial state and disturbance. Playback: 2× speed.',fontsize=10)
    fig.text(.075,.03,'SIMULATION ONLY • Provisional parameters • Perfect state feedback • No ground contact\nAttitude views are centered on the aircraft. Angle/height axes use different scales. Tilt causes sideways drift; position is not held.',fontsize=10,color='#4a5d73')
    colors = ['#087f8c','#ed7e32','#6c58a6','#cf4770']
    # Keep both panels comparable while making small control corrections visible.
    thrust_min = min(float(data['state'][:,13:17].min()) for data in datasets.values())
    thrust_max = max(float(data['state'][:,13:17].max()) for data in datasets.values())
    thrust_padding = max((thrust_max-thrust_min)*.15, .05)
    for col,(name,data) in enumerate(datasets.items()):
        t,s = data['time'],data['state']; e=angles(s)
        top = axes[0,col]
        top.set_xlim(-.3,.3);top.set_ylim(-.25,.25);top.set_aspect('equal');top.axis('off')
        top.set_title('FEEDBACK CONTROL' if col==0 else 'FIXED HOVER THROTTLE',fontsize=13,fontweight='bold',color='#142d4e')
        axes[1,col].plot(t,data['target'],color='#142d4e',ls='--',label='Roll target')
        axes[1,col].plot(t,np.degrees(np.unwrap(np.radians(e[:,0]))),color=colors[0],label='Roll')
        axes[1,col].plot(t,e[:,1],color=colors[1],label='Pitch (target 0°)')
        axes[1,col].plot(t,e[:,2],color=colors[2],ls=':',label='Yaw (target 0°)')
        axes[1,col].set_ylabel('Angle (deg)')
        axes[1,col].legend(fontsize=7,ncol=2,loc='upper left')
        for j,label in enumerate(['FR','RR','RL','FL']):
            axes[2,col].plot(t,s[:,13+j],color=colors[j],label=label,
                             lw=2,ls=['-','--',':','-.'][j])
        axes[2,col].set_ylabel('Motor thrust (N)')
        axes[2,col].set_ylim(thrust_min-thrust_padding,thrust_max+thrust_padding)
        axes[2,col].ticklabel_format(axis='y',style='plain',useOffset=False)
        axes[2,col].legend(fontsize=9,ncol=4,loc='lower left',
                           bbox_to_anchor=(0,1.02),borderaxespad=0,frameon=False)
        if np.allclose(s[:,13:17],s[:,13:14],rtol=0,atol=1e-9):
            axes[2,col].text(.98,.92,'All four motors overlap',
                             transform=axes[2,col].transAxes,ha='right',va='top',
                             fontsize=9,color='#4a5d73')
        axes[3,col].plot(t,-s[:,2],color=colors[0],label='Height')
        axes[3,col].axhline(0,ls='--',color='#142d4e',lw=1,label='Target/start height')
        axes[3,col].set_ylabel('Height (m)');axes[3,col].set_xlabel('Simulation time (s)')
        for row in [1,2,3]:
            a=axes[row,col];a.set_xlim(0,22);a.grid(alpha=.2);a.axvspan(17,17.15,color='#ed7e32',alpha=.25)
    fig.canvas.draw()
    fig.savefig(output/'comparison_charts.png',dpi=150)
    fig.set_dpi(100);fig.canvas.draw()
    base = Image.fromarray(np.asarray(fig.canvas.buffer_rgba()).copy()).convert('RGB')
    width,height=base.size
    try: font=ImageFont.truetype('DejaVuSans.ttf',13)
    except OSError: font=ImageFont.load_default(size=13)
    motor_positions=Quad().positions
    frames=[]
    times=np.linspace(0,22,111)
    for t in times:
        frame=base.copy();draw=ImageDraw.Draw(frame)
        for col,data in enumerate(datasets.values()):
            index=min(np.searchsorted(data['time'],t),len(data['time'])-1)
            s=data['state'][index]
            # Front view: body/world east right; world up positive on page.
            motors=motor_positions@rotation(s[6:10]).T
            xy=axes[0,col].transData.transform(np.column_stack((motors[:,1],-motors[:,2])))
            xy=[(float(x),float(height-y)) for x,y in xy]
            for pair,color in [([0,2],'#087f8c'),([1,3],'#ed7e32')]:
                draw.line([xy[i] for i in pair],fill=color,width=6)
            for x,y in xy: draw.ellipse((x-7,y-7,x+7,y+7),fill='#142d4e')
            box=axes[0,col].get_window_extent()
            draw.text((box.x0,height-box.y0-15),'Roll view (east/up), aircraft-centered',fill='#4a5d73',font=font)
            for row in [1,2,3]:
                a=axes[row,col];box=a.get_window_extent();x=a.transData.transform((t,0))[0]
                draw.line([(x,height-box.y0),(x,height-box.y1)],fill='#8393a5',width=1)
            x=width*(.075+.49*col)
            draw.text((x,height*.325),f'Height {-s[2]:+.3f} m | horizontal travel {np.linalg.norm(s[:2]):.2f} m',fill='#142d4e',font=font)
        phase = 'HOVER' if t<10 else 'ROLL REQUEST' if t<14 else 'RETURN TO LEVEL' if t<17 else 'DISTURBANCE' if t<17.15 else 'RECOVERY'
        draw.text((width*.075,height*.108),f't = {t:04.1f} s   /   {phase}',fill='#142d4e',font=font)
        frames.append(frame.convert('P',palette=Image.Palette.WEB,dither=Image.Dither.NONE))
    frame.save(output/'comparison.png')
    frames[0].save(output/'comparison.gif',save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=False)
    plt.close(fig)
    print(f'Visuals saved to {output.resolve()}')


if __name__ == '__main__':
    folder=Path(__file__).parent/'outputs/control'
    datasets={name:dict(np.load(folder/f'{name}.npz')) for name in ['controlled','uncontrolled']}
    export(datasets,folder)
