"""Construct an original low-poly Roadster-inspired glTF with embedded buffers.
No Tesla CAD, textures or third-party vehicle meshes are included.
"""
import base64
import json
import math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
gltf = {'asset':{'version':'2.0','generator':'Roadster Flight Lab / original reference-inspired geometry',
                  'copyright':'Original procedural model; not official Tesla CAD'},
        'scene':0,'scenes':[{'nodes':[]}],'nodes':[],'meshes':[],'materials':[], 'buffers':[], 'bufferViews':[], 'accessors':[]}
data = bytearray()
def material(name, color, metal=0., rough=.4, emission=None):
    value={'name':name,'doubleSided':True,'pbrMetallicRoughness':{'baseColorFactor':color,'metallicFactor':metal,'roughnessFactor':rough}}
    if emission:value['emissiveFactor']=emission
    gltf['materials'].append(value);return len(gltf['materials'])-1
white=material('Pearl ceramic white',[.89,.91,.90,1],.45,.26)
glass=material('Smoked panoramic glass',[.016,.035,.042,1],.65,.14)
black=material('Carbon graphite',[.025,.029,.032,1],.4,.35)
rubber=material('Satin tire',[.018,.02,.023,1],0,.78)
metal=material('Machined titanium',[.45,.49,.51,1],.85,.2)
light=material('Ice white running light',[.8,.94,1,1],0,.18,[.8,.94,1])
red=material('Rear light',[.5,.005,.002,1],.2,.2,[.8,.012,.006])
nozzle=material('Nozzle interior',[.008,.012,.014,1],.65,.45)

def accessor(values, typ):
    a=np.asarray(values,dtype='<f4'); start=len(data);data.extend(a.tobytes())
    gltf['bufferViews'].append({'buffer':0,'byteOffset':start,'byteLength':a.nbytes,'target':34962})
    item={'bufferView':len(gltf['bufferViews'])-1,'componentType':5126,'count':len(a),'type':typ}
    if typ=='VEC3':item.update(min=a.min(axis=0).tolist(),max=a.max(axis=0).tolist())
    gltf['accessors'].append(item);return len(gltf['accessors'])-1

def mesh(name, triangles, mat, position=None):
    tris=np.array(triangles,dtype=float).reshape(-1,3,3)
    tris=tris[np.linalg.norm(np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]),axis=1)>1e-12]
    vertices=tris.reshape(-1,3)
    normals=[]
    for tri in vertices.reshape(-1,3,3):
        n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n/=max(np.linalg.norm(n),1e-12)
        normals.extend([n]*3)
    primitive={'attributes':{'POSITION':accessor(vertices,'VEC3'),'NORMAL':accessor(normals,'VEC3')},'material':mat}
    gltf['meshes'].append({'name':name,'primitives':[primitive]})
    node={'name':name,'mesh':len(gltf['meshes'])-1}
    if position is not None:node['translation']=position
    gltf['nodes'].append(node);gltf['scenes'][0]['nodes'].append(len(gltf['nodes'])-1)
    return len(gltf['nodes'])-1

def quad(a,b,c,d):return [a,b,c,a,c,d]
def box(name,center,size,mat):
    c=np.array(center);s=np.array(size)/2
    v=[(c+np.array([x,y,z])*s).tolist() for x,y,z in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
    out=[]
    for a,b,c,d in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:out+=quad(v[a],v[b],v[c],v[d])
    mesh(name,out,mat)

def loft(name, sections, mat):
    # Counterclockwise cross sections, viewed from +X.
    rings=[]
    for x,w,low,shoulder,top in sections:
        rings.append([[x,-w,low],[x,w,low],[x,w,shoulder],[x,w*.75,top],[x,-w*.75,top],[x,-w,shoulder]])
    out=[]
    for a,b in zip(rings[:-1],rings[1:]):
        for j in range(6):out+=quad(a[j],a[(j+1)%6],b[(j+1)%6],b[j])
    for ring,reverse in [(rings[0],True),(rings[-1],False)]:
        for i in range(1,5):out+=([ring[0],ring[i+1],ring[i]] if reverse else [ring[0],ring[i],ring[i+1]])
    mesh(name,out,mat)

loft('Faceted monocoque', [(-2.22,.72,-.30,.10,.18),(-1.7,.96,-.34,.24,.31),(-.7,.94,-.34,.25,.32),(.8,.90,-.32,.14,.23),(1.7,.91,-.25,.05,.16),(2.24,.77,-.19,-.02,.06)],white)
box('Carbon undertray',[0,0,-.35],[3.95,1.66,.12],black)
# Cabin tapers toward the windscreen, like the reference coupe.
loft('Glass canopy',[(-1.38,.74,.23,.29,.34),(-.86,.68,.27,.61,.67),(.35,.64,.22,.68,.72),(1.14,.75,.16,.18,.22)],glass)
# Roof rails, windscreen pillars and side sill geometry.
for side in [-1,1]:
    mesh('White roof rail '+str(side),quad([-.98,side*.69,.63],[.34,side*.67,.74],[.43,side*.63,.73],[-.88,side*.63,.65]),white)
    mesh('A pillar '+str(side),quad([.30,side*.66,.73],[1.19,side*.78,.20],[1.08,side*.78,.17],[.22,side*.64,.70]),white)
    mesh('Rear sail '+str(side),quad([-1.65,side*.83,.26],[-.88,side*.69,.66],[-.75,side*.73,.63],[-1.33,side*.87,.25]),white)
    box('Sill '+str(side),[-.05,side*.94,-.27],[2.4,.07,.13],black)
    box('Door handle '+str(side),[-.5,side*.947,.15],[.25,.018,.026],black)
    box('Mirror stalk '+str(side),[.88,side*.89,.24],[.12,.19,.025],black)
    box('Wing mirror '+str(side),[.85,side*1.02,.25],[.24,.15,.085],white)
    # Front light in a wraparound angular band.
    mesh('Headlight '+str(side),quad([2.245,side*.72,.018],[2.25,side*.25,.024],[2.17,side*.25,.055],[2.16,side*.75,.052]),light)
    box('Tail strip '+str(side),[-2.225,side*.43,.12],[.018,.60,.035],red)
box('Front dark intake',[2.22,0,-.095],[.04,1.22,.13],black)
box('Rear diffuser',[-2.22,0,-.19],[.08,1.47,.22],black)
box('Active rear wing',[-1.94,0,.49],[.38,1.85,.045],black)
for side in [-1,1]:box('Wing mount '+str(side),[-1.93,side*.62,.34],[.08,.055,.3],black)

# Tires are annular, not solid cylinders: silver rims, dark barrels, five spokes.
def cylinder(name,center,radius,length,mat,axis='y',inner=0,segments=48):
    out=[]
    def pt(a,r,w):
        return ([r*math.cos(a),w,r*math.sin(a)] if axis=='y' else [r*math.cos(a),r*math.sin(a),w])
    for i in range(segments):
        a,b=i*2*math.pi/segments,(i+1)*2*math.pi/segments
        aa,ab,ba,bb=pt(a,radius,-length/2),pt(a,radius,length/2),pt(b,radius,-length/2),pt(b,radius,length/2)
        out+=quad(aa,ba,bb,ab)
        for w in [-length/2,length/2]:
            face=quad(pt(a,inner,w),pt(b,inner,w),pt(b,radius,w),pt(a,radius,w))
            if (axis=='y' and w<0) or (axis!='y' and w>0):
                face=np.array(face).reshape(-1,3,3)[:,::-1,:].reshape(-1,3).tolist()
            out+=face
    return mesh(name,out,mat,center)
for x in [-1.4,1.4]:
    for y in [-.92,.92]:
        center=[x,y,-.25]
        suffix=f'{x}_{y}'
        # Child geometry is local to this hub. Explicit node IDs avoid coordinate
        # suffix collisions (e.g. '1.4_0.92' also matches '-1.4_0.92').
        children = [
            cylinder('Tire '+suffix,[0,0,0],.37,.23,rubber,inner=.285),
            cylinder('Rim '+suffix,[0,0,0],.29,.235,metal,inner=.25),
            cylinder('Brake '+suffix,[0,0,0],.24,.19,black),
            cylinder('Hub '+suffix,[0,0,0],.075,.245,metal),
        ]
        out=[]
        w=.124*(1 if y>0 else -1)
        for i in range(5):
            a=i*2*math.pi/5+.2
            pts=[]
            for r,angle in [(.06,a-.25),(.275,a-.08),(.275,a+.07),(.06,a+.3)]:pts.append([r*math.cos(angle),w,r*math.sin(angle)])
            out+=quad(*pts)
        children.append(mesh('Turbine spokes '+suffix,out,metal))
        gltf['scenes'][0]['nodes'] = [i for i in gltf['scenes'][0]['nodes'] if i not in children]
        gltf['nodes'].append({'name':'Wheel axle '+suffix,'translation':center,'children':children})
        gltf['scenes'][0]['nodes'].append(len(gltf['nodes'])-1)
# Six physical nozzles, with fixed directions matching the simulation.
import sys
sys.path.insert(0,str(ROOT.parent/'src'))
from rocket_landing.roadster import NOZZLE_POS,NOZZLE_DIR
for i,(pos,direction) in enumerate(zip(NOZZLE_POS,NOZZLE_DIR)):
    # Rings perpendicular to exhaust direction (-thrust).
    d=-direction; other=np.array([1.,0,0]) if abs(d[0])<.9 else np.array([0.,1,0])
    u=np.cross(d,other);u/=np.linalg.norm(u);v=np.cross(d,u)
    out=[];inside=[]
    for j in range(24):
        a,b=j*2*math.pi/24,(j+1)*2*math.pi/24
        def p(r,a,z):return (pos+z*d+r*(math.cos(a)*u+math.sin(a)*v)).tolist()
        out+=quad(p(.085,a,0),p(.085,b,0),p(.12,b,.11),p(.12,a,.11))
        inside += [pos.tolist(),p(.08,a,.115),p(.08,b,.115)]
    mesh(f'Nozzle_{i:02d}',out,metal);mesh(f'Nozzle throat {i}',inside,nozzle)

while len(data)%4:data.append(0)
gltf['buffers']=[{'byteLength':len(data),'uri':'data:application/octet-stream;base64,'+base64.b64encode(data).decode()}]
path=ROOT/'web/public/assets/roadster.gltf';path.parent.mkdir(parents=True,exist_ok=True)
path.write_text(json.dumps(gltf,separators=(',',':')))
print(f'{path}: {len(gltf["meshes"])} meshes, {len(data):,} geometry bytes')
