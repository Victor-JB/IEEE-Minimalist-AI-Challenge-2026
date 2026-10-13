"""Finger-ray targeting with a camera-guided pointer for the simulated desk.

A monocular finger ray frequently misses the small desk at an oblique camera angle.
When that happens, fingertip image position controls an explicitly virtual desk
cursor. Neither mode drives hardware. Digital IK uses all five model joints.
"""
import math
from tracking import SubjectTracker
from kinematics import fk, beam_hit, wrist_ik, HEAD_PIVOT, LIGHT_OFFSET, SHADE_CLEARANCE, SHADE_DEPTH


def simulated_aim(target,closer=0.):
    """Turn the digital base toward the spot, then aim the shade from a raised wrist.

    The head link, light offset and beam share an axis at zero roll, so aiming
    wrist -> target also puts the light axis exactly through target. This avoids
    the broken physical arm's narrow pitch/roll limits and leaves room below the shade.
    """
    if len(target)!=3 or not all(math.isfinite(v) for v in target):return None
    if target[2]!=0 or not (.20<=target[0]<=.65 and abs(target[1])<=.30):return None
    yaw=math.degrees(math.atan2(target[1],target[0]))
    radius=math.hypot(*target[:2]);q=[20.,55.,0.]
    wrist=fk(q).wrist
    phi=math.degrees(math.atan2(radius-wrist[0],-wrist[2]))
    if phi<0:phi+=360
    q[2]=phi-q[0]-q[1]
    pose=[yaw,*q,0.]
    return adjust_pose(pose,target,closer) if closer else pose


def _finger_target(p,joints,pivot,width,height):
    if len(joints)!=3 or any(not j.get('healthy') or not isinstance(j.get('angle'),(int,float)) or not math.isfinite(j['angle']) for j in joints):
        return None,'Physical camera pose unavailable'
    span=math.dist(p[5][:2],p[17][:2])
    if span<12:return None,'Palm too small for depth'
    f=width/(2*math.tan(math.radians(SubjectTracker.HFOV)/2));scale=.08/span;depth=f*scale
    if not .15<depth<3:return None,'Palm depth outside range'
    midz=(p[5][2]+p[17][2])/2
    roll=pivot.get('current',0) if pivot.get('enabled') and not pivot.get('fault') else 0
    if not isinstance(roll,(int,float)) or not math.isfinite(roll):return None,'Camera swivel unavailable'
    h=fk([j['angle'] for j in joints],roll)
    def world(v):
        z=max(.1,depth+(v[2]-midz)*scale);x=(v[0]-width/2)*z/f;y=-(v[1]-height/2)*z/f
        return [h.camera[i]+x*h.right[i]+y*h.up[i]+z*h.axis[i] for i in range(3)]
    base,tip=world(p[6]),world(p[8]);v=[tip[i]-base[i] for i in range(3)];norm=math.hypot(*v)
    if norm<.01 or v[2]/norm>=-.15:return None,'Finger ray is not pointing down toward the desk'
    t=-tip[2]/v[2]
    if t<0 or t*norm>1.5:return None,'Finger ray does not meet the nearby desk'
    hit=[tip[i]+t*v[i] for i in range(3)];hit[2]=0.
    if not (.20<=hit[0]<=.65 and abs(hit[1])<=.30):return None,'Finger ray falls outside the simulated desk'
    return hit,None


def target_from_hand(hand,joints,pivot,width=640,height=480,allow_virtual=True,prefer_virtual=False):
    p=hand.get('points',[])
    if width<=0 or height<=0 or len(p)!=21 or any(len(v)!=3 or not all(math.isfinite(c) for c in v) for v in p):return None
    # Only the palm and index need to be fully visible for a virtual cursor.
    if any(not (0<p[i][0]<width and 0<p[i][1]<height) for i in (0,5,6,8,9,17)):return None
    hit,reason=_finger_target(p,joints,pivot,width,height)
    # Once a drag falls back to image control, keep that mapping until release.
    # Alternating between unrelated ray/cursor coordinates makes the light jump.
    if prefer_virtual and allow_virtual:hit=None;reason='Keeping the virtual pointer during this drag'
    mode='finger_ray';source='Estimated 3D finger ray'
    if hit is None:
        if not allow_virtual:return None
        # Image right maps to robot right (-y); image top maps to farther away.
        u=max(0,min(1,(p[8][0]/width-.1)/.8));v=max(0,min(1,(p[8][1]/height-.1)/.8))
        hit=[.25+.40*(1-v),.30-.60*u,0.]
        mode='virtual_pointer';source='Virtual desk pointer · fingertip position in camera image'
    pose=simulated_aim(hit)
    if pose is None:return None
    return dict(point=hit,pose=pose,source=source,mode=mode,ray_note=reason,
                image_tip=[p[8][0]/width,p[8][1]/height])


def physical_aim(joints,target):
    """Legacy geometry helper; the hybrid demo never uses it to point hardware."""
    if not target or len(joints)!=3 or any(j.get('angle') is None for j in joints):return None
    q=[j['angle'] for j in joints];h=fk(q)
    dx=target[0]-h.pivot[0];dz=-h.pivot[2]
    roll=math.degrees(math.atan2(-(target[1]-h.pivot[1]),math.hypot(dx,dz)))
    roll=max(-45,min(45,roll));hit=beam_hit(q,roll)
    if hit is None or math.dist(hit,target)>.18:return None
    return roll


def adjust_pose(pose,target,delta):
    """Move the digital wrist along the beam while keeping its yaw and target."""
    h=fk(pose[1:4],pose[4],pose[0]);distance=math.dist(h.light,target)
    desired=max(SHADE_DEPTH-LIGHT_OFFSET+SHADE_CLEARANCE,distance-delta)
    def solve(standoff):
        pivot=[target[i]-(standoff+LIGHT_OFFSET)*h.axis[i] for i in range(3)]
        wrist=[pivot[i]-HEAD_PIVOT*h.axis[i] for i in range(3)]
        yaw=math.radians(pose[0]);radial=wrist[0]*math.cos(yaw)+wrist[1]*math.sin(yaw)
        for sh,el in wrist_ik(radial,wrist[2]):
            wr=h.phi-sh-el
            if -90<=sh<=90 and 0<=el<=135 and -125<=wr<=135:
                return [pose[0],sh,el,wr,0.]
        return None
    result=solve(desired)
    if result is not None:return result
    # Saturate at the reachable boundary. Returning the first halved step can
    # reverse a held Close/Far gesture as the requested offset grows.
    low,high=0.,1.;result=list(pose)
    for _ in range(24):
        fraction=(low+high)/2
        candidate=solve(distance+(desired-distance)*fraction)
        if candidate is None:high=fraction
        else:low=fraction;result=candidate
    return result
