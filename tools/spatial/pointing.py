"""Estimated index-finger ray -> desk intersection, using the physical head frame."""
import math
from tracking import SubjectTracker
from kinematics import fk, aim, beam_hit, move_along_beam


def target_from_hand(hand,joints,pivot,width=640,height=480):
    if len(joints)!=3 or any(not j.get('healthy') or j.get('angle') is None for j in joints):return None
    p=hand['points']
    if len(p)!=21 or any(not (0<v[0]<width and 0<v[1]<height) for v in p):return None
    span=math.dist(p[5][:2],p[17][:2])
    if span<12:return None
    f=width/(2*math.tan(math.radians(SubjectTracker.HFOV)/2));scale=.08/span;depth=f*scale
    if not .15<depth<3:return None
    midz=(p[5][2]+p[17][2])/2
    # Released pivot is explicitly the same manual zero assumption as the initial view.
    roll=pivot.get('current',0) if pivot.get('enabled') and not pivot.get('fault') else 0
    h=fk([j['angle'] for j in joints],roll)
    def world(v):
        z=max(.1,depth+(v[2]-midz)*scale);x=(v[0]-width/2)*z/f;y=-(v[1]-height/2)*z/f
        return [h.camera[i]+x*h.right[i]+y*h.up[i]+z*h.axis[i] for i in range(3)]
    base,tip=world(p[6]),world(p[8]);v=[tip[i]-base[i] for i in range(3)];norm=math.hypot(*v)
    if norm<.01 or v[2]/norm>=-.15:return None
    t=-tip[2]/v[2]
    if t<0 or t*norm>1.5:return None
    hit=[tip[i]+t*v[i] for i in range(3)]
    if not (.20<=hit[0]<=.65 and abs(hit[1])<=.30):return None
    pose=aim(hit)
    if pose is None:return None
    return dict(point=hit,pose=[0,*pose.q,pose.roll],source='estimated finger ray on desk · 100° FOV / 8 cm palm')


def physical_aim(joints,target):
    """Only pivot can move. Return its bounded angle and reachable beam hit."""
    if not target or len(joints)!=3 or any(j.get('angle') is None for j in joints):return None
    q=[j['angle'] for j in joints];h=fk(q)
    dx=target[0]-h.pivot[0];dz=-h.pivot[2]
    roll=math.degrees(math.atan2(-(target[1]-h.pivot[1]),math.hypot(dx,dz)))
    roll=max(-45,min(45,roll));hit=beam_hit(q,roll)
    if hit is None or math.dist(hit,target)>.18:return None
    return roll


def adjust_pose(pose,target,delta):
    adjusted,_=move_along_beam(pose[1:4],pose[4],target,delta)
    return [0,*adjusted.q,adjusted.roll]
