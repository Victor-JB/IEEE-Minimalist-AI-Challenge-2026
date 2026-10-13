"""Extract full-model choreography without importing MuJoCo or executing branch code.

The original show is preserved. Interactive actions select passages from it;
ELEGNT oscillations remain additive layers, at authored amplitudes (the browser
has no stepper speed/acceleration caps). Bundle angles are degrees.
"""
import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
REF = "ef1b0d2057c5e449a0e02638a1ff7d5b58dd3cbd"


def numeric(node, variables):
    """Only numeric literals, lists, names and arithmetic; never execute source."""
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return node.value
    if isinstance(node, (ast.List, ast.Tuple)):
        return [numeric(v, variables) for v in node.elts]
    if isinstance(node, ast.Name):
        return variables[node.id]
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return -numeric(node.operand, variables)
    if isinstance(node, ast.BinOp):
        a, b = numeric(node.left, variables), numeric(node.right, variables)
        if isinstance(node.op, ast.Mult): return a * b
        if isinstance(node.op, ast.Div): return a / b
    raise ValueError(ast.dump(node))


def build():
    sources = {name: subprocess.check_output(
        ["git", "show", f"{REF}:arm_mujoco/{name}.py"], cwd=ROOT)
        for name in ("lamp_demo", "elegnt_demo")}
    tree = ast.parse(sources["lamp_demo"])
    assignments = {n.targets[0].id: n.value for n in tree.body
                   if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)}
    poses = ast.literal_eval(assignments["POSE"])
    show = ast.literal_eval(assignments["SHOW"])
    vmax = ast.literal_eval(assignments["VMAX"].args[0])
    amax = ast.literal_eval(assignments["AMAX"].args[0])

    def timeline(entries):
        q = poses[entries[0][0]]
        start = q
        t = entries[0][1]
        segments = []
        for name, hold, speed, stagger in entries[1:]:
            end = poses[name]
            dq = [abs(math.radians(b-a)) for a, b in zip(q, end)]
            duration = max(.25, *[max(1.875*d/(v*speed), math.sqrt(5.774*d/(a*speed**2)))
                                   for d, v, a in zip(dq, vmax, amax)])
            segments.append(dict(t=t, duration=duration, start=q, end=end,
                                 stagger=stagger, pose=name))
            t += duration + 4*stagger + hold
            q = end
        return dict(start=start, segments=segments, duration=t+.5)

    look = ('look', 0, 1, 0)
    passages = dict(full_show=show, wake=show[:5], greet=[look, show[10], show[2], show[4]],
                    curious=[look, *show[8:10]], peek=[look, *show[10:13]],
                    nod=[look, *show[13:17]], shake=[look, *show[17:21]],
                    joy=[look, *show[21:25]], look_around=[look, *show[5:8]],
                    sleep=[look, *show[25:]], stretch=[*show[:5], *show[25:]])
    clips = {name: timeline(entries) for name, entries in passages.items()}
    show_class = next(n for n in ast.parse(sources["elegnt_demo"]).body
                      if isinstance(n, ast.ClassDef) and n.name == "Show")
    methods = {n.name: n for n in show_class.body if isinstance(n, ast.FunctionDef)}
    overlays = {}
    for name, method in dict(nod='nod', shake='shake', bounce='bounce',
                             breathe='relax', dance='dance', sway='sway').items():
        fn = methods[method]
        variables = {a.arg: numeric(d, {}) for a, d in zip(fn.args.args[-len(fn.args.defaults):], fn.args.defaults)}
        variables.update(bpm=96, beats=4, fb=1.6)
        osc = next(n for n in ast.walk(fn) if isinstance(n, ast.Call)
                   and isinstance(n.func, ast.Attribute) and n.func.attr == 'osc')
        amp, freq, cycles = [numeric(n, variables) for n in osc.args[:3]]
        phase = numeric(next(k.value for k in osc.keywords if k.arg == 'phase'), variables)
        overlays[name] = dict(amplitude=[math.degrees(a) for a in amp], frequency=freq,
                              duration=cycles/freq, phase=phase, ramp=.18,
                              source=f'elegnt_demo.Show.{method}')
    bundle = dict(source=dict(branch='full-model-animations', commit=REF,
                              sha256={n+'.py': hashlib.sha256(s).hexdigest() for n, s in sources.items()}),
                  joints=['yaw', 'shoulder', 'elbow', 'wrist', 'head_tilt'],
                  poses=poses, clips=clips, overlays=overlays)
    dest = ROOT / 'arm/clips/full-model.json'
    dest.write_text(json.dumps(bundle, indent=2)+'\n')
    print(f'Imported {len(clips)} full-model actions and {len(overlays)} additive layers into {dest}')


if __name__ == '__main__':
    build()
