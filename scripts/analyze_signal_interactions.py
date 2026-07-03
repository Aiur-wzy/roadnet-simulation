#!/usr/bin/env python3
"""Summarize SUMO traffic-light/movement interactions for offset tuning."""
from __future__ import annotations
import argparse, csv, math, xml.etree.ElementTree as ET
from collections import defaultdict

def green_windows(phases, link, offset):
    cycle=sum(float(p.get('duration','0')) for p in phases); t=(-offset)%cycle; out=[]; cur=0.0
    for p in phases:
        dur=float(p.get('duration','0')); state=p.get('state','')
        if link < len(state) and state[link] in 'Gg': out.append((cur,cur+dur))
        cur+=dur
    return cycle,out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('net', nargs='?', default='data/test.net.xml'); ap.add_argument('--out',default='diag_signal_analysis')
    a=ap.parse_args(); root=ET.parse(a.net).getroot()
    import pathlib; out=pathlib.Path(a.out); out.mkdir(exist_ok=True)
    tl_phases={tl.get('id'):tl.findall('phase') for tl in root.findall('tlLogic')}
    tl_offsets={tl.get('id'):float(tl.get('offset','0')) for tl in root.findall('tlLogic')}
    conns=[]; moves={}
    for c in root.findall('connection'):
        f,t=c.get('from'),c.get('to')
        if not f or not t or f.startswith(':') or t.startswith(':'): continue
        li=int(c.get('linkIndex','-1')) if c.get('linkIndex') is not None else -1
        row=dict(fromEdge=f,toEdge=t,fromLane=int(c.get('fromLane','0')),toLane=int(c.get('toLane','0')),tlID=c.get('tl',''),linkIndex=li,dir=c.get('dir',''))
        conns.append(row)
        key=(f,t,row['tlID'],li,row['dir'])
        m=moves.setdefault(key, {'fromLanes':set(),'toLanes':set()}); m['fromLanes'].add(row['fromLane']); m['toLanes'].add(row['toLane'])
    with open(out/'tl_logic_summary.csv','w',newline='') as f:
        w=csv.writer(f); w.writerow(['tlID','offset','cycle','phaseDurations','phaseStates','controlledLinks'])
        bytl=defaultdict(set)
        for c in conns:
            if c['tlID']: bytl[c['tlID']].add(c['linkIndex'])
        for tl,ph in tl_phases.items(): w.writerow([tl,tl_offsets.get(tl,0),sum(float(p.get('duration','0')) for p in ph),'|'.join(p.get('duration','') for p in ph),'|'.join(p.get('state','') for p in ph),len(bytl[tl])])
    with open(out/'connections.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['fromEdge','toEdge','fromLane','toLane','tlID','linkIndex','dir']); w.writeheader(); w.writerows(conns)
    move_rows=[]
    for i,(key,v) in enumerate(moves.items()):
        f,t,tl,li,d=key; cyc,wins=(0,[])
        if tl in tl_phases and li>=0: cyc,wins=green_windows(tl_phases[tl],li,tl_offsets.get(tl,0))
        move_rows.append(dict(movementID=i,fromEdge=f,toEdge=t,tlID=tl,linkIndex=li,dir=d,fromLanes='|'.join(map(str,sorted(v['fromLanes']))),toLanes='|'.join(map(str,sorted(v['toLanes']))),cycle=cyc,greenWindows='|'.join(f'{x:g}-{y:g}' for x,y in wins)))
    with open(out/'movement_groups.csv','w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=move_rows[0].keys()); w.writeheader(); w.writerows(move_rows)
    feeders=defaultdict(list); drainers=defaultdict(list)
    for r in move_rows: feeders[r['toEdge']].append(r); drainers[r['fromEdge']].append(r)
    motifs=[]
    for road,fs in feeders.items():
        for f in fs:
            ft=set(map(int,filter(None,f['toLanes'].split('|'))))
            for d in drainers.get(road,[]):
                df=set(map(int,filter(None,d['fromLanes'].split('|')))); inter=ft&df
                if len(inter)<min(len(ft),len(df)) or len(fs)>1:
                    motifs.append({**{f'up_{k}':v for k,v in f.items()}, **{f'down_{k}':v for k,v in d.items()}, 'sharedEdge':road, 'intersection':'|'.join(map(str,sorted(inter))), 'feederCount':len(fs)})
    with open(out/'downstream_storage_motifs.csv','w',newline='') as f:
        fields=list(motifs[0].keys()) if motifs else ['sharedEdge']; w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(motifs)
    print(f'wrote {out}')
if __name__=='__main__': main()
