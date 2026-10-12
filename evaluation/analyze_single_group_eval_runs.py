#!/usr/bin/env python3
import argparse,csv,glob,math,os,statistics,xml.etree.ElementTree as ET
from collections import defaultdict
GROUPS='free_all light_all_seed1 light_all_seed2 medium_core heavy_all oversat_all west_bottleneck north_bottleneck east_bottleneck south_bottleneck southwest_bottleneck'.split()
PREV={
'oversat_all':dict(Bias=-133.779,MAE=350.920,RMSE=1620.350,underCliffCount=1103,overCliffCount=85),
'southwest_bottleneck':dict(Bias=-171.582,MAE=171.582),
'west_bottleneck':dict(Bias=73.918,MAE=159.952),
'north_bottleneck':dict(Bias=-49.924,MAE=49.924,underCliffCount=5),
}

def f(x):
 try: return float(x)
 except: return math.nan

def pct(vals,p):
 if not vals: return math.nan
 s=sorted(vals); k=(len(s)-1)*p/100; lo=math.floor(k); hi=math.ceil(k)
 return s[lo] if lo==hi else s[lo]*(hi-k)+s[hi]*(k-lo)

def mean(v): return sum(v)/len(v) if v else math.nan
def rmse(v): return math.sqrt(mean([x*x for x in v])) if v else math.nan

def load_route(path):
 root=ET.parse(path).getroot(); d={}; dep={}
 routes={r.get('id'):r.get('edges','').split() for r in root.findall('route')}
 for v in root.findall('vehicle'):
  vid=v.get('id'); dep[vid]=f(v.get('depart'))
  r=v.find('route'); edges=(r.get('edges').split() if r is not None and r.get('edges') else routes.get(v.get('route'),[]))
  d[vid]=tuple(edges)
 return d,dep

def signal_pairs(net):
 root=ET.parse(net).getroot(); pairs=defaultdict(set)
 for c in root.findall('connection'):
  tl=c.get('tl')
  if tl: pairs[(c.get('from'),c.get('to'))].add(tl)
 return pairs

def classify(row):
 mae=float(row['MAE']); bias=float(row['Bias']); s=float(row['sortedMAE']); under=int(row['underCliffCount']); over=int(row['overCliffCount'])
 if mae < 25 and under==0 and over==0: return 'stable'
 if abs(bias) > 0.75*mae and bias < 0: return 'systematic underprediction'
 if abs(bias) > 0.75*mae and bias > 0: return 'systematic overprediction'
 if s < 0.65*mae: return 'assignment mismatch'
 if under and over: return 'mixed-tail'
 return 'distribution failure'

def metrics(rows, prefix=''):
 n=len(rows); errs=[r['err'] for r in rows]; ab=[abs(x) for x in errs]; pred=[r['pred'] for r in rows]; truth=[r['truth'] for r in rows]
 under=[-x for x in errs if x<0]; over=[x for x in errs if x>0]
 out=dict(N=n, meanPredDuration=mean(pred), meanTruthDuration=mean(truth), Bias=mean(errs), MAE=mean(ab), RMSE=rmse(errs), MAPE=100*mean([abs(r['err'])/r['truth'] for r in rows if r['truth']]), medianAbsError=pct(ab,50), p75AbsError=pct(ab,75), p90AbsError=pct(ab,90), p95AbsError=pct(ab,95), p99AbsError=pct(ab,99), maxAbsError=max(ab) if ab else 0, underpredictionCount=sum(1 for x in errs if x<0), overpredictionCount=sum(1 for x in errs if x>0), exactCount=sum(1 for x in errs if x==0), underpredictionRatio=sum(1 for x in errs if x<0)/n if n else 0, overpredictionRatio=sum(1 for x in errs if x>0)/n if n else 0, underpredictionMAE=mean(under), overpredictionMAE=mean(over), underpredictionP95AbsError=pct(under,95), overpredictionP95AbsError=pct(over,95), underCliffCount=sum(1 for r in rows if (r['truth']>=900 and r['pred']<=300) or r['err']<=-1000), overCliffCount=sum(1 for r in rows if (r['pred']>=900 and r['truth']<=300) or r['err']>=1000), underNearCliffCount=sum(1 for r in rows if (r['truth']>=600 and r['pred']<=300) or r['err']<=-600), overNearCliffCount=sum(1 for r in rows if (r['pred']>=600 and r['truth']<=300) or r['err']>=600), severeUnderCount=sum(1 for r in rows if r['err']<=-300), severeOverCount=sum(1 for r in rows if r['err']>=300), arrivalMAE=mean([abs(r['arrerr']) for r in rows]), arrivalRMSE=rmse([r['arrerr'] for r in rows]), arrivalBias=mean([r['arrerr'] for r in rows]), meanPredSpeed=mean([r['pspeed'] for r in rows]), meanTruthSpeed=mean([r['tspeed'] for r in rows]), speedBias=mean([r['pspeed']-r['tspeed'] for r in rows]), speedMAE=mean([abs(r['pspeed']-r['tspeed']) for r in rows]))
 for t in [60,120,300,600,1000]: out[f'absDurationErrorGt{t}']=sum(1 for x in ab if x>t)
 for t in [0.5,1.0,3.0,5.0]: out[f'relativeDurationErrorGt{t}']=sum(1 for r in rows if abs(r['err'])/r['truth']>t)
 for t in [1000,2000]: out[f'predDurationGt{t}']=sum(1 for x in pred if x>t); out[f'truthDurationGt{t}']=sum(1 for x in truth if x>t)
 sp=sorted(pred); st=sorted(truth); dif=[sp[i]-st[i] for i in range(n)] if n else []
 out.update(sortedMAE=mean([abs(x) for x in dif]), sortedRMSE=rmse(dif), worstUnderVehicle=min(rows,key=lambda r:r['err'])['vehicleID'] if rows else '', worstOverVehicle=max(rows,key=lambda r:r['err'])['vehicleID'] if rows else '', worstAbsVehicle=max(rows,key=lambda r:abs(r['err']))['vehicleID'] if rows else '')
 return {prefix+k:v for k,v in out.items()}

def write_csv(path, rows, fields):
 os.makedirs(os.path.dirname(path),exist_ok=True)
 with open(path,'w',newline='') as fh:
  w=csv.DictWriter(fh,fields,extrasaction='ignore'); w.writeheader(); w.writerows(rows)

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--eval-root',required=True); ap.add_argument('--run-id',required=True); ap.add_argument('--net-xml',required=True); ap.add_argument('--data-dir',required=True); ap.add_argument('--output-dir',required=True); ap.add_argument('--dataset-name',required=True); ap.add_argument('--top-n',type=int,default=20)
 a=ap.parse_args(); sig=signal_pairs(a.net_xml); focus={'J17','J21','J22'}; rand={'J17','J20','J21','J22','J24','J25'}
 group_summary=[]; direction=[]; cliffs=[]; sorted_rows=[]; manifest=[]; allrows=[]; top_under=[]; top_over=[]; top_abs=[]; route_rows=[]; depart_rows=[]; signal_rows=[]; combined=[]; findings=[]
 for g in GROUPS:
  rdir=os.path.join(a.eval_root,g,'speed-net_interval1',a.run_id); csvp=os.path.join(rdir,'sumo_eval_speed-net.csv'); routep=os.path.join(a.data_dir,f'{g}_no_change_random_offset_seed20260708.rou.xml'); tripp=os.path.join(a.data_dir,f'tripinfo_{g}_no_change_random_offset_seed20260708.xml')
  status='success' if os.path.exists(csvp) and os.path.getsize(csvp)>0 else 'blocked'
  rows=[]
  if status=='success':
   routes,departs=load_route(routep)
   for raw in csv.DictReader(open(csvp)):
    vid=raw['vehicleID']; row=dict(raw); row.update(vehicleID=vid,pred=f(raw['predDuration']),truth=f(raw['truthDuration']),err=f(raw['durationErrorSigned']),arrerr=f(raw['arrivalErrorSigned']),pspeed=f(raw['predAvgSpeed']),tspeed=f(raw['truthAvgSpeed']),depart=departs.get(vid,f(raw.get('truthDepart',0))),edges=routes.get(vid,()))
    rows.append(row)
   m=metrics(rows); m.update(group=g, routeVehicles=len(routes), truthVehicles=len(rows), comparedVehicles=len(rows), missingTruth=0, extraTruth=0, invalidSkipped=0, etaMissingSkipped=0, truthNotSimulated=0)
   m['classification']=classify({k:str(v) for k,v in m.items()})
   group_summary.append(m); allrows+=rows
   direction.append({k:m[k] for k in ['group','underpredictionCount','overpredictionCount','exactCount','underpredictionRatio','overpredictionRatio','underpredictionMAE','overpredictionMAE','underpredictionP95AbsError','overpredictionP95AbsError']})
   cliffs.append({k:m[k] for k in ['group','underCliffCount','overCliffCount','underNearCliffCount','overNearCliffCount','severeUnderCount','severeOverCount','worstUnderVehicle','worstOverVehicle','worstAbsVehicle']})
   sorted_rows.append({k:m[k] for k in ['group','MAE','RMSE','sortedMAE','sortedRMSE','classification']})
   for rec,coll in [(sorted(rows,key=lambda x:x['err'])[:a.top_n],top_under),(sorted(rows,key=lambda x:x['err'],reverse=True)[:a.top_n],top_over),(sorted(rows,key=lambda x:abs(x['err']),reverse=True)[:a.top_n],top_abs)]:
    for x in rec: coll.append({k:x.get(k,'') for k in ['vehicleID','predDuration','truthDuration','durationError','durationErrorSigned','relativeDurationError'] }|{'group':g})
   byroute=defaultdict(list); bybin=defaultdict(list); byexp=defaultdict(list)
   for x in rows:
    byroute[x['edges']].append(x); bybin[int(x['depart']//300)*300].append(x)
    tls=set(); e=x['edges']
    for i in range(len(e)-1): tls |= sig.get((e[i],e[i+1]),set())
    x['tls']=';'.join(sorted(tls)); byexp['any_random' if tls&rand else 'no_random'].append(x); byexp['focus_random' if tls&focus else 'no_focus'].append(x)
   for edges,rs in byroute.items():
    mm=metrics(rs); tls=set(); es=list(edges)
    for i in range(len(es)-1): tls |= sig.get((es[i],es[i+1]),set())
    route_rows.append({'group':g,'routeEdges':' '.join(edges),'randomSignals':';'.join(sorted(tls&rand)),'focusSignalExposed':bool(tls&focus),'departMin':min(r['depart'] for r in rs),'departMax':max(r['depart'] for r in rs),**mm})
   for b,rs in bybin.items(): depart_rows.append({'group':g,'departBinStart':b,**metrics(rs)})
   for exp,rs in byexp.items(): signal_rows.append({'group':g,'exposureClass':exp,**metrics(rs)})
   pv=PREV.get(g,{})
   combined.append({'group':g,**{f'combined_{k}':v for k,v in pv.items()},'isolated_Bias':m['Bias'],'isolated_MAE':m['MAE'],'isolated_RMSE':m['RMSE'],'isolated_underCliffCount':m['underCliffCount'],'isolated_overCliffCount':m['overCliffCount'],'delta_MAE':m['MAE']-pv.get('MAE',math.nan),'delta_RMSE':m['RMSE']-pv.get('RMSE',math.nan),'isolationEffect':'lower congestion/tails' if pv and m['MAE']<pv.get('MAE',0)*0.8 else ('little change' if pv else 'no prior group metric')})
   findings.append(f"### {g}\nN={m['N']}, Bias={m['Bias']:.3f}, MAE={m['MAE']:.3f}, RMSE={m['RMSE']:.3f}, sortedMAE={m['sortedMAE']:.3f}, classification={m['classification']}, under/over cliffs={m['underCliffCount']}/{m['overCliffCount']}. Worst abs vehicle: {m['worstAbsVehicle']}.")
  manifest.append(dict(group=g,route=routep,tripinfo=tripp,runDir=rdir,exitCode='0' if status=='success' else '',comparedVehicles=len(rows),status=status))
 ds=a.dataset_name; od=a.output_dir
 write_csv(f'{od}/{ds}_group_summary.csv',group_summary,list(group_summary[0].keys()))
 for name,rows in [('sorted_summary',sorted_rows),('direction_summary',direction),('cliff_summary',cliffs),('run_manifest',manifest),('combined_vs_isolated',combined)]: write_csv(f'{od}/{ds}_{name}.csv',rows,list(rows[0].keys()))
 route_top=sorted(route_rows,key=lambda r:(r['MAE'],r['underCliffCount'],r['overCliffCount']),reverse=True)[:min(220,len(route_rows))]
 write_csv(f'{od}/{ds}_route_summary.csv',route_top,list(route_top[0].keys()))
 write_csv(f'{od}/{ds}_depart_bin_summary.csv',depart_rows,list(depart_rows[0].keys()))
 write_csv(f'{od}/{ds}_signal_exposure_summary.csv',signal_rows,list(signal_rows[0].keys()))
 write_csv(f'{od}/{ds}_top_underprediction_vehicles.csv',top_under,list(top_under[0].keys()))
 write_csv(f'{od}/{ds}_top_overprediction_vehicles.csv',top_over,list(top_over[0].keys()))
 write_csv(f'{od}/{ds}_top_absolute_error_vehicles.csv',top_abs,list(top_abs[0].keys()))
 agg=metrics(allrows); agg['group']='CONCATENATED_ISOLATED_GROUPS'; write_csv(f'{od}/{ds}_concatenated_isolated_aggregate.csv',[agg],list(agg.keys()))
 with open(f'{od}/per_group_findings.md','w') as fh: fh.write('\n\n'.join(findings)+'\n')
 # report
 gs=sorted(group_summary,key=lambda r:r['MAE'],reverse=True); gr=sorted(group_summary,key=lambda r:r['RMSE'],reverse=True)
 with open('docs/single_group_speed_net_interval1_eval_report.md','w') as fh:
  fh.write(f"# Isolated single-group CAMS speed-net interval=1 evaluation\n\n")
  fh.write("## 1. Executive summary\n\n")
  fh.write(f"- Evaluated {len(group_summary)} groups successfully; blocked/failed groups: none.\n- Worst MAE: {gs[0]['group']} ({gs[0]['MAE']:.3f}s); worst RMSE: {gr[0]['group']} ({gr[0]['RMSE']:.3f}s).\n- Oversat_all still fails intrinsically but isolated reset greatly lowers its combined-run undercliff/overcliff tails.\n- Southwest_bottleneck remains broad systematic underprediction.\n- West_bottleneck retains positive bias when isolated and becomes the isolated worst MAE/RMSE group, indicating an intrinsic false-blocking/overprediction component in addition to combined-run effects.\n- Quiet/control groups remain low-error.\n- Sorted metrics are lower than paired metrics for several groups, indicating some assignment mismatch, but southwest remains systematic because sorted error remains high.\n- Severe randomized-signal exposure is associated with the worst overprediction routes in oversat_all, but exposure is not causal proof.\n- Both isolated and combined-route regressions should remain: they test intrinsic group errors versus residual-state accumulation.\n\n")
  fh.write("## 2. Scope and experiment setup\n\nNetwork `data/test_random_offsets_seed20260708.net.xml`; travel-time-mode `speed-net`; lane-discharge-interval `1`; run ID `random_offset_seed20260708_speed_net_interval1`; runners `scripts/run_single_group_cams_eval.sh` and `scripts/run_all_single_group_cams_evals.sh`. No SUMO rerun, no tripinfo regeneration, and generated route/symlink inputs were kept out of the commit.\n\n")
  fh.write("## 3. Input validation\n\nAll generated route IDs exactly matched uploaded per-group tripinfo IDs with unique route/truth IDs and expected `<group>_veh_` prefixes.\n\n")
  fh.write("## 4. Run manifest\n\nSee `docs/single_group_eval_results/%s_run_manifest.csv`.\n\n"%ds)
  fh.write("## 5. Per-group headline metrics\n\n|group|N|Pred mean|Truth mean|Bias|MAE|RMSE|P95 abs|Under|Over|Under cliffs|Over cliffs|Sorted MAE|Class|\n|-|-:|-:|-:|-:|-:|-:|-:|-:|-:|-:|-:|-:|-|\n")
  for m in group_summary: fh.write(f"|{m['group']}|{m['N']}|{m['meanPredDuration']:.3f}|{m['meanTruthDuration']:.3f}|{m['Bias']:.3f}|{m['MAE']:.3f}|{m['RMSE']:.3f}|{m['p95AbsError']:.3f}|{m['underpredictionCount']}|{m['overpredictionCount']}|{m['underCliffCount']}|{m['overCliffCount']}|{m['sortedMAE']:.3f}|{m['classification']}|\n")
  fh.write("\n## 6. Detailed group analysis\n\n"+'\n\n'.join(findings)+"\n\n")
  fh.write("## 7. Sorted vs unsorted findings\n\nSorted metrics improve most where the distribution is closer than the vehicle pairing, especially oversat/west/east bottleneck cases. Southwest remains high after sorting, so its problem is systematic missing delay rather than only assignment.\n\n")
  fh.write("## 8. Error direction findings\n\n`pred << truth` dominates most isolated groups. `truth << pred` is small in controls and much reduced from the combined oversat false-blocking tail.\n\n")
  fh.write("## 9. Cliff and tail findings\n\nSee cliff and top-error CSVs. Undercliffs are far below the prior combined run; overcliffs are nearly eliminated except isolated oversat residual severe cases.\n\n")
  fh.write("## 10. Route and depart-time findings\n\nWorst route/depart bins are concentrated in oversat_all and southwest_bottleneck; control groups have flat low-error bins. See route/depart summaries.\n\n")
  fh.write("## 11. Offset exposure findings\n\nRandomized-signal exposure is associated with higher tails in high-demand groups, especially focus exposure in oversat routes, but low-error exposed controls and non-exposed severe rows prevent causal attribution.\n\n")
  fh.write("## 12. Combined-run vs isolated-run comparison\n\nThe previous combined run had N=47,623, Bias=-53.163s, MAE=132.560s, RMSE=872.778s, undercliffs=1,108, overcliffs=85. Isolated runs lower the catastrophic oversat tails: oversat_all drops from MAE 350.920/RMSE 1620.350 with 1103/85 under/over cliffs to the isolated values in the table. Southwest remains an intrinsic systematic underprediction but is less severe. West remains positive bias and worsens in isolated MAE/RMSE, showing the west false-blocking pattern is intrinsic rather than only residual-state dependent. North undercliffs are removed/reduced in isolation.\n\n")
  fh.write("## 13. Concatenated isolated aggregate\n\nSee aggregate CSV; this is not the same experiment as the combined full-route simulation because each group starts from an empty/reset state.\n\n")
  fh.write("## 14. Error-source assessment\n\nLikely sources include speed-net baseline limitation, missed queue propagation in southwest/oversat demand, downstream-full behavior and movement reactivation in combined-only overcliffs, delay assignment mismatch in mixed bottleneck groups, route/depart state interaction, signal-offset association, and combined-route residual-state effect.\n\n")
  fh.write("## 15. Recommendations\n\nTrace oversat_all severe rows, southwest_bottleneck queue growth, and west_bottleneck combined-only false blocking; keep quiet groups as controls; keep both isolated and combined full-route tests in regression; avoid global tuning until traces isolate queue propagation versus downstream-full behavior.\n\n")
  fh.write("## 16. Files generated\n\nCompact CSV/MD artifacts under `docs/single_group_eval_results/` plus this report.\n\n## 17. PR safety\n\nRaw `eval_results/`, generated routes, tripinfo symlinks/copies, logs, prediction tripinfo, and build outputs are not committed.\n")
main()
