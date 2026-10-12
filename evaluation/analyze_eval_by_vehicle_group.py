#!/usr/bin/env python3
"""Detailed grouped SUMO/CAMS evaluation analysis (stdlib only)."""

import argparse, collections, csv, math, os, re, statistics, xml.etree.ElementTree as ET
from pathlib import Path

BINS=[(0,60),(60,120),(120,300),(300,600),(600,900),(900,1800),(1800,math.inf)]
DEPART_BIN=300
PCTS=[10,25,50,75,90,95,99]

def fnum(x, default=None):
    try:
        return float(x)
    except (TypeError,ValueError):
        return default

def fmt(v):
    if isinstance(v,str): return v
    if v is None or (isinstance(v,float) and math.isnan(v)): return ""
    if isinstance(v,int): return str(v)
    return f"{v:.6g}"

def mean(v): return statistics.fmean(v) if v else math.nan

def pct(vals,p):
    vals=sorted(vals)
    if not vals: return math.nan
    if len(vals)==1: return vals[0]
    pos=(len(vals)-1)*p/100; lo=math.floor(pos); hi=math.ceil(pos)
    return vals[lo] if lo==hi else vals[lo]+(vals[hi]-vals[lo])*(pos-lo)

def group_for(vid,single=None):
    if single: return single
    if "veh" in vid:
        return vid.split("veh",1)[0].rstrip("_") or "ungrouped"
    parts=vid.split("_")
    if len(parts)>1 and parts[-1].isdigit(): return "_".join(parts[:-1]) or "ungrouped"
    return "ungrouped"

def parse_route_xml(path):
    if not path: return {},0
    root=ET.parse(path).getroot(); routes={}; named={}
    for r in root.findall('route'):
        rid=r.get('id'); edges=(r.get('edges') or '').split()
        if rid: named[rid]=edges
    for v in root.findall('vehicle'):
        vid=v.get('id'); edges=[]
        r=v.find('route')
        if r is not None: edges=(r.get('edges') or '').split()
        elif v.get('route') in named: edges=named[v.get('route')]
        routes[vid]={"depart":fnum(v.get('depart')),"edges":edges,"pattern":" ".join(edges),"first":edges[0] if edges else "","last":edges[-1] if edges else "","edgeCount":len(edges)}
    return routes,len(routes)

def parse_tripinfo(path, pred=False):
    if not path: return {}
    root=ET.parse(path).getroot(); out={}
    for t in root.findall('tripinfo'):
        vid=t.get('id')
        if not vid: continue
        out[vid]={"depart":fnum(t.get('depart')),"arrival":fnum(t.get('arrival')),
                  "duration":fnum(t.get('predDuration') if pred else t.get('duration')),
                  "predDuration":fnum(t.get('predDuration'))}
    return out

def bin_label(v,bins=BINS):
    for a,b in bins:
        if v is not None and v>=a and v<b: return f"[{a},{'+inf' if b==math.inf else b})"
    return "unknown"

def depart_label(v):
    if v is None: return "unknown"
    a=int(v//DEPART_BIN)*DEPART_BIN; return f"[{a},{a+DEPART_BIN})"

def base_stats(rows):
    e=[r['err'] for r in rows]; ae=[abs(x) for x in e]; pred=[r['pred'] for r in rows]; truth=[r['truth'] for r in rows]
    top=max(rows,key=lambda r:abs(r['err'])) if rows else {}
    under=[r for r in rows if r['err']<0]; over=[r for r in rows if r['err']>0]
    topu=min(under,key=lambda r:r['err']) if under else {}; topo=max(over,key=lambda r:r['err']) if over else {}
    n=len(rows)
    d={"comparedVehicles":n,"positiveErrorCount":len(over),"negativeErrorCount":len(under),"zeroErrorCount":sum(1 for x in e if x==0),
       "positiveErrorRatio":len(over)/n if n else 0,"negativeErrorRatio":len(under)/n if n else 0,
       "meanPredDuration":mean(pred),"meanTruthDuration":mean(truth),"bias":mean(e),"MAE":mean(ae),"MSE":mean([x*x for x in e]),"RMSE":math.sqrt(mean([x*x for x in e])) if e else math.nan,"MAPE":mean([abs(r['err']/r['truth'])*100 for r in rows if r['truth']]),
       "medianAbsError":pct(ae,50),"p75AbsError":pct(ae,75),"p90AbsError":pct(ae,90),"p95AbsError":pct(ae,95),"p99AbsError":pct(ae,99),"maxAbsError":max(ae) if ae else math.nan}
    for th in [60,120,300,600,1000]: d[f"absDurationError_gt_{th}"]=sum(1 for x in ae if x>th)
    for th,s in [(0.5,'0_5'),(1.0,'1_0'),(3.0,'3_0'),(5.0,'5_0')]: d[f"relativeDurationError_gt_{s}"]=sum(1 for r in rows if r.get('rel') is not None and r['rel']>th)
    for col in ['pred','truth']:
        name='predDuration' if col=='pred' else 'truthDuration'
        d[f"{name}_gt_1000"]=sum(1 for r in rows if r[col]>1000); d[f"{name}_gt_2000"]=sum(1 for r in rows if r[col]>2000)
    for prefix,row in [('topAbsError',top),('topUnder',topu),('topOver',topo)]:
        d.update({f"{prefix}VehicleID":row.get('vehicleID',''),f"{prefix}PredDuration":row.get('pred',''),f"{prefix}TruthDuration":row.get('truth',''),f"{prefix}SignedError":row.get('err',''),f"{prefix}AbsError":abs(row['err']) if row else ''})
    return d

def sorted_stats(rows):
    pred=sorted(r['pred'] for r in rows); truth=sorted(r['truth'] for r in rows); diff=[a-b for a,b in zip(pred,truth)]
    d={"sortedMAE":mean([abs(x) for x in diff]),"sortedRMSE":math.sqrt(mean([x*x for x in diff])) if diff else math.nan,"sortedBias":mean(diff)}
    for p in PCTS:
        d.update({f"predP{p}":pct(pred,p),f"truthP{p}":pct(truth,p),f"diffP{p}":pct(pred,p)-pct(truth,p)})
    d.update({"predMean":mean(pred),"truthMean":mean(truth),"diffMean":mean(pred)-mean(truth)})
    return d

def side_stats(rows):
    n=len(rows); under=[r for r in rows if r['err']<0]; over=[r for r in rows if r['err']>0]; severe=[r for r in rows if abs(r['err'])>1000]
    d={}
    for name,side in [('under',under),('over',over)]:
        ae=[abs(r['err']) for r in side]; er=[r['err'] for r in side]
        d.update({f"{name}Count":len(side),f"{name}Ratio":len(side)/n if n else 0,f"{name}MeanError":mean(er),f"{name}MAE":mean(ae),f"{name}MedianAbsError":pct(ae,50),f"{name}P90AbsError":pct(ae,90),f"{name}P95AbsError":pct(ae,95),f"{name}MaxAbsError":max(ae) if ae else math.nan})
    su=sum(1 for r in severe if r['err']<0); so=sum(1 for r in severe if r['err']>0)
    d.update({"severeUnderCount_absError_gt_1000":su,"severeOverCount_absError_gt_1000":so,"severeUnderRatioAmongSevere":su/len(severe) if severe else 0,"severeOverRatioAmongSevere":so/len(severe) if severe else 0})
    return d

def cliff_stats(rows):
    under=[r for r in rows if (r['truth']>=900 and r['pred']<=300) or r['err']<=-1000]
    over=[r for r in rows if (r['pred']>=900 and r['truth']<=300) or r['err']>=1000]
    def tops(rs, key): return ";".join(k for k,_ in collections.Counter(r[key] for r in rs if r.get(key)).most_common(5))
    return {"underCliffCount":len(under),"underCliffRatio":len(under)/len(rows) if rows else 0,"overCliffCount":len(over),"overCliffRatio":len(over)/len(rows) if rows else 0,
            "topUnderCliffVehicles":";".join(r['vehicleID'] for r in sorted(under,key=lambda r:r['err'])[:5]),"topOverCliffVehicles":";".join(r['vehicleID'] for r in sorted(over,key=lambda r:r['err'],reverse=True)[:5]),
            "topUnderCliffRoutePatterns":tops(under,'pattern'),"topOverCliffRoutePatterns":tops(over,'pattern'),"topUnderCliffDepartBins":tops(under,'departBin'),"topOverCliffDepartBins":tops(over,'departBin')}

def write_csv(path, rows):
    if not rows: return
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields: fields.append(k)
    with open(path,'w',newline='') as f:
        w=csv.DictWriter(f,fields,lineterminator='\n'); w.writeheader(); w.writerows({k:fmt(r.get(k)) for k in fields} for r in rows)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--eval-csv',required=True); ap.add_argument('--dataset-name',required=True); ap.add_argument('--single-group'); ap.add_argument('--route-xml'); ap.add_argument('--truth-tripinfo'); ap.add_argument('--prediction-tripinfo'); ap.add_argument('--route-generator'); ap.add_argument('--output-csv'); ap.add_argument('--output-md'); ap.add_argument('--output-report'); ap.add_argument('--output-dir'); ap.add_argument('--output-prefix')
    a=ap.parse_args(); outdir=Path(a.output_dir or Path(a.output_csv or '.').parent); prefix=a.output_prefix or a.dataset_name
    routes,route_count=parse_route_xml(a.route_xml); truth=parse_tripinfo(a.truth_tripinfo); predxml=parse_tripinfo(a.prediction_tripinfo,True)
    rows=[]; groups=collections.defaultdict(list)
    with open(a.eval_csv,newline='') as f:
        for row in csv.DictReader(f):
            if row.get('validVehicle','true').lower()=='false': continue
            vid=row.get('vehicleID',''); pred=fnum(row.get('predDuration')); tru=fnum(row.get('truthDuration')); err=fnum(row.get('durationErrorSigned'), fnum(row.get('durationError')))
            if vid=='' or pred is None or tru is None or err is None: continue
            meta=routes.get(vid,{})
            r={"vehicleID":vid,"pred":pred,"truth":tru,"err":err,"rel":fnum(row.get('relativeDurationError')),"depart":fnum(row.get('truthDepart'),meta.get('depart')),"pattern":meta.get('pattern',''),"routeFirstEdge":meta.get('first',''),"routeLastEdge":meta.get('last',''),"routeLengthEdgesCount":meta.get('edgeCount','')}
            r['departBin']=depart_label(r['depart']); r['group']=group_for(vid,a.single_group); rows.append(r); groups[r['group']].append(r)
    groups['ALL']=rows
    summary=[]; sorted_rows=[]; bias=[]; cliff=[]; truth_bins=[]; pred_bins=[]; route_summary=[]
    for g,rs in sorted(groups.items()):
        base={"dataset":a.dataset_name,"group":g}; summary.append({**base,**base_stats(rs),**sorted_stats(rs),**side_stats(rs),**cliff_stats(rs),"departMin":min([r['depart'] for r in rs if r['depart'] is not None], default=math.nan),"departMax":max([r['depart'] for r in rs if r['depart'] is not None], default=math.nan)})
        sorted_rows.append({**base,**sorted_stats(rs)}); bias.append({**base,**side_stats(rs)}); cliff.append({**base,**cliff_stats(rs)})
        for labeler,col,target in [(bin_label,'truth',truth_bins),(bin_label,'pred',pred_bins)]:
            for bl,brs in collections.defaultdict(list, {k:[x for x in rs if labeler(x[col])==k] for k in set(labeler(x[col]) for x in rs)}).items():
                if brs: target.append({**base,"bin":bl,"count":len(brs),"meanPredDuration":mean([x['pred'] for x in brs]),"meanTruthDuration":mean([x['truth'] for x in brs]),"bias":mean([x['err'] for x in brs]),"MAE":mean([abs(x['err']) for x in brs]),"RMSE":math.sqrt(mean([x['err']*x['err'] for x in brs])),"medianAbsError":pct([abs(x['err']) for x in brs],50),"p90AbsError":pct([abs(x['err']) for x in brs],90)})
        if g!='ALL':
            for pat,prs in collections.Counter(r['pattern'] for r in rs if r['pattern']).most_common(5):
                pr=[r for r in rs if r['pattern']==pat]; route_summary.append({**base,"routePattern":pat,"count":len(pr),"meanAbsError":mean([abs(r['err']) for r in pr]),"absDurationError_gt_1000":sum(1 for r in pr if abs(r['err'])>1000),"meanUnderprediction":mean([r['err'] for r in pr if r['err']<0]),"meanOverprediction":mean([r['err'] for r in pr if r['err']>0])})
    write_csv(a.output_csv or outdir/f"{prefix}_detailed_group_summary.csv",summary)
    write_csv(outdir/f"{prefix}_sorted_distribution_by_group.csv",sorted_rows); write_csv(outdir/f"{prefix}_left_right_bias_by_group.csv",bias); write_csv(outdir/f"{prefix}_truth_duration_bins_by_group.csv",truth_bins); write_csv(outdir/f"{prefix}_pred_duration_bins_by_group.csv",pred_bins); write_csv(outdir/f"{prefix}_cliff_summary_by_group.csv",cliff); write_csv(outdir/f"{prefix}_route_pattern_summary.csv",route_summary)
    mismatches=sum(1 for r in rows if r['vehicleID'] in predxml and predxml[r['vehicleID']].get('duration') is not None and abs(predxml[r['vehicleID']]['duration']-r['pred'])>1e-3)
    missing_pred=sum(1 for r in rows if predxml and r['vehicleID'] not in predxml); extra_pred=len(set(predxml)-{r['vehicleID'] for r in rows}) if predxml else 0
    md=[f"# Detailed grouped evaluation: {a.dataset_name}","",f"Compared vehicles: {len(rows)}. Route vehicles: {route_count}. Truth records: {len(truth)}.",f"Prediction tripinfo consistency: missing={missing_pred}, extra={extra_pred}, mismatchedDurations={mismatches}.",""]
    for r in sorted(summary,key=lambda x:(x['group']!='ALL', x['group'])):
        md += [f"## {r['group']}",f"- Vehicles: {r['comparedVehicles']}; bias={fmt(r['bias'])}s; MAE={fmt(r['MAE'])}s; RMSE={fmt(r['RMSE'])}s; paired p95 abs={fmt(r['p95AbsError'])}s.",f"- Sorted distribution: sortedMAE={fmt(r['sortedMAE'])}s; sortedRMSE={fmt(r['sortedRMSE'])}s; diffMean={fmt(r['diffMean'])}s.",f"- Left/right: under={r['underCount']} ({fmt(r['underRatio'])}); over={r['overCount']} ({fmt(r['overRatio'])}); severe under/over={r['severeUnderCount_absError_gt_1000']}/{r['severeOverCount_absError_gt_1000']}.",f"- Cliffs: under={r['underCliffCount']} ({fmt(r['underCliffRatio'])}); over={r['overCliffCount']} ({fmt(r['overCliffRatio'])}). Worst vehicle={r['topAbsErrorVehicleID']} ({fmt(r['topAbsErrorSignedError'])}s).",""]
    md += ["## Route-generation-based possible causes","", "See the complete report for generator-code interpretation; this dataset report keeps compact metrics only."]
    outmd=a.output_md or outdir/f"{prefix}_detailed_group_report.md"; Path(outmd).write_text("\n".join(md)+"\n")
    if a.output_report: Path(a.output_report).write_text("\n".join(md)+"\n")
    print(f"prediction_tripinfo_check missing={missing_pred} extra={extra_pred} mismatchedDurations={mismatches}")
    print(f"groups={','.join(sorted(groups))}")
if __name__=='__main__': main()
