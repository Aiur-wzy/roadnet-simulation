#!/usr/bin/env python3
import argparse,csv,math,statistics,collections,xml.etree.ElementTree as ET,hashlib,re
from pathlib import Path
MOD={'J17','J21','J22'}
def f(x,d=0.0):
    try: return float(x)
    except Exception: return d
def group(vid):
    return vid.split('veh')[0].rstrip('_') if 'veh' in vid else re.sub(r'_\d+$','',vid) if re.search(r'_\d+$',vid) else 'ungrouped'
def rh(edges): return hashlib.sha1(' '.join(edges).encode()).hexdigest()[:12]
def prev(edges): return ' -> '.join(edges if len(edges)<=6 else edges[:3]+['...']+edges[-2:])
def q(vals,p):
    if not vals: return 0
    vals=sorted(vals); return vals[min(len(vals)-1,math.ceil(p*len(vals))-1)]
def stats(rows):
    n=len(rows); errs=[r['err'] for r in rows]; ae=[abs(x) for x in errs]
    return dict(vehicleCount=n,meanPredDuration=sum(r['pred'] for r in rows)/n,meanTruthDuration=sum(r['truth'] for r in rows)/n,bias=sum(errs)/n,MAE=sum(ae)/n,RMSE=math.sqrt(sum(x*x for x in errs)/n),medianAbsError=statistics.median(ae),p90AbsError=q(ae,.9),p95AbsError=q(ae,.95),maxAbsError=max(ae),underCount=sum(e<0 for e in errs),overCount=sum(e>0 for e in errs),underCliffCount=sum(r['underCliff'] for r in rows),overCliffCount=sum(r['overCliff'] for r in rows),underNearCliffCount=sum(r['underNear'] for r in rows),overNearCliffCount=sum(r['overNear'] for r in rows),severeAbsErrorGt1000Count=sum(abs(e)>1000 for e in errs))
def write(path, rows, fields):
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    with open(path,'w',newline='') as o:
        w=csv.DictWriter(o,fields,lineterminator='\n'); w.writeheader(); w.writerows([{k:r.get(k,'') for k in fields} for r in rows])
def parse_routes(p):
    root=ET.parse(p).getroot(); routes={}
    defs={r.get('id'):r.get('edges','').split() for r in root.findall('route')}
    for v in root.findall('vehicle'):
        vid=v.get('id'); edges=[]
        r=v.find('route')
        if r is not None: edges=r.get('edges','').split()
        elif v.get('route') in defs: edges=defs[v.get('route')]
        routes[vid]={'depart':f(v.get('depart')),'edges':edges,'routeHash':rh(edges),'routePreview':prev(edges),'firstEdge':edges[0] if edges else '', 'lastEdge':edges[-1] if edges else '', 'edgeCount':len(edges)}
    return routes
def parse_trip(p,prefix):
    d={}
    for t in ET.parse(p).getroot().iter('tripinfo'):
        vid=t.get('id'); d[vid]={prefix+'Depart':f(t.get('depart')),prefix+'Arrival':f(t.get('arrival')),prefix+'Duration':f(t.get('duration')),prefix+'WaitingTime':f(t.get('waitingTime')),prefix+'TimeLoss':f(t.get('timeLoss')),prefix+'RouteLength':f(t.get('routeLength'))}
    return d
def parse_net(p):
    root=ET.parse(p).getroot(); tls={}; con={}; edges={}
    for tl in root.findall('tlLogic'):
        phases=[(f(ph.get('duration')),ph.get('state','')) for ph in tl.findall('phase')]
        tls[tl.get('id')]={'offset':f(tl.get('offset')),'phases':phases,'cycle':sum(x for x,_ in phases)}
    for e in root.findall('edge'):
        if e.get('function'): continue
        lanes=e.findall('lane')
        if lanes: edges[e.get('id')]={'speed':f(lanes[0].get('speed'),13.89),'length':f(lanes[0].get('length'),0)}
    for c in root.findall('connection'):
        if c.get('tl'): con[(c.get('from'),c.get('to'))]={'tl':c.get('tl'),'linkIndex':int(c.get('linkIndex','0'))}
    return tls,con,edges
def sig_state(tl,idx,time):
    if not tl or tl['cycle']<=0: return ''
    x=(time+tl['offset'])%tl['cycle']; acc=0
    for dur,state in tl['phases']:
        acc+=dur
        if x<acc: return state[idx] if idx < len(state) else '?'
    return '?'
def main():
    ap=argparse.ArgumentParser();
    for a in ['eval-csv','route-xml','truth-tripinfo','prediction-tripinfo','net-xml','output-dir','dataset-name']: ap.add_argument('--'+a,required=True)
    ap.add_argument('--route-generator'); ap.add_argument('--top-n',type=int,default=20); args=ap.parse_args(); out=Path(args.output_dir); ds=args.dataset_name
    routes=parse_routes(args.route_xml); truth=parse_trip(args.truth_tripinfo,'truth'); predxml=parse_trip(args.prediction_tripinfo,'pred'); tls,con,edgeinfo=parse_net(args.net_xml)
    rows=[]
    for r in csv.DictReader(open(args.eval_csv)):
        vid=r.get('vehicleID') or r.get('vehicleId') or r.get('id'); pred=f(r.get('predDuration')); tru=f(r.get('truthDuration')); err=pred-tru
        rt=routes.get(vid,{}); edges=rt.get('edges',[]); sigs=[]; mods=[]; modcnt=allcnt=0; firstmod=''; ff=rt.get('depart',0); firststate=''
        for a,b in zip(edges,edges[1:]):
            m=con.get((a,b));
            if m:
                allcnt+=1; sigs.append(m['tl'])
                if m['tl'] in MOD:
                    modcnt+=1; mods.append(m['tl']); firstmod=firstmod or m['tl']; firststate=firststate or sig_state(tls.get(m['tl']),m['linkIndex'],ff)
            if a in edgeinfo and edgeinfo[a]['speed']>0: ff+=edgeinfo[a]['length']/edgeinfo[a]['speed']
        rec={'vehicleID':vid,'group':group(vid),'depart':rt.get('depart',f(r.get('truthDepart'))),'pred':pred,'truth':tru,'err':err,'abs':abs(err),'rel':f(r.get('relativeDurationError')),'truthArrival':f(r.get('truthArrival')),'predArrival':f(r.get('predArrival')),'truthWaitingTime':f(r.get('truthWaitingTime')),'truthTimeLoss':f(r.get('truthTimeLoss')),**rt,'exposedModifiedSignals':';'.join(sorted(set(mods))),'exposedAllSignals':';'.join(sorted(set(sigs))),'modifiedSignalCount':modcnt,'allSignalCount':allcnt,'firstModifiedSignal':firstmod,'firstModifiedSignalDepartState':firststate}
        rec['underCliff']=(tru>=900 and pred<=300) or err<=-1000; rec['overCliff']=(pred>=900 and tru<=300) or err>=1000; rec['underNear']=(tru>=600 and pred<=300) or err<=-600; rec['overNear']=(pred>=600 and tru<=300) or err>=600
        if rec['group']=='oversat_all' and rec['underCliff']: lab='oversat_under_cliff_all_corridor'
        elif rec['group']=='oversat_all' and rec['overCliff']: lab='oversat_over_cliff_false_blocking'
        elif rec['group']=='west_bottleneck' and err<0: lab='west_directional_underprediction'
        elif rec['group']=='southwest_bottleneck' and err<0: lab='southwest_directional_underprediction'
        elif rec['group']=='north_bottleneck' and err<0: lab='north_directional_underprediction'
        else: lab='unknown'
        rec['candidateCauseLabel']=lab; rows.append(rec)
    val={'evalVehicles':len(rows),'missingRoute':sum(r['vehicleID'] not in routes for r in rows),'missingTruth':sum(r['vehicleID'] not in truth for r in rows),'missingPredictionTripinfo':sum(r['vehicleID'] not in predxml for r in rows),'predDurationMismatchGt001':sum(abs(r['pred']-predxml.get(r['vehicleID'],{}).get('predDuration',r['pred']))>.01 for r in rows)}
    basefields=['vehicleID','group','depart','predDuration','truthDuration','durationError','absDurationError','relativeDurationError','truthArrival','predArrival','truthWaitingTime','truthTimeLoss','routeHash','routePreview','firstEdge','lastEdge','edgeCount','exposedModifiedSignals','exposedAllSignals','modifiedSignalCount','firstModifiedSignal','candidateCauseLabel']
    def vehout(rs):
        return [dict(x, predDuration=x['pred'], truthDuration=x['truth'], durationError=x['err'], absDurationError=x['abs'], relativeDurationError=x['rel']) for x in rs]
    write(out/f'{ds}_top_undercliff_vehicles.csv',vehout(sorted([r for r in rows if r['underCliff']],key=lambda r:r['err'])[:args.top_n]),basefields)
    write(out/f'{ds}_top_overcliff_vehicles.csv',vehout(sorted([r for r in rows if r['overCliff']],key=lambda r:-r['err'])[:args.top_n]),basefields)
    write(out/f'{ds}_top_abs_error_vehicles.csv',vehout(sorted(rows,key=lambda r:-r['abs'])[:args.top_n]),basefields)
    route_rows=[]
    for (g,h),rs in collections.defaultdict(list,{}).items(): pass
    buckets=collections.defaultdict(list)
    for r in rows:
        buckets[(r['group'],r['routeHash'])].append(r); buckets[('ALL',r['routeHash'])].append(r)
    for (g,h),rs in buckets.items():
        s=stats(rs); top=max(rs,key=lambda r:r['abs']); ex=sum(1 for r in rs if r['exposedModifiedSignals'])
        route_rows.append({**s,'group':g,'routeHash':h,'routePreview':rs[0]['routePreview'],'firstEdge':rs[0]['firstEdge'],'lastEdge':rs[0]['lastEdge'],'edgeCount':rs[0]['edgeCount'],'topAbsErrorVehicleID':top['vehicleID'],'topAbsErrorPredDuration':top['pred'],'topAbsErrorTruthDuration':top['truth'],'topAbsErrorSignedError':top['err'],'departMin':min(r['depart'] for r in rs),'departMax':max(r['depart'] for r in rs),'exposedModifiedSignals':';'.join(sorted(set(';'.join(r['exposedModifiedSignals'] for r in rs).split(';'))-{''})),'modifiedSignalExposureRate':ex/len(rs)})
    rfields=['group','routeHash','routePreview','firstEdge','lastEdge','edgeCount','vehicleCount','meanPredDuration','meanTruthDuration','bias','MAE','RMSE','medianAbsError','p90AbsError','p95AbsError','maxAbsError','underCount','overCount','underCliffCount','overCliffCount','underNearCliffCount','overNearCliffCount','severeAbsErrorGt1000Count','topAbsErrorVehicleID','topAbsErrorPredDuration','topAbsErrorTruthDuration','topAbsErrorSignedError','departMin','departMax','exposedModifiedSignals','modifiedSignalExposureRate']
    write(out/f'{ds}_top_route_patterns_by_group.csv',sorted(route_rows,key=lambda x:(x['group'],-x['MAE'])),rfields)
    write(out/f'{ds}_top_undercliff_route_patterns.csv',sorted([r for r in route_rows if r['underCliffCount']],key=lambda x:(-x['underCliffCount'],-x['MAE']))[:100],rfields)
    write(out/f'{ds}_top_overcliff_route_patterns.csv',sorted([r for r in route_rows if r['overCliffCount']],key=lambda x:(-x['overCliffCount'],-x['MAE']))[:100],rfields)
    write(out/f'{ds}_top_mae_route_patterns.csv',sorted([r for r in route_rows if r['vehicleCount']>=5],key=lambda x:-x['MAE'])[:100],rfields)
    binrows=[]; bb=collections.defaultdict(list)
    for r in rows: bb[(r['group'],int(r['depart']//300)*300)].append(r)
    for (g,b),rs in bb.items():
        s=stats(rs); top=max(rs,key=lambda r:r['abs']); binrows.append({**s,'group':g,'departBinStart':b,'departBinEnd':b+300,'topAbsErrorVehicleID':top['vehicleID']})
    bfields=['group','departBinStart','departBinEnd','vehicleCount','meanPredDuration','meanTruthDuration','bias','MAE','RMSE','underCount','overCount','underCliffCount','overCliffCount','underNearCliffCount','overNearCliffCount','severeAbsErrorGt1000Count','topAbsErrorVehicleID']
    write(out/f'{ds}_depart_time_error_bins.csv',sorted(binrows,key=lambda x:(x['group'],x['departBinStart'])),bfields)
    for g in ['oversat_all','west_bottleneck','southwest_bottleneck','north_bottleneck']: write(out/f'{ds}_{g}_depart_bins.csv',[r for r in sorted(binrows,key=lambda x:x['departBinStart']) if r['group']==g],bfields)
    grows=[]
    for g,rs in collections.defaultdict(list,{}).items(): pass
    gb=collections.defaultdict(list)
    for r in rows: gb[r['group']].append(r); gb['ALL'].append(r)
    for g,rs in gb.items():
        s=stats(rs); grows.append({**s,'group':g,'vehicleCount':len(rs),'modifiedSignalExposureRate':sum(bool(r['exposedModifiedSignals']) for r in rs)/len(rs),'pctVehiclesExposedToJ17':sum('J17' in r['exposedModifiedSignals'] for r in rs)/len(rs),'pctVehiclesExposedToJ21':sum('J21' in r['exposedModifiedSignals'] for r in rs)/len(rs),'pctVehiclesExposedToJ22':sum('J22' in r['exposedModifiedSignals'] for r in rs)/len(rs),'avgModifiedSignalCount':sum(r['modifiedSignalCount'] for r in rs)/len(rs)})
    efields=['group','vehicleCount','MAE','bias','underCliffCount','overCliffCount','modifiedSignalExposureRate','avgModifiedSignalCount','pctVehiclesExposedToJ17','pctVehiclesExposedToJ21','pctVehiclesExposedToJ22']
    write(out/f'{ds}_signal_offset_exposure_by_group.csv',sorted(grows,key=lambda x:x['group']),efields)
    write(out/f'{ds}_signal_offset_exposure_by_route.csv',route_rows,rfields)
    assoc=[]
    for g,rs in gb.items():
        ex=[r for r in rs if r['exposedModifiedSignals']]; nx=[r for r in rs if not r['exposedModifiedSignals']]
        assoc.append({'group':g,'exposedCount':len(ex),'nonExposedCount':len(nx),'exposedMAE':stats(ex)['MAE'] if ex else '','nonExposedMAE':stats(nx)['MAE'] if nx else '','exposedUnderCliffRate':sum(r['underCliff'] for r in ex)/len(ex) if ex else '', 'nonExposedUnderCliffRate':sum(r['underCliff'] for r in nx)/len(nx) if nx else '', 'exposedOverCliffRate':sum(r['overCliff'] for r in ex)/len(ex) if ex else '', 'nonExposedOverCliffRate':sum(r['overCliff'] for r in nx)/len(nx) if nx else ''})
    write(out/f'{ds}_modified_signal_cliff_association.csv',assoc,['group','exposedCount','nonExposedCount','exposedMAE','nonExposedMAE','exposedUnderCliffRate','nonExposedUnderCliffRate','exposedOverCliffRate','nonExposedOverCliffRate'])
    phasefields=['scope','vehicleID','group','routeHash','routePreview','depart','modifiedSignals','firstModifiedSignal','firstModifiedSignalDepartState','cycleLength','offset']
    topveh=sorted(rows,key=lambda r:-r['abs'])[:args.top_n]
    phase=[{'scope':'vehicle','vehicleID':r['vehicleID'],'group':r['group'],'routeHash':r['routeHash'],'routePreview':r['routePreview'],'depart':r['depart'],'modifiedSignals':r['exposedModifiedSignals'],'firstModifiedSignal':r['firstModifiedSignal'],'firstModifiedSignalDepartState':r['firstModifiedSignalDepartState'],'cycleLength':tls.get(r['firstModifiedSignal'],{}).get('cycle',''),'offset':tls.get(r['firstModifiedSignal'],{}).get('offset','')} for r in topveh]
    write(out/f'{ds}_initial_phase_exposure_top_vehicles.csv',phase,phasefields)
    phase2=[]
    for rr in sorted(route_rows,key=lambda r:-r['MAE'])[:args.top_n]: phase2.append({'scope':'route','vehicleID':rr['topAbsErrorVehicleID'],'group':rr['group'],'routeHash':rr['routeHash'],'routePreview':rr['routePreview'],'depart':rr['departMin'],'modifiedSignals':rr['exposedModifiedSignals'],'firstModifiedSignal':'','firstModifiedSignalDepartState':'','cycleLength':'','offset':''})
    write(out/f'{ds}_initial_phase_exposure_top_routes.csv',phase2,phasefields)
    rec=vehout(sorted([r for r in rows if r['group'] in {'oversat_all','west_bottleneck','southwest_bottleneck','north_bottleneck'}],key=lambda r:-r['abs'])[:20]); write(out/f'{ds}_recommended_trace_vehicles.csv',rec,basefields)
    print(f'Wrote compact diagnosis CSV outputs to {out}')
if __name__=='__main__': main()
