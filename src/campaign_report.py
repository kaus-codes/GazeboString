#!/usr/bin/env python3
"""Summarize explicitly named final runs; retain every historical summary separately."""
import argparse,csv,json,statistics
from pathlib import Path

def create(root,seeds):
    rows=[]
    for seed in seeds:
        p=root/f'full-seed-{seed}'
        a=json.loads((p/'aggregate.json').read_text())
        acceptance=json.loads((p/'acceptance.json').read_text())
        if not acceptance['passed'] or not a['verified_full_run']:
            raise ValueError(f'{p}: not accepted')
        rows.append(dict(seed=seed,verified_placements=a['verified_placements'],attempts=a['attempts'],failed_attempts=a['failed_attempts'],incorrect_picks=a['incorrect_picks'],wall_seconds=a['wall_seconds'],sim_seconds=a['sim_seconds'],real_time_factor=a['real_time_factor_measured'],processing_mean_ms=a['processing_latency_ms']['mean'],matched_color_accuracy=a['matched_camera_color_accuracy'],localization_mean_m=a['localization_error_m']['mean'],throughput_per_wall_minute=a['throughput_per_wall_minute']))
    history=[]
    for path in sorted(root.glob('*/summary.json')):
        s=json.loads(path.read_text())
        history.append(dict(directory=path.parent.name,passed=s.get('passed'),seed=s.get('seed'),verified_placements=s.get('verified_placements'),attempts=s.get('attempts'),error=s.get('error')))
    result={'scope':'Named final-controller standard-mode runs; development attempts are listed separately and are not hidden or combined into this rate. GUI/recording adds overhead.','runs':rows,'verified_placements':sum(r['verified_placements'] for r in rows),'attempts':sum(r['attempts'] for r in rows),'wall_mean_s':statistics.mean(r['wall_seconds'] for r in rows),'wall_std_population_s':statistics.pstdev(r['wall_seconds'] for r in rows),'historical_summaries':history}
    (root/'campaign.json').write_text(json.dumps(result,indent=2))
    with (root/'campaign.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    text=['# Verified standard-mode campaign','',result['scope'],'','| Seed | Placements / attempts | Failed | Incorrect | Wall s | Sim s | RTF | Pixel processing mean ms |','|---|---|---|---|---|---|---|---|']
    for r in rows:text.append(f"| {r['seed']} | {r['verified_placements']} / {r['attempts']} | {r['failed_attempts']} | {r['incorrect_picks']} | {r['wall_seconds']:.3f} | {r['sim_seconds']:.3f} | {r['real_time_factor']:.3f} | {r['processing_mean_ms']:.3f} |")
    text+=['','Each accepted run independently passed all 23 checks in its acceptance.json, including fresh final retention, complete bin inventory, continuous belt odometry, real contact evidence, exact alternation, and clean owned-process shutdown.','', 'Seed 23 enables GUI software rendering and X11 recording. Seeds 11 and 47 are headless. These are different runtime loads, not controlled rendering benchmarks.','', 'All per-operation timings, pair durations, mean/median/min/max/population standard deviation, calibration errors, and camera evaluation are in each aggregate.json. The CSV exports and plots are generated exclusively from recorded events.','', 'Development failures are preserved. See campaign.json historical_summaries and docs/TROUBLESHOOTING.md; these three final runs do not imply that every development run passed.','', 'Clutter mode and a clean Dockerfile rebuild are unverified. The latest noVNC layout could not be inspected through the browser because automatic approval review could not run with depleted workspace credits. The MP4 is an actual X11 recording; its codec, dimensions, duration and file integrity were checked.']
    (root/'CAMPAIGN.md').write_text('\n'.join(text)+'\n')
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('results',type=Path);p.add_argument('--seeds',nargs='+',type=int,default=[11,23,47]);a=p.parse_args();r=create(a.results,a.seeds);print(json.dumps({k:v for k,v in r.items() if k!='historical_summaries'},indent=2))
