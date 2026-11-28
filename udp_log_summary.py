"""按自动发现的主从会话离线核对收发、缺失以及收到但未采用的命令。"""

import argparse
import json
from pathlib import Path


def missing_ranges(received, final):
    """用区间表示缺失，避免按 uint64 序号范围逐个展开。"""
    start = 1
    missing = []
    for seq in sorted(received):
        if seq > final:
            break
        if seq > start:
            missing.append([start, seq - 1])
        start = seq + 1
    if start <= final:
        missing.append([start, final])
    return missing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--master-final', action='append', default=[], metavar='SESSION:SEQ',
                        help='可选，主端日志的会话及最终序号；可多次给出，用于核对尾部缺失')
    args = parser.parse_args()
    final_sequences = dict(tuple(map(int, value.split(':'))) for value in args.master_final)
    meta = json.loads((args.directory / 'metadata.json').read_text())
    received, applied, sent, failed = {}, {}, {}, {}
    counts, rejected = {}, {}
    integrity, events = [], []
    for path in sorted(args.directory.glob('events_*.jsonl')):
        summary_path = path.with_suffix('.summary.json')
        integrity.append(dict(file=path.name, summary=json.loads(summary_path.read_text())
                              if summary_path.exists() else '缺少完整性摘要，可能尚未停止或异常退出'))
        with path.open() as stream:
            for line in stream:
                row = json.loads(line)
                p = row['payload']
                if row['kind'] == 'udp_rejected':
                    reason = p['reason']
                    rejected[reason] = rejected.get(reason, 0) + 1
                elif row['kind'] == 'udp_receive':
                    key = f"{p['master_session']}:{p['kind']}"
                    counter = counts.setdefault(key, {})
                    reason = p['disposition']
                    counter[reason] = counter.get(reason, 0) + 1
                    if p['kind'] == meta['kind'] and p['address'] == meta['remote']:
                        received.setdefault(p['master_session'], set()).add(p['cmd_seq'])
                elif row['kind'] == 'applied':
                    applied.setdefault(p['applied_master_session'], set()).add(p['applied_cmd_seq'])
                elif row['kind'] == 'udp_send':
                    session, seq = p['extension'][2:4]
                    (sent if p['success'] else failed).setdefault(session, set()).add(seq)
                elif row['kind'] in ('master_session_changed', 'slave_session_started', 'udp_link',
                                     'udp_send_error', 'udp_send_recovered'):
                    events.append(dict(event=row['kind'], **p))
    command_reports = []
    for session, seqs in received.items():
        last = final_sequences.get(session, max(seqs))
        used = applied.get(session, set())
        command_reports.append(dict(
            master_session=session, received_unique=len(seqs), first_seen_seq=min(seqs),
            applied_unique=len(used), received_not_applied=sorted(seqs-used) if meta['kind'] == 0 else [],
            missing_ranges=missing_ranges(seqs, last), final_seq=last,
            missing_scope='到主端最终序号' if session in final_sequences else '仅到已见最大序号；起始前未观察到的包与网络丢包须对照主端日志',
        ))
    feedback_reports = [dict(slave_session=session, sent=len(sent.get(session, set())),
                             failed=sorted(failed.get(session, set())),
                             final_seq=max(sent.get(session, set()) | failed.get(session, set())))
                        for session in sent.keys() | failed.keys()]
    print(json.dumps(dict(command_sessions=command_reports, feedback_sessions=feedback_reports,
                          dispositions=counts, rejected=rejected, events=sorted(events,key=lambda e:e['t_event_us']),
                          integrity=integrity), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
