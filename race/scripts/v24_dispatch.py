#!/usr/bin/env python3
"""v24 portable race CLI. Profile-bound map selection; never edits entrant source."""
from __future__ import annotations
import json
import os
from pathlib import Path
import sys

OFFICIAL_STAGES={'qualifying','semi_a','semi_b','final','refresh-maps','show-maps'}
MAP_COMMANDS={'preview','rehearse'}
TRACKS=frozenset({'monza','hungaroring','monaco','spa','cota','baku'})

def load_profile(root:Path)->tuple[str,tuple[str,...]]:
    p=root/'race/v24_profile.json'
    if not p.is_file():raise ValueError('v24 설정 파일 없음: '+str(p))
    d=json.loads(p.read_text(encoding='utf-8'))
    mode=d.get('mode')
    selected=d.get('tracks')
    if mode not in ('practice','official') or not isinstance(selected,list):
        raise ValueError('v24 프로필 형식 오류')
    tr=tuple(selected)
    required={'monza','hungaroring','monaco'} if mode=='practice' else TRACKS
    if set(tr)!=required or len(tr)!=len(required):
        raise ValueError('v24 맵 목록이 배포 버전과 일치하지 않습니다')
    return mode,tr

def option_get(args:list[str],option:str)->str|None:
    result=None
    for i,arg in enumerate(args):
        if arg==option:
            if i+1==len(args) or args[i+1].startswith('--'):
                raise ValueError(option+' 값 누락')
            result=args[i+1]
        elif arg.startswith(option+'='):
            result=arg.split('=',1)[1]
    return result

def selection(value:str|None,available:tuple[str,...])->str:
    if value in (None,'all','practice','2','3','6'):
        return ','.join(available)
    tokens=[x.strip().lower() for x in value.split(',') if x.strip()]
    if not tokens or any(x not in available for x in tokens):
        raise ValueError('이 배포본에 없는 맵을 요청했습니다: '+str(value))
    return ','.join(dict.fromkeys(tokens))

def compatibility_args(args:list[str],available:tuple[str,...])->list[str]:
    # Preserve all other switches, normalize exactly one --tracks occurrence.
    out=[];i=0;used=False
    while i<len(args):
        item=args[i]
        if item=='--tracks':
            if i+1>=len(args):raise ValueError('--tracks 값 누락')
            if used:raise ValueError('--tracks 중복')
            out+=['--tracks',selection(args[i+1],available)];i+=2;used=True
        elif item.startswith('--tracks='):
            if used:raise ValueError('--tracks 중복')
            out+=['--tracks',selection(item.split('=',1)[1],available)];i+=1;used=True
        else:out.append(item);i+=1
    if not used:out+=['--tracks',','.join(available)]
    return out

def route(root:Path,args:list[str])->list[str]:
    if not args:raise ValueError('사용법: bash race_v24.sh preview|rehearse|compatibility|check|sync|prepare|vehicles|results [옵션]')
    mode,available=load_profile(root)
    verb=args[0];extra=args[1:]
    if verb=='compatibility':
        return ['python3',str(root/'race/scripts/compatibility_v23.py'),
                '--root',str(root),*compatibility_args(extra,available)]
    if mode=='practice':
        if verb in OFFICIAL_STAGES:
            raise ValueError('연습용에는 공식 경기/맵 추첨 기능이 없습니다. rehearse --track <연습맵>을 사용하십시오')
        if verb in MAP_COMMANDS:
            tr=option_get(extra,'--track')
            if tr is None:
                raise ValueError('연습용 preview/rehearse는 --track monza|hungaroring|monaco 명시 필수')
            if tr not in available:
                raise ValueError('연습용에서 사용할 수 없는 맵: '+tr)
        if verb not in {'preview','rehearse','compatibility','check','sync','prepare','vehicles','results','summary','help','--help'}:
            raise ValueError('연습용에서 허용되지 않는 명령: '+verb)
        return ['bash',str(root/'race_v22.sh'),*args]
    if verb in MAP_COMMANDS:
        tr=option_get(extra,'--track')
        if tr is not None and tr not in available:
            raise ValueError('지원하지 않는 맵: '+tr)
    if verb in OFFICIAL_STAGES and verb not in {'refresh-maps','show-maps'}:
        tr=option_get(extra,'--track')
        if tr is not None and tr not in available:
            raise ValueError('지원하지 않는 공식 맵: '+tr)
    return ['bash',str(root/'race_v23.sh'),*args]

def main(argv:list[str]|None=None,root:Path|None=None)->int:
    argv=list(sys.argv[1:] if argv is None else argv)
    root=(root or Path(__file__).resolve().parents[2]).resolve()
    try:
        cmd=route(root,argv)
    except (ValueError,OSError,json.JSONDecodeError) as e:
        print('[v24] 실행 차단:',e,file=sys.stderr)
        return 2
    os.execvp(cmd[0],cmd)
    return 0
if __name__=='__main__':sys.exit(main())
