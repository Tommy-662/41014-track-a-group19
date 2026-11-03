"""Build a self-contained static GitHub project page from maintained Markdown."""
import json,html,re
from urllib.parse import urlparse,parse_qs
from pathlib import Path
import markdown
ROOT=Path(__file__).resolve().parents[1]
def main():
    links=json.loads((ROOT/'project_links.json').read_text()); sections=[]
    for key,title,path in [('overview','Overview','README.md'),('design','Model, frames & design','docs/model-and-design.md'),('results','Quantitative results','docs/results.md'),('discussion','Discussion','docs/discussion.md'),('contribution','Individual contribution','docs/contributions.md')]:
        source=(ROOT/path).read_text(encoding='utf-8')
        if path.startswith('docs/'):
            source=re.sub(r'\]\(\.\./results/','](results/',source)
            source=source.replace('](frames.svg)','](docs/frames.svg)')
            source=re.sub(r'\]\((?!https?://|#|results/)([^)]+\.md)\)',r'](docs/\1)',source)
        body=markdown.markdown(source,extensions=['tables','fenced_code','toc'])
        sections.append(f'<section id="{key}"><div class="section-label">{html.escape(title)}</div>{body}</section>')
    actual=[]
    for key,label in [('repository_url','Code repository'),('video_url','Five-minute video')]:
        if links[key]: actual.append(f'<a href="{html.escape(links[key],quote=True)}">{label}</a>')
    linkbar=' | '.join(actual) or 'Repository and video links pending publication'
    notice='' if links['video_url'] else '<div class="notice">The demonstration video will be linked here after upload.</div>'
    demonstration=''
    if links['video_url'] and links['repository_url']:
        parsed=urlparse(links['video_url'])
        video_id=parsed.path.strip('/').split('/')[0] if parsed.hostname=='youtu.be' else parse_qs(parsed.query).get('v',[''])[0]
        if not re.fullmatch(r'[A-Za-z0-9_-]{11}',video_id): raise ValueError('Expected a valid YouTube video URL')
        repo=html.escape(links['repository_url'].rstrip('/'),quote=True)
        demonstration=f'''<section id="demonstration"><h2>Demonstration and code</h2>
        <iframe class="demo-video" src="https://www.youtube-nocookie.com/embed/{video_id}" title="Group 19 — Track A five-minute demonstration" allow="encrypted-media; picture-in-picture; fullscreen" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
        <p>The five-minute video shows the nominal mission, a camera-observation dropout trial, uncertainty and quantitative results.</p>
        <p><a href="{repo}">View code repository</a> · <a href="{repo}/archive/refs/heads/main.zip">Download complete code and results (ZIP)</a> · <a href="{repo}/blob/main/run_sim.py">View simulation source</a></p>
        <p><a href="{html.escape(links['video_url'],quote=True)}">Open video on YouTube</a></p></section>'''
    page='''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Group 19 | Track A | 41014</title><style>
    *{box-sizing:border-box}body{margin:0;color:#28363e;background:white;font:16px/1.8 Georgia,"Times New Roman",serif}a{color:#185f83}header,main{max-width:1040px;margin:auto;padding:32px 36px}header{border-bottom:2px solid #344b57;padding-bottom:24px}header .meta{font:14px/1.7 Arial,sans-serif;color:#58676e}header h1{font-size:34px;line-height:1.3;max-width:850px;margin:18px 0}header p{margin:10px 0;color:#536169}.linkbar{font:14px/1.7 Arial,sans-serif}nav{padding:12px 20px;text-align:center;border-bottom:1px solid #dbe2e5;font:14px/1.7 Arial,sans-serif}nav a{display:inline-block;margin:3px 12px}section{padding:18px 0 32px;border-bottom:1px solid #dbe2e5;overflow-wrap:anywhere}.section-label{font:13px Arial,sans-serif;color:#62747c;margin:20px 0}h1{font-size:28px;line-height:1.35}h2{font-size:23px;margin-top:35px}h3{font-size:19px}pre{background:#f4f6f7;padding:18px;overflow:auto;font:13px/1.7 Consolas,monospace}code{font-family:Consolas,monospace}img{max-width:100%;height:auto;display:block;margin:24px auto}table{display:block;overflow:auto;border-collapse:collapse;font:14px/1.7 Arial,sans-serif;margin:24px 0}th,td{padding:10px;border:1px solid #ced8dd;text-align:left;vertical-align:top}th{background:#f0f4f6}.notice{border-left:3px solid #b3bec4;background:#f7f8f9;padding:12px 18px;font:14px/1.8 Arial,sans-serif}footer{text-align:center;color:#62737b;font-size:13px;padding:26px}@media(max-width:600px){header,main{padding:22px 18px}header h1{font-size:28px}nav a{margin:3px 8px}body{font-size:16px}}
    </style></head><body><header><div class="meta">UTS 41014 · Integrated Project · Group 19</div><h1>Visual waypoint navigation and ArUco docking</h1><p>Enqian Tang · 25646922 · Approved solo team · Simulation only</p><div class="linkbar">LINKBAR</div></header><nav><a href="#overview">Overview</a><a href="#design">Model & design</a><a href="#results">Results</a><a href="#discussion">Discussion</a><a href="#contribution">Contribution</a></nav><main>NOTICESECTIONS</main><footer>Three-waypoint validation: 3 November 2026 · No hardware claim · Raw logs and source hashes included</footer></body></html>'''
    page=page.replace('</style>','.demo-video{display:block;width:100%;aspect-ratio:16/9;border:0;background:#f3f5f6}</style>').replace('<main>NOTICESECTIONS','<main>NOTICEDEMONSTRATIONSECTIONS')
    (ROOT/'index.html').write_text(page.replace('NOTICE',notice).replace('DEMONSTRATION',demonstration).replace('LINKBAR',linkbar).replace('SECTIONS','\n'.join(sections)),encoding='utf-8')
    (ROOT/'.nojekyll').touch()
    print('Built index.html from Markdown. Publication and remote access remain unverified.')
if __name__=='__main__': main()
