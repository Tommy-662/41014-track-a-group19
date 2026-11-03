"""Prepare actual links without publishing or submitting anything."""
import argparse,json
from pathlib import Path
from urllib.parse import urlparse
ROOT=Path(__file__).resolve().parents[1]
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--repository',required=True); ap.add_argument('--page',required=True); ap.add_argument('--video',required=True); a=ap.parse_args()
    for label,url in vars(a).items():
        parsed=urlparse(url)
        if parsed.scheme!='https' or not parsed.hostname or any(s in url.lower() for s in ['your_account','example.com','placeholder','真实']): ap.error(f'{label}: provide an actual HTTPS URL')
    if urlparse(a.repository).hostname!='github.com': ap.error('repository must be a GitHub repository URL')
    p=ROOT/'project_links.json'; obj=json.loads(p.read_text()); obj.update(repository_url=a.repository,page_url=a.page,video_url=a.video); p.write_text(json.dumps(obj,indent=2)+'\n')
    text=f'''DRAFT - review all rubric requirements before submission. Links have not been checked remotely by this script.

41014 Integrated Project - Track A
Group 19: Enqian Tang (25646922), approved single-student group.
Project page: {a.page}
The project page contains the demonstration video, code repository and code download, alongside the model, sensing, estimation, controller design and quantitative results.
'''
    preparation=ROOT.parent/'提交操作'; preparation.mkdir(exist_ok=True)
    (preparation/'Canvas_submission_DRAFT.txt').write_text(text,encoding='utf-8')
    print('Links saved locally. Rebuild index.html, review requirements, publish, then verify links before Canvas submission.')
if __name__=='__main__': main()
