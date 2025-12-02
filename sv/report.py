import json, pathlib

def make_report(run_id: str, out: str):
    if run_id == "last":
        arts = sorted(pathlib.Path("artifacts").glob("RUN-*/outcome.json"))
        assert arts, "no runs yet"
        f = arts[-1]
    else:
        f = pathlib.Path("artifacts")/run_id/"outcome.json"
    outcome = json.loads(f.read_text())
    outdir = pathlib.Path(out); outdir.mkdir(exist_ok=True)
    html = f"""
    <html><body>
    <h1>Run {outcome['run_id']}</h1>
    <pre>{json.dumps(outcome, indent=2)}</pre>
    </body></html>"""
    (outdir/"report.html").write_text(html)
    print("[REPORT]", outdir/"report.html")