# sbsvf

- Lint
  - python -m sv.cli lint scenarios/tw_urban_cutin_v1
- Run
  - python -m sv.cli run plans/smoke.yml
    - 觀察 artifacts/RUN-xxxx/outcome.json
- Report
  - python -m sv.cli report --out out
    xdg-open out/report.html
