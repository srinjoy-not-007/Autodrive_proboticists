#!/usr/bin/env python3
"""Scan DockerHub for public RoboRacer / AutoDRIVE competition submissions.

Usage (host, from repo root):

    nice -n 19 taskset -c 28-31 env OMP_NUM_THREADS=1 .venv/bin/python \
        tools/hub_scan.py --out data/research --md docs/research/public_submissions.md

    # cheaper re-run that reuses the raw JSON already on disk:
    ... tools/hub_scan.py --reuse data/research/hub_scan_20260919.json

What it does
------------
1. Runs a broad keyword list through DockerHub's public search API (v3 catalog
   endpoint, with the older v2 endpoint as a fallback -- both unauthenticated).
2. For every repository hit, lists its tags (name, size, last_pushed).
3. Scores each repo/tag by (a) tag names matching the competition tag grammar
   (``*iros*compete*``, ``*iros*practice*``, ``2026-iros-*``, ``*icra*compete*``)
   and (b) ``last_pushed`` falling inside a submission window
   (qualification Sep 5-13 2026, finals Sep 15-20 2026).
4. Writes raw JSON to ``data/research/hub_scan_<date>.json`` and a ranked
   markdown table to ``docs/research/public_submissions.md`` (``--md``).

The API is unauthenticated and rate limited: every call sleeps ``--sleep``
seconds and retries with exponential backoff on HTTP 429/5xx.

Nothing here runs a container. Pull/inspect of candidates is a separate manual
step (``docker manifest inspect``, ``docker create`` + ``docker cp``); never run
a competitor image against the simulator.
"""

from __future__ import annotations

import argparse
import datetime as dt
import fnmatch
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

UA = "hub_scan/1.0 (+roboracer research; contact: repo owner)"

# Broad keyword list. Cheap to extend -- each term is one search call.
QUERIES = [
    "roboracer",
    "autodrive",
    "autodrive_roboracer",
    "roboracer_api",
    "autodrive_devkit",
    "f1tenth",
    "f1-tenth",
    "f1_tenth",
    "iros2026",
    "iros-2026",
    "iros_2026",
    "2026-iros",
    "srl2026",
    "sim racing",
    "racing league",
    "sim racing league",
    "racecar",
    "ros2 racing",
    "ackermann",
    "autonomous racing",
    "tenth scale racing",
]

# Tag-name patterns that mark a competition entry.
TAG_GLOBS = [
    "*iros*compete*",
    "*iros*practice*",
    "2026-iros*",
    "*iros2026*",
    "*iros-2026*",
    "*icra*compete*",
    "*icra*practice*",
    "*2026-icra*",
    "*qualif*",
    "*final*",
    "*compete*",
]

# Submission windows (UTC). Qualification was due Sep 12 2026 AoE, finals Sep 19.
WINDOWS = {
    "qualification": (dt.date(2026, 9, 5), dt.date(2026, 9, 13)),
    "finals": (dt.date(2026, 9, 15), dt.date(2026, 9, 20)),
}

# Repos worth spending a tag-listing call on, and namespaces worth enumerating.
# Deliberately NARROW. The obvious wider pattern (…|racer|racing|iros|srl|
# ackermann) matches "amirosman", "irosh2002", "racingmars", "ackermannj" and a
# few thousand others: the scan then spends hours listing tags for strangers and
# never finishes in a sitting. Broad words still earn points in score(), they
# just do not buy an API call.
NAME_HINTS = re.compile(r"roboracer|autodrive|f1.?tenth|f1_tenth", re.I)
NS_HINTS = NAME_HINTS


def _get(url: str, sleep: float, tries: int = 5):
    """GET JSON with polite backoff. Returns None on permanent failure."""
    delay = sleep
    for attempt in range(tries):
        req = urllib.request.Request(url, headers={"User-Agent": UA, "Accept": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            time.sleep(sleep)
            return data
        except urllib.error.HTTPError as exc:
            if exc.code in (429, 500, 502, 503, 504):
                delay = min(delay * 2, 60)
                sys.stderr.write(f"  HTTP {exc.code} on {url} -> sleeping {delay:.0f}s\n")
                time.sleep(delay)
                continue
            if exc.code == 404:
                return None
            sys.stderr.write(f"  HTTP {exc.code} on {url}\n")
            return None
        except Exception as exc:  # noqa: BLE001 - network is best effort
            sys.stderr.write(f"  {type(exc).__name__} on {url}: {exc}\n")
            time.sleep(delay)
            delay = min(delay * 2, 60)
    return None


def search(query: str, sleep: float, page_size: int = 100, max_pages: int = 3):
    """DockerHub search. v3 catalog endpoint first, v2 as fallback."""
    out = []
    q = urllib.parse.quote(query)
    for page in range(1, max_pages + 1):
        url = (
            f"https://hub.docker.com/api/search/v3/catalog/search"
            f"?query={q}&size={page_size}&from={(page - 1) * page_size}"
        )
        data = _get(url, sleep)
        results = (data or {}).get("results")
        if results is None:
            # fallback: legacy v2 search
            url = (
                f"https://hub.docker.com/v2/search/repositories/"
                f"?query={q}&page_size={page_size}&page={page}"
            )
            data = _get(url, sleep)
            results = (data or {}).get("results") or []
            for r in results:
                out.append(
                    {
                        "name": r.get("repo_name", ""),
                        "description": r.get("short_description", ""),
                        "star_count": r.get("star_count", 0),
                        "pull_count": r.get("pull_count", 0),
                        "query": query,
                    }
                )
            if len(results) < page_size:
                break
            continue
        for r in results:
            ns = (r.get("id") or "")
            out.append(
                {
                    "name": ns,
                    "description": r.get("short_description") or (r.get("description") or "")[:200],
                    "star_count": r.get("star_count", 0),
                    "pull_count": (r.get("pull_count") or {}).get("value", 0)
                    if isinstance(r.get("pull_count"), dict)
                    else r.get("pull_count", 0),
                    "query": query,
                }
            )
        if len(results) < page_size:
            break
    return out


def list_tags(repo: str, sleep: float, page_size: int = 100):
    if "/" not in repo:
        repo = f"library/{repo}"
    ns, name = repo.split("/", 1)
    url = (
        f"https://hub.docker.com/v2/repositories/{urllib.parse.quote(ns)}/"
        f"{urllib.parse.quote(name)}/tags?page_size={page_size}"
    )
    data = _get(url, sleep)
    if not data:
        return []
    tags = []
    for t in data.get("results", []):
        tags.append(
            {
                "tag": t.get("name"),
                "last_pushed": t.get("tag_last_pushed") or t.get("last_updated"),
                "size": t.get("full_size"),
                "digest": t.get("digest"),
            }
        )
    return tags


def list_namespace(ns: str, sleep: float, page_size: int = 100):
    """All public repos under one namespace (users found via a hit)."""
    url = f"https://hub.docker.com/v2/repositories/{urllib.parse.quote(ns)}/?page_size={page_size}"
    data = _get(url, sleep)
    if not data:
        return []
    return [f"{ns}/{r['name']}" for r in data.get("results", [])]


def _parse_date(s):
    if not s:
        return None
    try:
        return dt.datetime.fromisoformat(s.replace("Z", "+00:00")).date()
    except Exception:  # noqa: BLE001
        return None


def score(repo_name: str, tags: list) -> tuple:
    """(score, reasons) -- higher is a more plausible competition entry."""
    pts = 0
    why = []
    low = repo_name.lower()
    if "roboracer" in low:
        pts += 30
        why.append("name:roboracer")
    if "autodrive" in low:
        pts += 20
        why.append("name:autodrive")
    if re.search(r"f1.?tenth", low):
        pts += 15
        why.append("name:f1tenth")
    if "iros" in low:
        pts += 15
        why.append("name:iros")
    if re.search(r"racer|racing", low):
        pts += 5
        why.append("name:racing")

    for t in tags:
        tag = (t.get("tag") or "").lower()
        for glob in TAG_GLOBS:
            if fnmatch.fnmatch(tag, glob):
                bump = 40 if "iros" in glob else 15
                pts += bump
                why.append(f"tag:{tag}")
                break
        d = _parse_date(t.get("last_pushed"))
        if d:
            for wname, (lo, hi) in WINDOWS.items():
                if lo <= d <= hi:
                    pts += 25
                    why.append(f"pushed:{d}({wname})")
                    break
    return pts, sorted(set(why))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="data/research", help="directory for the raw JSON")
    ap.add_argument("--md", default="docs/research/public_submissions.md", help="markdown table out")
    ap.add_argument("--sleep", type=float, default=1.5, help="seconds between API calls")
    ap.add_argument("--min-score", type=int, default=20, help="drop repos scoring below this")
    ap.add_argument("--queries", nargs="*", default=None, help="override the keyword list")
    ap.add_argument("--extra", nargs="*", default=[], help="extra repos to force-inspect (ns/name)")
    ap.add_argument("--reuse", default=None, help="reuse a previous raw JSON instead of searching")
    ap.add_argument("--no-ns", action="store_true", help="skip namespace enumeration")
    ap.add_argument("--no-md", action="store_true")
    args = ap.parse_args()

    if args.reuse:
        with open(args.reuse) as fh:
            blob = json.load(fh)
    else:
        queries = args.queries or QUERIES
        hits = {}
        for q in queries:
            sys.stderr.write(f"search: {q}\n")
            for r in search(q, args.sleep):
                name = r["name"]
                if not name:
                    continue
                if name not in hits:
                    hits[name] = r
                    hits[name]["queries"] = [q]
                else:
                    hits[name]["queries"].append(q)

        # Expand namespaces of promising hits: a team's entry often sits next to
        # unrelated repos under the same user.
        namespaces = sorted(
            {n.split("/")[0] for n in hits if NS_HINTS.search(n)}
        )
        for ns in namespaces:
            if ns in ("library",) or args.no_ns:
                continue
            sys.stderr.write(f"namespace: {ns}\n")
            for name in list_namespace(ns, args.sleep):
                hits.setdefault(name, {"name": name, "description": "", "queries": ["ns:" + ns]})

        for name in args.extra:
            hits.setdefault(name, {"name": name, "description": "", "queries": ["manual"]})

        # Tag listing is the expensive part -- only for name-plausible repos.
        repos = []
        for name, meta in sorted(hits.items()):
            if not NAME_HINTS.search(name) and name not in args.extra:
                repos.append({**meta, "tags": [], "skipped": "name filter"})
                continue
            sys.stderr.write(f"tags: {name}\n")
            tags = list_tags(name, args.sleep)
            repos.append({**meta, "tags": tags})

        blob = {
            "generated": dt.datetime.now(dt.timezone.utc).isoformat(),
            "queries": queries,
            "repos": repos,
        }

        os.makedirs(args.out, exist_ok=True)
        stamp = dt.date.today().strftime("%Y%m%d")
        raw_path = os.path.join(args.out, f"hub_scan_{stamp}.json")
        with open(raw_path, "w") as fh:
            json.dump(blob, fh, indent=1, sort_keys=True)
        sys.stderr.write(f"raw -> {raw_path}\n")

    ranked = []
    for r in blob["repos"]:
        pts, why = score(r["name"], r.get("tags") or [])
        if pts >= args.min_score:
            ranked.append((pts, why, r))
    ranked.sort(key=lambda x: -x[0])

    for pts, why, r in ranked[:60]:
        tags = ", ".join(
            f"{t['tag']}@{(t.get('last_pushed') or '')[:10]}" for t in (r.get("tags") or [])[:8]
        )
        print(f"{pts:4d}  {r['name']:<50} {tags}")
        print(f"      {' '.join(why)}")

    if not args.no_md:
        os.makedirs(os.path.dirname(args.md), exist_ok=True)
        with open(args.md.replace(".md", "_hub_table.md"), "w") as fh:
            fh.write("| score | repo | tags (name@pushed) | size (GB) | why |\n")
            fh.write("|---|---|---|---|---|\n")
            for pts, why, r in ranked:
                tags = "<br>".join(
                    f"`{t['tag']}` {(t.get('last_pushed') or '')[:10]}"
                    for t in (r.get("tags") or [])[:10]
                )
                sizes = [t.get("size") for t in (r.get("tags") or []) if t.get("size")]
                gb = f"{max(sizes) / 1e9:.2f}" if sizes else ""
                fh.write(f"| {pts} | `{r['name']}` | {tags} | {gb} | {' '.join(why)} |\n")
        sys.stderr.write(f"table -> {args.md.replace('.md', '_hub_table.md')}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
