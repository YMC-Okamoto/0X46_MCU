import argparse
import re
from pathlib import Path
from collections import defaultdict
from datetime import datetime

INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)

def norm_id(s: str) -> str:
    return re.sub(r'[^0-9A-Za-z_]', '_', s)

def get_module(rel_path: Path) -> str:
    parts = rel_path.parts
    if "src" in parts and "code" in parts:
        i = parts.index("code")
        if i + 1 < len(parts):
            return parts[i + 1]
    return rel_path.parent.name if rel_path.parent.name else "ROOT"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="workspace root")
    ap.add_argument("--out", required=True, help="output markdown file")
    ap.add_argument("--max-edges", type=int, default=200)
    args = ap.parse_args()

    root = Path(args.root).resolve()
    files = list(root.rglob("*.c")) + list(root.rglob("*.h")) + list(root.rglob("*.hpp"))
    if not files:
        raise SystemExit("No C/C++ files found.")

    # basename -> modules that own that header
    header_owner = defaultdict(set)
    for f in files:
        if f.suffix.lower() in (".h", ".hpp"):
            rel = f.relative_to(root)
            header_owner[f.name].add(get_module(rel))

    edge_count = defaultdict(int)
    modules = set()

    for f in files:
        rel = f.relative_to(root)
        src_mod = get_module(rel)
        modules.add(src_mod)

        try:
            text = f.read_text(encoding="cp932", errors="ignore")
        except Exception:
            text = f.read_text(encoding="utf-8", errors="ignore")

        for inc in INCLUDE_RE.findall(text):
            inc_name = Path(inc).name
            dst_mods = header_owner.get(inc_name, set())
            for dst_mod in dst_mods:
                modules.add(dst_mod)
                if dst_mod != src_mod:
                    edge_count[(src_mod, dst_mod)] += 1

    # sort edges by weight
    edges = sorted(edge_count.items(), key=lambda x: x[1], reverse=True)[: args.max_edges]

    lines = []
    lines.append("mermaid")
    lines.append("flowchart LR")
    for m in sorted(modules):
        lines.append(f'    {norm_id(m)}["{m}"]')
    for (s, d), w in edges:
        lines.append(f'    {norm_id(s)} -->|{w}| {norm_id(d)}')

    md = []
    md.append(f"# Module Block Diagram")
    md.append(f"- Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    md.append(f"- Root: `{root}`")
    md.append("")
    md.append("````mermaid")
    md.extend(lines[1:])  # skip first "mermaid" word
    md.append("````")
    md.append("")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(md), encoding="cp932", errors="ignore")
    print(f"Generated: {out}")

if __name__ == "__main__":
    main()