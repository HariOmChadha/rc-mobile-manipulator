"""Scan Git blobs for common credentials without printing their values (stdlib only)."""

import argparse
import re
import subprocess
from pathlib import Path, PurePosixPath


RULES = {
    "private key": re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH |DSA |ENCRYPTED )?PRIVATE KEY-----"),
    "Google API key": re.compile(rb"AIza[0-9A-Za-z_-]{35}"),
    "GitHub token": re.compile(rb"(?:gh[pousr]_[A-Za-z0-9]{36,}|github_pat_[A-Za-z0-9_]{40,})"),
    "AWS access key": re.compile(rb"(?:AKIA|ASIA)[A-Z0-9]{16}"),
    "Slack token": re.compile(rb"xox[baprs]-[A-Za-z0-9-]{20,}"),
    "literal credential": re.compile(
        rb'''(?i)["']?(?:pi_api_key|api_key|api_secret|robot_token|access_token|client_secret|password)["']?\s*[:=]\s*["']([A-Za-z0-9_+/=.\-]{24,})["']'''
    ),
    "dotenv PI key": re.compile(rb"(?m)^PI_API_KEY[ \t]*=[ \t]*[A-Za-z0-9_+/=.\-]{24,}[ \t]*$"),
}
PLACEHOLDERS = {b"replace-with-the-same-random-secret-on-both-computers"}


def private_path(name):
    path = PurePosixPath(name)
    base = path.name.lower()
    return (
        (base == ".env" or base.startswith(".env.")) and base != ".env.example"
        or base == ".robot-token"
        or base.endswith((".local.json", ".pem", ".key", ".p12", ".pfx"))
        or bool({"credentials", "secrets", "calibration", "training_dataset", "data", "work"} & set(path.parts))
        or "service-account" in base
        or "sa-key" in base
        or base.startswith("pi-ready") and ".tar.gz" in base
    )


def findings(name, content, *, check_path=True):
    result = ["private file path"] if check_path and private_path(name) else []
    for label, pattern in RULES.items():
        for match in pattern.finditer(content):
            if label == "literal credential" and match.group(1) in PLACEHOLDERS:
                continue
            result.append(label)
            break
    return result


def git(*args):
    return subprocess.check_output(["git", *args])


def blobs(*, staged=False, history=False):
    if history:
        # Historical path names are not a publish guard: scan content, including deleted blobs.
        objects = git("rev-list", "--objects", "--all").decode().splitlines()
        for item in objects:
            oid, _, name = item.partition(" ")
            if git("cat-file", "-t", oid).strip() == b"blob":
                yield f"{oid[:12]}:{name}", git("cat-file", "blob", oid)
    elif staged:
        for item in git("ls-files", "--stage", "-z").decode().split("\0"):
            if item:
                header, name = item.split("\t", 1)
                _, oid, _ = header.split()
                yield name, git("cat-file", "blob", oid)
    else:
        for raw in git("ls-files", "-z").split(b"\0"):
            if raw:
                path = Path(raw.decode())
                if path.is_file():
                    yield str(path), path.read_bytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--staged", action="store_true", help="Read the full Git index, not working copies")
    group.add_argument("--history", action="store_true", help="Scan content of all reachable historical blobs")
    args = parser.parse_args()
    count, issues = 0, 0
    for name, content in blobs(staged=args.staged, history=args.history):
        count += 1
        for reason in findings(name, content, check_path=not args.history):
            print(f"BLOCKED: {name}: {reason}")
            issues += 1
    print(f"Scanned {count} files/blobs; {issues} potential credential/private-file findings.")
    return bool(issues)


if __name__ == "__main__":
    raise SystemExit(main())
