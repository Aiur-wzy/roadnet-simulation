"""Plan a recoverable source-layout deployment; no SSH, credentials or experiments.

Run this standard-library tool on the host being inspected. The bundle JSON has
``mapping`` (old -> new), ``files`` (payload relative path -> SHA256), and
``known_previous`` (relative path -> list of previously reviewed SHA256 values).
Only explicit source paths are inspected. Unknown remote modifications block a
plan; add an exact hash to known_previous only after reviewing that file.

inventory and plan are read-only except an explicitly requested new JSON output.
apply is also a dry run unless --execute is supplied. Actual apply requires the
SHA256 of the reviewed plan, fresh file checks and a fresh Linux /proc check.
All changed originals are copied and verified before any original is unlinked.
An interrupted apply retains its plan, original-byte backups and journal; it is
not an automatic rollback/resume tool. Stop new jobs during the maintenance
window: process inspection cannot prevent a job starting immediately afterward.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import sys
from datetime import datetime, timezone


SCHEMA = "remote-source-layout-v1"
PROTECTED = {"data", "train_data", "models", "model", "catch", "eval", "outputs",
             "baselines", "garbage", ".git", ".codex", ".agents", ".aws"}
CODE_DIRS = {"collection", "training", "catching", "features", "data_generation",
             "evaluation", "tools", "legacy", "scripts", "simulator", "tests",
             "vehicle_feature_pipeline"}
SUFFIXES = {".py", ".sh", ".cpp", ".h", ".hpp"}
SPECIAL = {"roadnet_paths.py", "CMakeLists.txt", "docs/source_layout.json",
           "docs/project_layout.md", "docs/manhattan_native_collection.md", "README.md", "AGENTS.md", "PROJECT_MANUAL.md", "cap_context_event_fixture.json",
           "tests/fixtures/cap_context_event_fixture.json", "tests/fixtures/cap_context_abc_rg_mock.rle",
           "tests/fixtures/cap_context_abc_rg_mock_queries.txt"}


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=True, separators=(",", ":")).encode()


def validate_relative(relative, source_only=True):
    if not isinstance(relative, str) or "\\" in relative or ":" in relative:
        raise ValueError("Expected a POSIX relative path")
    parts = PurePosixPath(relative).parts
    if (not parts or relative != "/".join(parts) or parts[0] == "/"
            or any(part in {".", "..", ""} for part in parts)):
        raise ValueError("Unsafe relative path: " + relative)
    if source_only:
        if any(part.startswith(".") for part in parts) or parts[0] in PROTECTED:
            raise ValueError("Protected path: " + relative)
        if relative not in SPECIAL and not (
                Path(relative).suffix in SUFFIXES
                and (len(parts) == 1 or parts[0] in CODE_DIRS)):
            raise ValueError("Not an allowed source path: " + relative)
    return parts


def scoped(root, relative, source_only=True):
    parts = validate_relative(relative, source_only)
    root = Path(root).resolve(strict=True)
    if root == Path(root.anchor) or root == Path.home().resolve():
        raise ValueError("Deployment root must be a specific project directory")
    current = root
    for part in parts:
        current = current / part
        if current.is_symlink():
            raise ValueError("Symlink path refused: " + str(current))
    current.resolve().relative_to(root)
    return current


def record(path):
    path = Path(path)
    if not path.exists():
        return None
    before = path.lstat()
    if not stat.S_ISREG(before.st_mode) or path.is_symlink():
        raise ValueError("Not a regular file: " + str(path))
    result = {"sha256": digest(path), "bytes": before.st_size,
              "mode": stat.S_IMODE(before.st_mode)}
    after = path.stat()
    if (before.st_size, before.st_mtime_ns, before.st_ino) != (
            after.st_size, after.st_mtime_ns, after.st_ino):
        raise ValueError("File changed while hashing: " + str(path))
    return result


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def validate_bundle(bundle):
    if set(bundle) != {"mapping", "files", "known_previous"}:
        raise ValueError("Bundle requires mapping, files and known_previous only")
    for old, new in bundle["mapping"].items():
        validate_relative(old)
        validate_relative(new)
        if old == new or new not in bundle["files"]:
            raise ValueError("Mapping target missing from payload: " + new)
    if len(set(bundle["mapping"].values())) != len(bundle["mapping"]):
        raise ValueError("Many-to-one source mapping is not supported")
    if set(bundle["mapping"]) & set(bundle["files"]):
        raise ValueError("Legacy source is also a payload target")
    for path, sha in bundle["files"].items():
        validate_relative(path)
        if not isinstance(sha, str) or not re.fullmatch("[0-9a-f]{64}", sha):
            raise ValueError("Invalid payload hash: " + path)
    for path, hashes in bundle["known_previous"].items():
        validate_relative(path)
        if not isinstance(hashes, list) or not all(
                isinstance(sha, str) and re.fullmatch("[0-9a-f]{64}", sha) for sha in hashes):
            raise ValueError("Invalid previously reviewed hashes: " + path)


def current_uid_name():
    import pwd
    return pwd.getpwuid(os.getuid()).pw_name


def transport_service_kind(command, comm, root):
    """Recognize only reviewed, single-title authentication/transport services."""
    arguments = command.rstrip(b"\0").split(b"\0")
    if len(arguments) != 1:
        return None
    title = arguments[0].decode(errors="replace").rstrip(" ")
    if root in title:
        return None
    if title == "/usr/lib/openssh/sftp-server" and comm == "sftp-server":
        return "openssh-sftp-server"
    if title == "(sd-pam)" and comm == "(sd-pam)":
        return "systemd-pam"
    if comm == "sshd" and title.startswith("sshd: "):
        # Bind to this UID's name, not an arbitrary account or command suffix.
        pattern = r"sshd: " + re.escape(current_uid_name()) + r"@(?:notty|pts/[0-9]+)"
        if re.fullmatch(pattern, title):
            return "openssh-session"
    return None


def process_evidence(root):
    """Inspect same-UID processes, returning scoped evidence without unrelated argv."""
    if sys.platform != "linux":
        return {"complete": False, "blockers": [{"reason": "Linux /proc required"}]}
    root = str(Path(root).resolve())
    blockers, errors, excluded = [], [], []
    for directory in sorted(Path("/proc").iterdir()):
        if not directory.name.isdigit() or int(directory.name) == os.getpid():
            continue
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            command = (directory / "cmdline").read_bytes()
            argv = command.replace(b"\0", b" ").decode(errors="replace")
            if not argv:
                continue
            links, denied = {}, []
            for name in ("cwd", "exe"):
                try:
                    links[name] = os.readlink(directory / name)
                except PermissionError:
                    denied.append(name)
            references_root = root in argv or any(
                value == root or value.startswith(root + "/") for value in links.values())
            command_sha = hashlib.sha256(argv.encode()).hexdigest()
            if references_root:
                # Do not publish argument values; they can contain credentials.
                blockers.append({"pid": int(directory.name), "cwd": links.get("cwd"),
                                 "executable": links.get("exe"), "command_sha256": command_sha,
                                 "reason": "process command/cwd/executable references deployment root"})
            if denied:
                service = None
                if not references_root:
                    comm = (directory / "comm").read_bytes().decode(errors="replace").rstrip("\n")
                    service = transport_service_kind(command, comm, root)
                if service:
                    excluded.append({"pid": int(directory.name), "service": service,
                                     "unreadable_links": denied, "command_sha256": command_sha,
                                     "reason": "reviewed system transport identity; only cwd/exe permission denied"})
                else:
                    errors.append({"pid": int(directory.name), "reason": "PermissionError",
                                   "unreadable_links": denied})
        except (FileNotFoundError, ProcessLookupError):
            continue
        except OSError as error:
            errors.append({"pid": int(directory.name), "reason": type(error).__name__})
        except KeyError:
            errors.append({"pid": int(directory.name), "reason": "current UID name unavailable"})
    return {"complete": not errors, "blockers": blockers, "inspection_errors": errors,
            "excluded_transport_services": excluded}


def inventory(root, bundle):
    validate_bundle(bundle)
    root = Path(root).resolve(strict=True)
    paths = sorted(set(bundle["mapping"]) | set(bundle["files"]))
    return {"schema": SCHEMA, "root": str(root),
            "bundle_sha256": hashlib.sha256(canonical(bundle)).hexdigest(),
            "files": {path: record(scoped(root, path)) for path in paths},
            "processes": process_evidence(root),
            "boundary": "Explicit source files only; models/data/unmapped files are untouched."}


def build_plan(root, payload, bundle, observed, archive_name):
    validate_bundle(bundle)
    if not re.fullmatch("[A-Za-z0-9][A-Za-z0-9_-]{0,79}", archive_name):
        raise ValueError("Invalid archive name")
    root, payload = Path(root).resolve(strict=True), Path(payload).resolve(strict=True)
    if root == payload or root in payload.parents:
        raise ValueError("Payload must be outside the deployment root")
    if observed["root"] != str(root) or observed["bundle_sha256"] != hashlib.sha256(canonical(bundle)).hexdigest():
        raise ValueError("Inventory root/bundle mismatch")
    blockers, backups, installs = [], [], []
    before = observed["files"]
    expected_paths = set(bundle["mapping"]) | set(bundle["files"])
    if set(before) != expected_paths:
        raise ValueError("Inventory path set mismatch")
    for path, sha in sorted(bundle["files"].items()):
        staged = record(scoped(payload, path))
        if staged is None or staged["sha256"] != sha:
            raise ValueError("Payload hash mismatch: " + path)
        prior = before[path]
        if prior is not None and prior["sha256"] == sha:
            continue
        if prior is not None:
            if prior["sha256"] not in bundle["known_previous"].get(path, []):
                blockers.append({"path": path, "reason": "unknown remote target modification", "sha256": prior["sha256"]})
            backups.append(path)
        installs.append({"path": path, "record": staged})
    for old in sorted(bundle["mapping"]):
        prior = before[old]
        if prior is None:
            continue
        if prior["sha256"] not in bundle["known_previous"].get(old, []):
            blockers.append({"path": old, "reason": "unknown remote legacy modification", "sha256": prior["sha256"]})
        backups.append(old)
    if not observed["processes"]["complete"]:
        blockers.append({"reason": "incomplete process inspection"})
    blockers.extend(observed["processes"]["blockers"])
    archive = "garbage/" + archive_name
    if scoped(root, archive, False).exists():
        blockers.append({"reason": "archive already exists", "path": archive})
    return {"schema": SCHEMA, "root": str(root), "payload": str(payload),
            "archive": archive, "bundle": bundle, "before": before,
            "backups": sorted(set(backups)), "installs": installs,
            "blockers": blockers, "inventory_processes": observed["processes"],
            "created_utc": datetime.now(timezone.utc).isoformat()}


def exclusive_copy(source, target, expected):
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as src, target.open("xb") as dst:
        shutil.copyfileobj(src, dst, 1024 * 1024)
        dst.flush()
        os.fsync(dst.fileno())
    os.chmod(target, expected["mode"])
    if record(target) != expected:
        raise ValueError("Copied file verification failed: " + str(target))


def apply_plan(plan, execute=False):
    root, payload = Path(plan["root"]), Path(plan["payload"])
    current = inventory(root, plan["bundle"])
    rebuilt = build_plan(root, payload, plan["bundle"], current, PurePosixPath(plan["archive"]).name)
    for key in ("before", "backups", "installs", "archive"):
        if rebuilt[key] != plan[key]:
            raise ValueError("Plan drift: " + key)
    if plan["blockers"] or rebuilt["blockers"]:
        return {"status": "blocked", "blockers": plan["blockers"] + rebuilt["blockers"]}
    if not execute:
        return {"status": "dry_run", "archive_files": len(plan["backups"]), "install_files": len(plan["installs"])}
    archive = scoped(root, plan["archive"], False)
    archive.mkdir(parents=True, exist_ok=False)
    with (archive / "plan.json").open("x", encoding="utf-8") as stream:
        json.dump(plan, stream, indent=2)
    with (archive / "journal.jsonl").open("x", encoding="utf-8", buffering=1) as journal:
        def log(action, path):
            journal.write(json.dumps({"action": action, "path": path}) + "\n")
            journal.flush()
            os.fsync(journal.fileno())
        # Finish all verified backups before changing any original source.
        for path in plan["backups"]:
            source = scoped(root, path)
            expected = plan["before"][path]
            if record(source) != expected:
                raise ValueError("Source drift before backup: " + path)
            exclusive_copy(source, scoped(archive, "originals/" + path, False), expected)
            log("backed_up", path)
        # Recheck process evidence after backup work, immediately before edits.
        processes = process_evidence(root)
        if not processes["complete"] or processes["blockers"]:
            raise ValueError("Process dependency appeared; verified backups retained, originals unchanged")
        for path in plan["backups"]:
            source = scoped(root, path)
            if record(source) != plan["before"][path]:
                raise ValueError("Source drift before relocation: " + path)
            source.unlink()
            log("archived_original", path)
        for entry in plan["installs"]:
            path, expected = entry["path"], entry["record"]
            source = scoped(payload, path)
            if record(source) != expected:
                raise ValueError("Payload drift: " + path)
            exclusive_copy(source, scoped(root, path), expected)
            log("installed", path)
        for path, sha in plan["bundle"]["files"].items():
            final = record(scoped(root, path))
            if final is None or final["sha256"] != sha:
                raise ValueError("Final target mismatch: " + path)
        for path in plan["backups"]:
            if record(scoped(archive, "originals/" + path, False)) != plan["before"][path]:
                raise ValueError("Final original backup mismatch: " + path)
        log("complete", str(archive))
    return {"status": "complete", "archive": str(archive),
            "archive_files": len(plan["backups"]), "install_files": len(plan["installs"])}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("inventory", "plan", "apply"), nargs="?", default="inventory")
    parser.add_argument("--root", type=Path)
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--payload", type=Path)
    parser.add_argument("--inventory", type=Path)
    parser.add_argument("--archive-name")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--expected-plan-sha256")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.execute and args.mode != "apply":
        parser.error("--execute applies only to apply")
    if args.mode == "inventory":
        if not args.root or not args.bundle:
            parser.error("inventory requires --root and --bundle")
        result = inventory(args.root, read_json(args.bundle))
    elif args.mode == "plan":
        if not all((args.root, args.bundle, args.payload, args.inventory, args.archive_name)):
            parser.error("plan requires --root --bundle --payload --inventory --archive-name")
        result = build_plan(args.root, args.payload, read_json(args.bundle), read_json(args.inventory), args.archive_name)
    else:
        if not args.plan:
            parser.error("apply requires --plan")
        if args.execute and (not args.expected_plan_sha256 or digest(args.plan) != args.expected_plan_sha256):
            parser.error("Execution requires the exact reviewed --expected-plan-sha256")
        result = apply_plan(read_json(args.plan), args.execute)
    rendered = json.dumps(result, ensure_ascii=True, indent=2) + "\n"
    if args.output:
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(rendered)
    else:
        print(rendered, end="")
    return 2 if result.get("status") == "blocked" else 0


if __name__ == "__main__":
    raise SystemExit(main())
