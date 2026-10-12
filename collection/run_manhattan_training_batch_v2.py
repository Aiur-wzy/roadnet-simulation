"""Supervise quality-gated native v2 collection of a Manhattan dataset.

Use a fresh --output-root; --resume follows the same frozen batch and never
retries a launched scenario. --pilot takes the first 32 vehicles, unchanged,
into separate inputs/results and does not automatically start full collection.
Create OUTPUT_ROOT/STOP to stop dispatching: running workers keep collecting.
Workers survive supervisor exit; nohup may be used for the supervisor itself.
No simulation hard end is introduced. A censored or failed job stops dispatch.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET


DATASET_SCHEMA = "manhattan-structured-demand-v1"
BATCH_SCHEMA = "manhattan-native-collection-batch-v2"
COLLECTOR = "collection.native_boundary_v2.collect_native_v2"
COLLECTOR_FILE = "collection/native_boundary_v2/collect_native_v2.py"
TERMINAL = {"COMPLETE", "CENSORED", "FAILED", "FAILED_PILOT_GATE", "INTERRUPTED"}
PILOT_COUNT = 32
# These are the output-only defaults written by the frozen demand generator.
# The collector owns all fresh output destinations and reasserts unfinished
# tripinfo output on its CLI. No other configuration option is rewritten.
GENERATED_OUTPUT_OPTIONS = {
    "tripinfo-output": "tripinfo.xml",
    "tripinfo-output.write-unfinished": "true",
    "statistic-output": "statistics.xml",
    "summary-output": "summary.xml",
}


def collector_sources(package_root):
    root = Path(package_root)
    required = {"collection/__init__.py", COLLECTOR_FILE,
                "collection/run_manhattan_training_batch_v2.py",
                "collection/native_boundary_v2/__init__.py", "collection/TraCI_Python_SUMO_Native.py",
                "collection/sumo_native_state.py", "collection/sumo_native_signals.py",
                "collection/traci_signal_rg.py", "features/__init__.py",
                "features/cap_context_features.py", "features/signal_rg_features.py"}
    for relative in required:
        if not (root / relative).is_file():
            raise ValueError("Collector dependency missing: " + relative)
    paths = required | {path.relative_to(root).as_posix() for path in (root / "features").rglob("*.py")}
    paths |= {path.relative_to(root).as_posix() for path in (root / "collection").glob("sumo_native_*.py")}
    paths |= {path.relative_to(root).as_posix() for path in (root / "collection/native_boundary_v2").rglob("*.py")}
    return {relative: sha(root / relative) for relative in sorted(paths)}


def now():
    return datetime.now(timezone.utc).isoformat()


def sha(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=True, separators=(",", ":")).encode()


def open_shared_read(path):
    """Windows readers permit atomic replacement: READ | WRITE | DELETE sharing."""
    if os.name != "nt":
        return Path(path).open("rb")
    import ctypes
    from ctypes import wintypes
    import msvcrt
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    create = kernel.CreateFileW
    create.argtypes = (wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
                       wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE)
    create.restype = wintypes.HANDLE
    handle = create(str(Path(path).resolve()), 0x80000000, 0x1 | 0x2 | 0x4, None, 3, 0x80, None)
    if handle == wintypes.HANDLE(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        descriptor = msvcrt.open_osfhandle(handle, os.O_RDONLY | os.O_BINARY)
    except Exception:
        kernel.CloseHandle.argtypes = (wintypes.HANDLE,)
        kernel.CloseHandle(handle)
        raise
    try:
        return os.fdopen(descriptor, "rb")
    except Exception:
        os.close(descriptor)
        raise


def retry_windows_io(operation, timeout=2.0):
    """Retry only transient Windows permission/share errors, within a deadline."""
    deadline, delay = time.monotonic() + timeout, .01
    while True:
        try:
            return operation()
        except PermissionError as error:
            if getattr(error, "winerror", None) not in {5, 32, 33}:
                raise
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise
            time.sleep(min(delay, remaining))
            if time.monotonic() >= deadline:
                raise
            delay = min(.2, delay * 2)


def read_json_once(path):
    with open_shared_read(path) as stream:
        return json.loads(stream.read().decode("utf-8-sig"))


def read_json(path):
    return retry_windows_io(lambda: read_json_once(path))


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=True, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    # On permanent failure retain both the previous JSON and this new temporary
    # file as evidence. Never fall back to truncating an active status file.
    retry_windows_io(lambda: os.replace(temporary, path))


def fresh_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=True, indent=2, sort_keys=True)
        stream.write("\n")


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch("[A-Za-z0-9][A-Za-z0-9_-]{0,127}", value):
        raise ValueError("Unsafe scenario identifier")
    return value


def input_path(root, relative):
    if not isinstance(relative, str) or "\\" in relative or ":" in relative:
        raise ValueError("Dataset paths must be POSIX relative paths")
    parts = PurePosixPath(relative).parts
    if (not parts or relative != "/".join(parts) or parts[0] == "/"
            or any(part in {"", ".", ".."} for part in parts)):
        raise ValueError("Dataset path escapes its root")
    path = Path(root)
    for part in parts:
        path = path / part
        if path.is_symlink():
            raise ValueError("Dataset symlink refused: " + str(path))
    path = path.resolve(strict=True)
    path.relative_to(Path(root).resolve(strict=True))
    if not path.is_file():
        raise ValueError("Dataset input is not a file: " + str(path))
    return path


def verify(path, expected, cache=None):
    if not isinstance(expected, str) or not re.fullmatch("[0-9a-f]{64}", expected):
        raise ValueError("Missing/invalid input SHA256: " + str(path))
    key = (str(path), expected)
    if cache is None or key not in cache:
        if sha(path) != expected:
            raise ValueError("Input SHA256 mismatch: " + str(path))
        if cache is not None:
            cache.add(key)


def load_dataset(manifest_path, selected=None, network_override=None):
    manifest_path = Path(manifest_path).resolve(strict=True)
    root, digest = manifest_path.parent, sha(manifest_path)
    manifest = read_json(manifest_path)
    if (manifest.get("schema") != DATASET_SCHEMA or manifest.get("ready") is not True
            or manifest.get("status") != "DEMAND_GENERATED_NOT_SIMULATED"):
        raise ValueError("Dataset is not a ready, generated structured-demand dataset")
    rows = manifest.get("scenarios")
    if not isinstance(rows, list) or not rows:
        raise ValueError("Dataset has no scenario records")
    index = {}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("Old string-only scenario index is unsupported")
        sid = identifier(row["scenario_id"])
        if sid in index:
            raise ValueError("Duplicate scenario ID: " + sid)
        index[sid] = row
    requested = list(selected) if selected is not None else list(index)
    if len(set(requested)) != len(requested) or set(requested) - set(index):
        raise ValueError("Unknown or duplicate requested scenarios")
    if not requested:
        raise ValueError("Empty scenario selection")
    cache, result = set(), []
    for sid in requested:
        row = index[sid]
        paths = {name: input_path(root, row[field]) for name, field in (
            ("route", "route_file"), ("config", "sumo_config"), ("manifest", "scenario_manifest"))}
        hashes = {"route": row["route_sha256"], "config": row["config_sha256"],
                  "manifest": row["scenario_manifest_sha256"]}
        for name, path in paths.items():
            verify(path, hashes[name], cache)
        network = Path(network_override or row["network_file"])
        if not network.is_absolute():
            network = root / network
        network = network.resolve(strict=True)
        verify(network, row["network_sha256"], cache)
        if row["network_sha256"] != manifest["inputs"]["network"]["sha256"]:
            raise ValueError("Scenario network differs from dataset network")
        detail = read_json(paths["manifest"])
        if detail.get("schema") != DATASET_SCHEMA or detail.get("status") != "DEMAND_GENERATED_NOT_SIMULATED":
            raise ValueError("Scenario manifest is not generated")
        for key in ("scenario_id", "split", "family_id", "vehicle_count"):
            if detail["scenario"][key] != row[key]:
                raise ValueError("Scenario identity/count mismatch: " + sid)
        count = row["vehicle_count"]
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("Invalid vehicle count: " + sid)
        if detail["inputs"]["network"]["sha256"] != row["network_sha256"]:
            raise ValueError("Scenario manifest network mismatch")
        for name, basename in (("route", "routes.rou.xml"), ("config", "run.sumocfg")):
            if detail["outputs"][basename]["sha256"] != hashes[name]:
                raise ValueError("Scenario output hash contract mismatch")
        check_config(paths["config"])
        result.append({"scenario_id": sid, "split": row["split"], "family_id": row["family_id"],
                       "vehicle_count": count, "paths": {key: str(value) for key, value in paths.items()},
                       "hashes": hashes, "network": str(network), "network_sha256": row["network_sha256"]})
    if sha(manifest_path) != digest:
        raise ValueError("Dataset manifest changed during validation")
    return digest, result


def check_config(path):
    tree = ET.parse(path)
    root = tree.getroot()
    if root.tag != "configuration" or root.find("time/end") is not None:
        raise ValueError("Expected a SUMO config without a hard end")
    inputs = root.find("input")
    if inputs is None or sorted(child.tag for child in inputs) != ["net-file", "route-files"]:
        raise ValueError("Only the frozen network and route inputs are supported")
    outputs = root.findall("output")
    if len(outputs) > 1:
        raise ValueError("Duplicate generated output section")
    removable = set()
    for section in outputs:
        if section.attrib:
            raise ValueError("Unexpected generated output section attributes")
        seen = set()
        for option in section:
            if (option.tag not in GENERATED_OUTPUT_OPTIONS or option.tag in seen
                    or list(option) or option.attrib != {"value": GENERATED_OUTPUT_OPTIONS[option.tag]}):
                raise ValueError("Unknown or changed generated output option: " + option.tag)
            seen.add(option.tag)
            removable.add(option)
    for option in root.iter():
        if option in removable or option in outputs:
            continue
        if ("output" in option.tag or option.tag in {"log", "error-log"}
                or option.tag.startswith("save-state.")):
            raise ValueError("Unmanaged output/log destination in config: " + option.tag)
    return tree


def write_pilot_route(source, destination, expected_count):
    """Retain first 32 XML vehicles exactly; declarations and root attrs survive."""
    kept, total, output, stack = [], 0, None, []
    with Path(source).open("rb") as stream:
        for event, element in ET.iterparse(stream, events=("start", "end")):
            if event == "start":
                if not stack:
                    if element.tag != "routes":
                        raise ValueError("Pilot source root must be routes")
                    output = ET.Element("routes", dict(element.attrib))
                stack.append(element)
                continue
            if len(stack) == 2:
                if element.tag == "vehicle":
                    total += 1
                    if len(kept) < PILOT_COUNT:
                        factor = float(element.attrib.get("speedFactor", "nan"))
                        if not math.isfinite(factor) or factor <= 0:
                            raise ValueError("Pilot requires explicit finite speedFactor")
                        if element.find("route") is None or not element.find("route").get("edges"):
                            raise ValueError("Pilot requires complete inline routes")
                        kept.append(element.attrib["id"])
                        output.append(copy.deepcopy(element))
                elif element.tag in {"vType", "route"} and total == 0:
                    output.append(copy.deepcopy(element))
                else:
                    raise ValueError("Unsupported demand element in pilot: " + element.tag)
                stack[0].remove(element)
                element.clear()
            stack.pop()
    if total != expected_count or len(set(kept)) != len(kept) or not kept:
        raise ValueError("Pilot source vehicle count/IDs disagree with manifest")
    with Path(destination).open("xb") as stream:
        ET.ElementTree(output).write(stream, encoding="utf-8", xml_declaration=True)
    return kept


class FileLock:
    """Kernel-released lock: stale lock files never authorize duplicate work."""
    def __init__(self, path):
        self.path, self.stream = Path(path), None

    def __enter__(self):
        self.stream = self.path.open("a+b")
        if self.stream.tell() == 0:
            self.stream.write(b"0")
            self.stream.flush()
        self.stream.seek(0)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.stream.close()
            self.stream = None
            raise RuntimeError("Another process holds " + str(self.path))
        return self

    def __exit__(self, *_):
        if self.stream is not None:
            if os.name == "nt":
                import msvcrt
                self.stream.seek(0)
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
            self.stream.close()


def process_identity(pid):
    if sys.platform == "linux":
        try:
            text = (Path("/proc") / str(pid) / "stat").read_text()
        except FileNotFoundError:
            return None
        fields = text[text.rfind(")") + 2:].split()
        if fields[0] == "Z":
            return None
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip() + ":" + fields[19]
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes
        api = ctypes.WinDLL("kernel32", use_last_error=True)
        api.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
        api.OpenProcess.restype = wintypes.HANDLE
        api.GetProcessTimes.argtypes = (wintypes.HANDLE,) + (ctypes.POINTER(wintypes.FILETIME),) * 4
        api.GetExitCodeProcess.argtypes = (wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD))
        api.CloseHandle.argtypes = (wintypes.HANDLE,)
        handle = api.OpenProcess(0x1000, False, int(pid))
        if not handle:
            if ctypes.get_last_error() == 87:
                return None
            raise PermissionError("Cannot inspect process identity")
        try:
            code = wintypes.DWORD()
            if not api.GetExitCodeProcess(handle, ctypes.byref(code)):
                raise OSError("GetExitCodeProcess failed")
            if code.value != 259:
                return None
            values = [wintypes.FILETIME() for _ in range(4)]
            if not api.GetProcessTimes(handle, *(ctypes.byref(value) for value in values)):
                raise OSError("GetProcessTimes failed")
            return str((values[0].dwHighDateTime << 32) | values[0].dwLowDateTime)
        finally:
            api.CloseHandle(handle)
    raise RuntimeError("Process identity is supported only on Linux and Windows")


def alive(pid, identity):
    return bool(pid and identity and process_identity(pid) == identity)


def popen_options():
    options = {"stdin": subprocess.DEVNULL, "close_fds": True}
    if os.name == "nt":
        options["creationflags"] = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        options["start_new_session"] = True
    return options


def collector_command(spec):
    return [spec["python"], "-u", "-B", "-m", COLLECTOR,
            "--sumo-config", spec["config"], "--net-file", spec["network"],
            "--output-dir", spec["result"], "--sumo-binary", spec["sumo_binary"],
            "--stall-seconds", str(spec["stall_seconds"])]


def prepare_job(output_root, scenario, contract, package_root):
    job = Path(output_root) / "jobs" / scenario["scenario_id"]
    job.mkdir(exist_ok=False)
    inputs = job / "inputs"
    inputs.mkdir()
    for key, path in scenario["paths"].items():
        verify(path, scenario["hashes"][key])
    verify(scenario["network"], scenario["network_sha256"])
    route = Path(scenario["paths"]["route"])
    pilot_ids = None
    if contract["pilot"]:
        route = inputs / "routes.rou.xml"
        pilot_ids = write_pilot_route(scenario["paths"]["route"], route, scenario["vehicle_count"])
    config = inputs / "run.sumocfg"
    tree = check_config(scenario["paths"]["config"])
    # Only the new per-job XML is changed. Keep the frozen source config and
    # all SUMO behavior/seed/time options intact; unknown outputs failed above.
    for section in tree.getroot().findall("output"):
        tree.getroot().remove(section)
    tree.find("input/net-file").set("value", scenario["network"])
    tree.find("input/route-files").set("value", str(route))
    with config.open("xb") as stream:
        tree.write(stream, encoding="utf-8", xml_declaration=True)
    for key, path in scenario["paths"].items():
        verify(path, scenario["hashes"][key])
    spec = {"schema": BATCH_SCHEMA, "scenario_id": scenario["scenario_id"],
            "contract_sha256": hashlib.sha256(canonical(contract)).hexdigest(),
            "package_root": str(package_root), "python": contract["python"],
            "sumo_binary": contract["sumo_binary"], "stall_seconds": contract["stall_seconds"],
            "pilot": contract["pilot"], "pilot_vehicle_ids": pilot_ids,
            "expected_vehicles": len(pilot_ids) if pilot_ids is not None else scenario["vehicle_count"],
            "config": str(config), "route": str(route), "network": scenario["network"],
            "result": str(job / "result"), "source_scenario": scenario,
            "collector_sha256": contract["collector_sha256"],
            "collector_sources_sha256": contract["collector_sources_sha256"],
            "runner_sha256": contract["runner_sha256"],
            "prepared_sha256": {str(config): sha(config), str(route): sha(route),
                                scenario["network"]: scenario["network_sha256"]}}
    fresh_json(job / "job.json", spec)
    atomic_json(job / "status.json", {"state": "PREPARED", "scenario_id": scenario["scenario_id"],
                                      "contract_sha256": spec["contract_sha256"],
                                      "job_spec_sha256": sha(job / "job.json"), "prepared_utc": now()})
    return job


def assess_audit(audit, returncode, expected, pilot):
    """Gate whole-run trust separately from excluded individual road visits."""
    problems, counts = [], audit.get("counts", {})
    if not isinstance(counts, dict):
        counts = {}
    if returncode != 0:
        problems.append("collector exit code is not zero")
    if audit.get("status") != "COMPLETE":
        problems.append("collector audit status is not COMPLETE")
    if "error" not in audit or audit["error"] is not None:
        problems.append("collector error is not explicitly null")
    if audit.get("training_released") is not True:
        problems.append("quality-gated training output is not released")
    for key in ("expected_vehicles", "arrived", "tripinfo_count", "normal_arrived"):
        if type(counts.get(key)) is not int or counts[key] != expected:
            problems.append(key + " does not equal the selected vehicle count")
    for key in ("teleports", "collisions"):
        if type(counts.get(key)) is not int or counts[key] != 0:
            problems.append(key + " is nonzero or absent")
    if audit.get("explicit_speed_factor_audited") is not True:
        problems.append("explicit SF audit is not confirmed")
    if audit.get("native_outputs_closed") is not True:
        problems.append("native outputs are not confirmed closed")
    quality_counts = ("road_occurrences", "trainable", "excluded")
    if any(type(counts.get(key)) is not int or counts[key] < 0 for key in quality_counts):
        problems.append("missing/invalid per-road quality counts")
    elif counts["trainable"] + counts["excluded"] != counts["road_occurrences"]:
        problems.append("per-road training/exclusion counts do not close")
    elif counts["road_occurrences"] < expected:
        problems.append("road occurrence count is below the vehicle count")
    if pilot and (type(counts.get("trainable")) is not int or counts["trainable"] <= 0):
        problems.append("pilot has no quality-admitted training rows")
    if audit.get("status") == "CENSORED":
        return "CENSORED", problems
    return ("FAILED_PILOT_GATE" if pilot else "FAILED") if problems else "COMPLETE", problems


def verify_result_bindings(spec, audit):
    """Recheck closed evidence against the exact frozen job, never live outputs.

    Hashes establish that the collector's completed-file/quality audit concerns
    these bytes. The collector is responsible for reading its streams to EOF
    before releasing them; the batch does not reinterpret scientific labels.
    """
    root = Path(spec["result"]).resolve(strict=True)
    manifest_path = input_path(root, "manifest.json")
    verify(manifest_path, audit.get("manifest_sha256"))
    manifest = read_json(manifest_path)
    inputs = manifest.get("inputs")
    if not isinstance(inputs, dict) or inputs != spec["prepared_sha256"]:
        raise ValueError("Collector manifest inputs differ from the prepared job")
    for path, expected in inputs.items():
        if Path(path).resolve(strict=True) != Path(path):
            raise ValueError("Collector input path is not canonical")
        verify(path, expected)
    sources = manifest.get("sources")
    if not isinstance(sources, dict) or sources != spec["collector_sources_sha256"]:
        raise ValueError("Collector manifest sources differ from the frozen bundle")
    for relative, expected in sources.items():
        verify(input_path(spec["package_root"], relative), expected)
    outputs = audit.get("closed_outputs_sha256")
    required = {"training.csv.gz", "road_visits.csv.gz", "tripinfo.xml", "vehroute.xml.gz"}
    if not isinstance(outputs, dict) or not required.issubset(outputs):
        raise ValueError("Collector audit lacks closed training/native output bindings")
    for relative, expected in outputs.items():
        if relative in {"audit.json", "manifest.json"} or ".partial" in PurePosixPath(relative).parts[-1]:
            raise ValueError("Invalid closed output binding")
        verify(input_path(root, relative), expected)
    return {"manifest_sha256": audit["manifest_sha256"], "closed_outputs_sha256": outputs}


def run_worker(spec_path):
    spec_path = Path(spec_path).resolve(strict=True)
    job, spec = spec_path.parent, read_json(spec_path)
    if spec.get("schema") != BATCH_SCHEMA or Path(spec["result"]) != job / "result":
        raise ValueError("Invalid worker specification")
    with FileLock(job / "worker.lock"):
        status = read_json(job / "status.json")
        verify(spec_path, status.get("job_spec_sha256"))
        if status.get("state") != "LAUNCHING":
            raise ValueError("Worker is not a fresh launch; refusing duplicate collection")
        status.update(state="RUNNING", worker_pid=os.getpid(), worker_identity=process_identity(os.getpid()), started_utc=now())
        atomic_json(job / "status.json", status)
        process = None
        try:
            if Path(spec["result"]).exists():
                raise ValueError("Collector result directory already exists")
            for path, expected in spec["prepared_sha256"].items():
                verify(path, expected)
            collector_file = Path(spec["package_root"]) / COLLECTOR_FILE
            verify(collector_file, spec["collector_sha256"])
            if collector_sources(spec["package_root"]) != spec["collector_sources_sha256"]:
                raise ValueError("Collector dependency bundle changed before worker launch")
            verify(__file__, spec["runner_sha256"])
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            env["PYTHONPATH"] = spec["package_root"] + os.pathsep + env.get("PYTHONPATH", "")
            command = collector_command(spec)
            with (job / "stdout.log").open("xb") as stdout, (job / "stderr.log").open("xb") as stderr:
                process = subprocess.Popen(command, cwd=spec["package_root"], env=env, stdout=stdout, stderr=stderr, **popen_options())
                status.update(collector_pid=process.pid, collector_identity=process_identity(process.pid), command=command)
                atomic_json(job / "status.json", status)
                returncode = process.wait()
            audit_path = Path(spec["result"]) / "audit.json"
            audit = read_json(audit_path) if audit_path.is_file() else {}
            state, problems = assess_audit(audit, returncode, spec["expected_vehicles"], spec["pilot"])
            if state == "COMPLETE":
                status["verified_bindings"] = verify_result_bindings(spec, audit)
            status.update(state=state, returncode=returncode, audit_status=audit.get("status"),
                          audit_sha256=sha(audit_path) if audit_path.is_file() else None,
                          gate_problems=problems, counts=audit.get("counts", {}), finished_utc=now())
        except Exception as error:
            # Preserve and await an already launched collector on local bookkeeping
            # errors. Never claim this worker has finished while its child runs.
            if process is not None and process.poll() is None:
                process.wait()
            status.update(state="FAILED", error=type(error).__name__ + ": " + str(error), finished_utc=now())
        atomic_json(job / "status.json", status)
    return 0 if status["state"] == "COMPLETE" else 2


def worker_active(job, status, child=None):
    if child is not None and child.poll() is None:
        return True
    if alive(status.get("worker_pid"), status.get("worker_identity")):
        return True
    if (job / "worker.lock").exists():
        try:
            with FileLock(job / "worker.lock"):
                pass
        except RuntimeError:
            return True
    if alive(status.get("collector_pid"), status.get("collector_identity")):
        raise RuntimeError("Collector survives its worker; no restart is permitted: " + str(job))
    return False


def supervise(output, scenarios, contract, package_root, max_parallel, poll_seconds):
    children, digest = {}, hashlib.sha256(canonical(contract)).hexdigest()
    # Closed evidence is rehashed once per supervisor session, including resume.
    # Do not reread multi-GB completed CSVs on each two-second scheduling poll.
    checked_results = set()
    def emit(event, **fields):
        row = {"time": now(), "event": event, **fields}
        text = json.dumps(row, ensure_ascii=True)
        print(text, flush=True)
        with (output / "batch.log").open("a", encoding="utf-8") as stream:
            stream.write(text + "\n")
    while True:
        pending, active, states = [], [], {}
        for scenario in scenarios:
            sid, job = scenario["scenario_id"], output / "jobs" / scenario["scenario_id"]
            if not job.exists():
                pending.append(scenario)
                states[sid] = "PENDING"
                continue
            if not (job / "status.json").is_file() or not (job / "job.json").is_file():
                raise RuntimeError("Incomplete job preparation retained; choose a new output root: " + sid)
            status = read_json(job / "status.json")
            verify(job / "job.json", status.get("job_spec_sha256"))
            spec = read_json(job / "job.json")
            if status.get("contract_sha256") != digest or spec.get("contract_sha256") != digest:
                raise ValueError("Existing scenario belongs to a different batch: " + sid)
            state = status.get("state")
            if state == "PREPARED":
                pending.append(scenario)
            elif state in {"LAUNCHING", "RUNNING"}:
                if worker_active(job, status, children.get(sid)):
                    active.append(sid)
                else:
                    # Worker may have just published its terminal status.
                    status = read_json(job / "status.json")
                    state = status.get("state")
                    if state not in TERMINAL:
                        state = "INTERRUPTED"
                        status.update(state=state, finished_utc=now(),
                                      error="Launched worker is gone; no implicit retry or result adoption")
                        atomic_json(job / "status.json", status)
                        emit("interrupted", scenario_id=sid)
            elif state not in TERMINAL:
                raise ValueError("Unknown job state: " + str(state))
            if state == "COMPLETE":
                audit_path = job / "result/audit.json"
                verify(audit_path, status.get("audit_sha256"))
                verified_state, _ = assess_audit(read_json(audit_path), status.get("returncode"),
                                                spec["expected_vehicles"], spec["pilot"])
                if verified_state != "COMPLETE":
                    raise ValueError("Completed scenario no longer passes its audit: " + sid)
                if sid not in checked_results:
                    bindings = verify_result_bindings(spec, read_json(audit_path))
                    if status.get("verified_bindings") != bindings:
                        raise ValueError("Completed worker evidence binding mismatch: " + sid)
                    checked_results.add(sid)
            if state in TERMINAL:
                child = children.get(sid)
                if child is not None:
                    if child.poll() is None:
                        active.append(sid)
                elif alive(status.get("worker_pid"), status.get("worker_identity")):
                    active.append(sid)
            states[sid] = state
        failures = [sid for sid, state in states.items() if state in TERMINAL - {"COMPLETE"}]
        stopped = (output / "STOP").exists()
        while pending and len(active) < max_parallel and not stopped and not failures:
            scenario = pending.pop(0)
            sid, job = scenario["scenario_id"], output / "jobs" / scenario["scenario_id"]
            if not job.exists():
                job = prepare_job(output, scenario, contract, package_root)
            if (output / "STOP").exists():
                stopped = True
                pending.insert(0, scenario)
                break
            status = read_json(job / "status.json")
            if status["state"] != "PREPARED":
                raise ValueError("Scenario is not eligible for its first launch")
            status.update(state="LAUNCHING", launch_requested_utc=now())
            atomic_json(job / "status.json", status)
            with (job / "worker.stdout.log").open("xb") as stdout, (job / "worker.stderr.log").open("xb") as stderr:
                child = subprocess.Popen([sys.executable, "-u", "-B", str(Path(__file__).resolve()),
                                          "--worker-spec", str(job / "job.json")], cwd=package_root,
                                         stdout=stdout, stderr=stderr, **popen_options())
            children[sid] = child
            active.append(sid)
            states[sid] = "LAUNCHING"
            emit("launched", scenario_id=sid, worker_pid=child.pid, pilot=contract["pilot"])
            stopped = (output / "STOP").exists()
        state = "RUNNING" if active else "FAILED" if failures else "STOPPED" if pending else "COMPLETE"
        summary = {"schema": BATCH_SCHEMA, "status": state, "updated_utc": now(), "pilot": contract["pilot"],
                   "scenarios": states, "active": active, "failures": failures, "stop_present": stopped,
                   "max_parallel": max_parallel, "pending": [row["scenario_id"] for row in pending]}
        atomic_json(output / "status.json", summary)
        if not active:
            emit("batch_finished", status=state, failures=failures, pending=summary["pending"])
            return 0 if state == "COMPLETE" else 2
        time.sleep(poll_seconds)


def run_batch(args, package_root=None):
    package_root = Path(package_root or Path(__file__).resolve().parents[1]).resolve(strict=True)
    output = Path(args.output_root).resolve()
    old = None
    if output.exists():
        if not args.resume:
            raise ValueError("Output root exists; use --resume for the same batch or a fresh path")
        old = read_json(output / "batch_manifest.json")["contract"]
    elif args.resume:
        raise ValueError("Cannot resume a nonexistent output root")
    selected = args.scenario if args.scenario is not None else old["scenario_ids"] if old else None
    manifest_sha, scenarios = load_dataset(args.dataset_manifest, selected, args.net_file)
    pilot = bool(args.pilot) if args.pilot is not None else old["pilot"] if old else False
    collector = package_root / COLLECTOR_FILE
    contract = {"dataset_manifest": str(Path(args.dataset_manifest).resolve()), "dataset_sha256": manifest_sha,
                "scenario_ids": [row["scenario_id"] for row in scenarios], "scenarios": scenarios,
                "pilot": pilot, "pilot_count": PILOT_COUNT, "python": sys.executable,
                "collector_sha256": sha(collector), "package_root": str(package_root),
                "collector_sources_sha256": collector_sources(package_root), "runner_sha256": sha(__file__),
                "sumo_binary": args.sumo_binary or (old["sumo_binary"] if old else "sumo"),
                "stall_seconds": args.stall_seconds if args.stall_seconds is not None else old["stall_seconds"] if old else 3600}
    if old is not None and contract != old:
        raise ValueError("Resume contract changed: dataset, inputs, selection, mode or collector differs")
    if args.dry_run:
        result = {"schema": BATCH_SCHEMA, "status": "DRY_RUN", "output_root": str(output),
                  "max_parallel": args.max_parallel, "contract": contract, "simulation_started": False,
                  "collector_module": COLLECTOR,
                  "prepared_config_paths": {row["scenario_id"]: str(output / "jobs" / row["scenario_id"] / "inputs/run.sumocfg")
                                            for row in scenarios}}
        print(json.dumps(result, ensure_ascii=True, indent=2), flush=True)
        return 0
    if old is None:
        output.mkdir(parents=True, exist_ok=False)
        (output / "jobs").mkdir()
    with FileLock(output / "supervisor.lock"):
        if old is None:
            fresh_json(output / "batch_manifest.json", {"schema": BATCH_SCHEMA, "created_utc": now(), "contract": contract})
        else:
            if read_json(output / "batch_manifest.json")["contract"] != contract:
                raise ValueError("Batch manifest changed while acquiring supervisor lock")
        return supervise(output, scenarios, contract, package_root, args.max_parallel, args.poll_seconds)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--dataset-manifest", type=Path)
    result.add_argument("--output-root", type=Path)
    result.add_argument("--scenario", action="append")
    result.add_argument("--max-parallel", type=int, default=2)
    result.add_argument("--pilot", action="store_true", default=None)
    result.add_argument("--dry-run", action="store_true")
    result.add_argument("--resume", action="store_true")
    result.add_argument("--net-file", type=Path, help="Relocated network path; must match the frozen SHA256")
    result.add_argument("--sumo-binary")
    result.add_argument("--stall-seconds", type=int)
    result.add_argument("--poll-seconds", type=float, default=2.)
    result.add_argument("--worker-spec", type=Path, help=argparse.SUPPRESS)
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if args.worker_spec:
        return run_worker(args.worker_spec)
    if not args.dataset_manifest or not args.output_root:
        cli.error("--dataset-manifest and --output-root are required")
    if args.max_parallel < 1 or not math.isfinite(args.poll_seconds) or args.poll_seconds < .05:
        cli.error("max-parallel must be positive and poll-seconds must be at least 0.05")
    if args.stall_seconds is not None and args.stall_seconds < 1:
        cli.error("stall-seconds must be positive")
    try:
        return run_batch(args)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print("BATCH_ERROR: " + str(error), file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
