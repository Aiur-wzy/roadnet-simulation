"""Safety and recoverability checks for explicit source-layout deployments."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import uuid
from unittest import mock


TOOL = Path(__file__).resolve().parents[2] / "tools" / "maintenance" / "align_remote_layout.py"
SPEC = importlib.util.spec_from_file_location("layout_alignment_under_test", TOOL)
ALIGN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ALIGN)


class AlignmentTest(unittest.TestCase):
    def setUp(self):
        # Retain tiny fixtures as evidence; ordinary mkdir works in the Windows
        # workspace sandbox where tempfile's restricted-directory ACL may not.
        self.base = TOOL.parents[3] / "test_runs" / uuid.uuid4().hex / self._testMethodName
        self.base.mkdir(parents=True)
        self.root, self.payload = self.base / "remote", self.base / "payload"
        self.root.mkdir()
        self.payload.mkdir()
        self.old = self.write(self.root, "old.py", b"original remote source\n")
        self.new = self.write(self.payload, "collection/new.py", b"new canonical package\n")
        self.model = self.write(self.root, "models/run/model.pkl", b"never deserialize or change\n")
        self.dataset = self.write(self.root, "train_data/observations.csv", b"preserve data\n")
        self.bundle = {"mapping": {"old.py": "collection/new.py"},
                       "files": {"collection/new.py": ALIGN.digest(self.new)},
                       "known_previous": {"old.py": [ALIGN.digest(self.old)]}}
        patch = mock.patch.object(ALIGN, "process_evidence", return_value={"complete": True, "blockers": []})
        patch.start()
        self.addCleanup(patch.stop)

    @staticmethod
    def write(base, relative, data):
        path = base / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return path

    def plan(self):
        observed = ALIGN.inventory(self.root, self.bundle)
        return ALIGN.build_plan(self.root, self.payload, self.bundle, observed, "source_alignment_test")

    def test_default_apply_does_not_modify_tree(self):
        before = sorted(str(path.relative_to(self.root)) for path in self.root.rglob("*"))
        result = ALIGN.apply_plan(self.plan())
        self.assertEqual(result["status"], "dry_run")
        self.assertEqual(before, sorted(str(path.relative_to(self.root)) for path in self.root.rglob("*")))
        self.assertTrue(self.old.exists())

    def test_explicit_apply_preserves_originals_models_and_data(self):
        original = self.old.read_bytes()
        untouched = {path: path.read_bytes() for path in (self.model, self.dataset)}
        result = ALIGN.apply_plan(self.plan(), execute=True)
        self.assertEqual(result["status"], "complete")
        self.assertFalse(self.old.exists())
        archive = Path(result["archive"])
        self.assertEqual((archive / "originals/old.py").read_bytes(), original)
        self.assertEqual((self.root / "collection/new.py").read_bytes(), self.new.read_bytes())
        for path, content in untouched.items():
            self.assertEqual(path.read_bytes(), content)
        self.assertEqual(json.loads((archive / "journal.jsonl").read_text().splitlines()[-1])["action"], "complete")

    def test_unique_remote_legacy_modification_blocks_every_edit(self):
        self.old.write_bytes(b"unique remote improvement\n")
        result = ALIGN.apply_plan(self.plan(), execute=True)
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(self.old.read_bytes(), b"unique remote improvement\n")
        self.assertFalse((self.root / "garbage").exists())

    def test_unknown_canonical_target_is_never_overwritten(self):
        path = self.write(self.root, "collection/new.py", b"remote only\n")
        result = ALIGN.apply_plan(self.plan(), execute=True)
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(path.read_bytes(), b"remote only\n")

    def test_reviewed_canonical_target_is_backed_up(self):
        path = self.write(self.root, "collection/new.py", b"reviewed previous package\n")
        self.bundle["known_previous"]["collection/new.py"] = [ALIGN.digest(path)]
        result = ALIGN.apply_plan(self.plan(), execute=True)
        self.assertEqual((Path(result["archive"]) / "originals/collection/new.py").read_bytes(), b"reviewed previous package\n")
        self.assertEqual(path.read_bytes(), self.new.read_bytes())

    def test_source_drift_after_plan_refuses_mutation(self):
        plan = self.plan()
        self.old.write_bytes(b"changed after review\n")
        with self.assertRaisesRegex(ValueError, "Plan drift"):
            ALIGN.apply_plan(plan, execute=True)
        self.assertEqual(self.old.read_bytes(), b"changed after review\n")

    def test_payload_drift_after_plan_refuses_mutation(self):
        plan = self.plan()
        self.new.write_bytes(b"changed payload\n")
        with self.assertRaisesRegex(ValueError, "Payload hash mismatch"):
            ALIGN.apply_plan(plan, execute=True)
        self.assertTrue(self.old.exists())

    def test_active_or_paused_dependency_blocks(self):
        with mock.patch.object(ALIGN, "process_evidence", return_value={"complete": True, "blockers": [{"pid": 123, "cwd": str(self.root)}]}):
            result = ALIGN.apply_plan(self.plan(), execute=True)
        self.assertEqual(result["status"], "blocked")
        self.assertTrue(self.old.exists())

    def test_incomplete_process_inspection_blocks(self):
        with mock.patch.object(ALIGN, "process_evidence", return_value={"complete": False, "blockers": []}):
            result = ALIGN.apply_plan(self.plan(), execute=True)
        self.assertEqual(result["status"], "blocked")

    def test_new_process_after_backups_keeps_originals(self):
        plan = self.plan()
        with mock.patch.object(ALIGN, "process_evidence", side_effect=[
                {"complete": True, "blockers": []},
                {"complete": True, "blockers": [{"pid": 234}]}]):
            with self.assertRaisesRegex(ValueError, "Process dependency appeared"):
                ALIGN.apply_plan(plan, execute=True)
        self.assertTrue(self.old.exists())
        self.assertEqual((self.root / "garbage/source_alignment_test/originals/old.py").read_bytes(), self.old.read_bytes())

    def test_path_traversal_and_protected_assets_rejected(self):
        for path in ("../old.py", "/tmp/x.py", "collection/../old.py", "C:/x.py",
                     "models/run.py", "train_data/build.py", ".git/hooks/run.py", "build/run.py"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                ALIGN.validate_relative(path)

    def test_nonfile_refused(self):
        (self.root / "collection/new.py").mkdir(parents=True)
        with self.assertRaisesRegex(ValueError, "Not a regular file"):
            self.plan()

    def test_existing_archive_blocks(self):
        (self.root / "garbage/source_alignment_test").mkdir(parents=True)
        self.assertEqual(ALIGN.apply_plan(self.plan(), execute=True)["status"], "blocked")

    def test_symlink_ancestor_is_rejected_before_file_access(self):
        original = Path.is_symlink
        with mock.patch.object(Path, "is_symlink", lambda path: path.name == "collection" or original(path)):
            with self.assertRaisesRegex(ValueError, "Symlink path refused"):
                ALIGN.scoped(self.root, "collection/new.py")

    def test_backup_failure_never_removes_original(self):
        plan = self.plan()
        with mock.patch.object(ALIGN, "exclusive_copy", side_effect=OSError("simulated full disk")):
            with self.assertRaisesRegex(OSError, "full disk"):
                ALIGN.apply_plan(plan, execute=True)
        self.assertTrue(self.old.exists())
        self.assertFalse((self.root / "collection/new.py").exists())

    def test_install_failure_retains_recoverable_original(self):
        plan = self.plan()
        original_copy = ALIGN.exclusive_copy

        def fail_on_install(source, target, expected):
            if target == self.root / "collection/new.py":
                raise OSError("simulated install failure")
            return original_copy(source, target, expected)

        with mock.patch.object(ALIGN, "exclusive_copy", side_effect=fail_on_install):
            with self.assertRaisesRegex(OSError, "install failure"):
                ALIGN.apply_plan(plan, execute=True)
        backup = self.root / "garbage/source_alignment_test/originals/old.py"
        self.assertEqual(backup.read_bytes(), b"original remote source\n")
        journal = self.root / "garbage/source_alignment_test/journal.jsonl"
        self.assertNotIn('"action": "complete"', journal.read_text())

    def test_payload_inside_target_tree_refused(self):
        nested = self.root / "staging"
        nested.mkdir()
        with self.assertRaisesRegex(ValueError, "outside"):
            ALIGN.build_plan(self.root, nested, self.bundle, ALIGN.inventory(self.root, self.bundle), "x")

    def test_apply_cli_requires_exact_reviewed_plan_hash(self):
        plan_path = self.base / "plan.json"
        plan_path.write_text(json.dumps(self.plan()), encoding="utf-8")
        with mock.patch("sys.stderr"), self.assertRaises(SystemExit) as caught:
            ALIGN.main(["apply", "--plan", str(plan_path), "--execute", "--expected-plan-sha256", "0" * 64])
        self.assertEqual(caught.exception.code, 2)
        self.assertTrue(self.old.exists())


class ProcessEvidenceTest(unittest.TestCase):
    def inspect(self, command, comm, *, links=None, failure=None, owner=1026):
        root = str(TOOL.parents[3].resolve())
        process = Path("/proc/54321")
        original_stat = Path.stat
        original_read = Path.read_bytes

        def stat_process(path, *args, **kwargs):
            if path == process:
                if failure == "stat":
                    raise PermissionError("stat denied")
                return SimpleNamespace(st_uid=owner)
            return original_stat(path, *args, **kwargs)

        def read_process(path):
            if path.parent == process:
                if failure == path.name:
                    raise PermissionError(path.name + " denied")
                return {"cmdline": command, "comm": comm.encode() + b"\n"}[path.name]
            return original_read(path)

        def link_process(path):
            value = (links or {}).get(path.name, PermissionError("link denied"))
            if isinstance(value, Exception):
                raise value
            return value

        with mock.patch.object(ALIGN.sys, "platform", "linux"), \
                mock.patch.object(ALIGN.os, "getuid", return_value=1026, create=True), \
                mock.patch.object(ALIGN.os, "getpid", return_value=99999), \
                mock.patch.object(ALIGN, "current_uid_name", return_value="zhiyuan"), \
                mock.patch.object(Path, "iterdir", return_value=iter([process])), \
                mock.patch.object(Path, "stat", autospec=True, side_effect=stat_process), \
                mock.patch.object(Path, "read_bytes", autospec=True, side_effect=read_process), \
                mock.patch.object(ALIGN.os, "readlink", side_effect=link_process):
            return ALIGN.process_evidence(root)

    def test_reviewed_transport_titles_with_only_denied_links_are_recorded(self):
        for command, comm, service in (
                (b"/usr/lib/openssh/sftp-server\0", "sftp-server", "openssh-sftp-server"),
                (b"(sd-pam) \0\0", "(sd-pam)", "systemd-pam"),
                (b"sshd: zhiyuan@pts/15 \0", "sshd", "openssh-session"),
                (b"sshd: zhiyuan@notty \0", "sshd", "openssh-session")):
            with self.subTest(command=command):
                result = self.inspect(command, comm)
                self.assertTrue(result["complete"])
                self.assertEqual(result["blockers"], [])
                excluded = result["excluded_transport_services"]
                self.assertEqual(len(excluded), 1)
                self.assertEqual(excluded[0]["service"], service)
                self.assertEqual(excluded[0]["unreadable_links"], ["cwd", "exe"])
                self.assertNotIn(command.decode(errors="replace"), json.dumps(result))

    def test_one_denied_link_still_checks_the_other_for_project_dependency(self):
        root = str(TOOL.parents[3].resolve())
        for links in ({"cwd": root}, {"exe": root + "/run.py"}):
            with self.subTest(links=links):
                result = self.inspect(b"/usr/lib/openssh/sftp-server\0", "sftp-server", links=links)
                self.assertFalse(result["complete"])
                self.assertEqual(len(result["blockers"]), 1)
                self.assertEqual(result["excluded_transport_services"], [])

    def test_one_denied_link_can_be_excluded_when_readable_link_is_unrelated(self):
        result = self.inspect(b"/usr/lib/openssh/sftp-server\0", "sftp-server",
                              links={"exe": "/usr/lib/openssh/sftp-server"})
        self.assertTrue(result["complete"])
        self.assertEqual(result["excluded_transport_services"][0]["unreadable_links"], ["cwd"])

    def test_unknown_or_extra_transport_arguments_fail_closed(self):
        for command, comm in (
                (b"python\0-m\0collection.run\0", "python"),
                (b"/tmp/sftp-server\0", "sftp-server"),
                (b"/usr/lib/openssh/sftp-server\0--extra\0", "sftp-server"),
                (b"(sd-pam) extra\0", "(sd-pam)"),
                (b"sshd: other@notty\0", "sshd"),
                (b"sshd: zhiyuan@notty extra\0", "sshd"),
                (b"sshd: zhiyuan@pts/x\0", "sshd"),
                (b"sshd: zhiyuan@notty\0", "python")):
            with self.subTest(command=command, comm=comm):
                result = self.inspect(command, comm)
                self.assertFalse(result["complete"])
                self.assertEqual(result["excluded_transport_services"], [])

    def test_project_root_in_transport_command_is_always_a_blocker(self):
        root = str(TOOL.parents[3].resolve())
        result = self.inspect(("sshd: zhiyuan@notty " + root).encode() + b"\0", "sshd")
        self.assertFalse(result["complete"])
        self.assertEqual(len(result["blockers"]), 1)
        self.assertEqual(result["excluded_transport_services"], [])

    def test_stat_cmdline_and_comm_denial_are_not_exempted(self):
        for field in ("stat", "cmdline", "comm"):
            with self.subTest(field=field):
                result = self.inspect(b"/usr/lib/openssh/sftp-server\0", "sftp-server", failure=field)
                self.assertFalse(result["complete"])
                self.assertEqual(result["excluded_transport_services"], [])

    def test_other_link_errors_are_not_exempted(self):
        result = self.inspect(b"/usr/lib/openssh/sftp-server\0", "sftp-server",
                              links={"cwd": OSError("unexpected IO error")})
        self.assertFalse(result["complete"])
        self.assertEqual(result["excluded_transport_services"], [])

    def test_other_uid_is_not_inspected_or_classified(self):
        result = self.inspect(b"/usr/lib/openssh/sftp-server\0", "sftp-server", owner=1027)
        self.assertTrue(result["complete"])
        self.assertEqual(result["excluded_transport_services"], [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
