#!/usr/bin/env python3
import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from verify_task_environment import validate
import verify_task_environment as environment

class EnvironmentContract(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.file=self.root/'message.idl';self.file.write_text('uint64 revision;')
        self.manifest={'prefixes':{'messages':str(self.root)},'files':{str(self.file):{'sha256':hashlib.sha256(self.file.read_bytes()).hexdigest(),'resolved':str(self.file.resolve())}}}
    def check(self,lookup=None):return validate(self.manifest,lookup or (lambda _:str(self.root)))
    def test_identical_snapshot_passes(self):self.assertEqual(self.check(),[])
    def test_wrong_overlay_rejected(self):self.assertTrue(any('OVERLAY_MISMATCH' in e for e in self.check(lambda _:str(self.root/'old'))))
    def test_missing_package_rejected(self):
        def missing(_):raise LookupError('absent')
        self.assertTrue(any('PACKAGE_UNAVAILABLE' in e for e in self.check(missing)))
    def test_changed_interface_rejected(self):
        self.file.write_text('uint32 revision;');self.assertTrue(any('ARTIFACT_CHANGED' in e for e in self.check()))
    def test_missing_artifact_rejected(self):
        self.file.unlink();self.assertTrue(any('ARTIFACT_UNAVAILABLE' in e for e in self.check()))
    def test_retargeted_symlink_rejected(self):
        other=self.root/'other';other.write_text('uint64 revision;');self.file.unlink();self.file.symlink_to(other)
        self.assertTrue(any('ARTIFACT_TARGET_CHANGED' in e for e in self.check()))

    def receipt(self, manifest=None, lookup=None):
        self.assertTrue(callable(getattr(environment, 'verify_manifest_file', None)),
                        'requested runtime snapshot needs a checked, hash-bound receipt')
        path=self.root/'manifest.json'
        path.write_text(json.dumps(self.manifest if manifest is None else manifest))
        return environment.verify_manifest_file(path, lookup or (lambda _:str(self.root)))

    def test_receipt_binds_exact_manifest_and_passed_snapshot(self):
        receipt=self.receipt()
        self.assertTrue(receipt['passed'])
        self.assertEqual(receipt['sha256'], hashlib.sha256((self.root/'manifest.json').read_bytes()).hexdigest())
        self.assertEqual(receipt['artifact_count'], 1)

    def test_receipt_preserves_wrong_overlay_failure(self):
        receipt=self.receipt(lookup=lambda _:str(self.root/'old'))
        self.assertFalse(receipt['passed'])
        self.assertTrue(any('OVERLAY_MISMATCH' in e for e in receipt['errors']))

    def test_receipt_rejects_empty_or_incomplete_snapshot(self):
        for manifest in ({}, {'prefixes':{}, 'files':{}},
                         {'prefixes':self.manifest['prefixes'], 'files':{}}):
            with self.subTest(manifest=manifest):
                self.assertFalse(self.receipt(manifest)['passed'])

    def test_receipt_rejects_dependency_outside_hashed_artifacts(self):
        manifest=dict(self.manifest, dependencies={str(self.root/'unhashed.so'):'fixture'})
        self.assertFalse(self.receipt(manifest)['passed'])

    def test_receipt_rejects_malformed_artifact_record(self):
        manifest=dict(self.manifest, files={str(self.file):{'sha256':'not-a-hash'}})
        self.assertFalse(self.receipt(manifest)['passed'])

    def test_receipt_rejects_changed_artifact(self):
        self.file.write_text('changed library')
        self.assertFalse(self.receipt()['passed'])

    def test_unchanged_missing_elf_dependency_is_rejected(self):
        # Real ELF fixture: ldd exits zero even though the required DSO is gone.
        provider = self.root/'provider.c'
        provider.write_text('int required_fixture(void) { return 1; }')
        consumer = self.root/'consumer.c'
        consumer.write_text('extern int required_fixture(void); int use_fixture(void) { return required_fixture(); }')
        dependency = self.root/'librequired_fixture.so'
        binary = self.root/'libconsumer_fixture.so'
        subprocess.check_call(['cc', '-fPIC', '-shared', str(provider), '-o', str(dependency)])
        subprocess.check_call(['cc', '-fPIC', '-shared', str(consumer), '-L'+str(self.root),
                               '-lrequired_fixture', '-Wl,-rpath,'+str(self.root), '-o', str(binary)])
        dependency.unlink()
        frozen = subprocess.run(['ldd', str(binary)], capture_output=True, text=True, check=True).stdout
        self.assertIn('librequired_fixture.so => not found', frozen)
        manifest = {'prefixes': self.manifest['prefixes'],
                    'files': {str(binary): {'sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
                                            'resolved': str(binary)}},
                    'dependencies': {str(binary): frozen}}
        receipt = self.receipt(manifest)
        self.assertFalse(receipt['passed'], 'an unchanged unresolved dependency must never pass')
        self.assertTrue(any('LIBRARY_DEPENDENCY_MISSING' in e for e in receipt['errors']))

if __name__=='__main__':unittest.main()
