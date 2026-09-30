import subprocess
from pathlib import Path

from scripts.check_secrets import blobs, findings


def test_common_secrets_detected_without_echoing_values():
    key = '-----BEGIN ' + 'PRIVATE KEY-----'
    secret = 'x' * 32
    for payload in [key, f'PI_API_KEY={secret}', f'{{"api_key":"{secret}"}}']:
        reasons = findings('accidental.txt', payload.encode())
        assert reasons
        assert secret not in str(reasons)
        assert key not in str(reasons)
    assert findings('.env', b'') == ['private file path']
    assert findings('config/pi.local.json', b'{}') == ['private file path']
    assert findings('innocent.json', key.encode()) == ['private key']
    assert not findings('.env.example', b'PI_API_KEY=\nGOOGLE_APPLICATION_CREDENTIALS=\n')
    assert not findings('config/pi.json', b'{"token_env": "ROBOT_TOKEN"}')
    assert not findings('README.md', b"export ROBOT_TOKEN='replace-with-the-same-random-secret-on-both-computers'")


def test_index_and_history_catch_secret_after_working_copy_is_cleaned(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    def git(*args):
        subprocess.run(['git', *args], check=True, capture_output=True)

    git('init')
    git('config', 'user.email', 'test@example.invalid')
    git('config', 'user.name', 'Test')
    p = Path('notes.txt')
    secret = 'PI_API_KEY=' + 'x' * 32
    p.write_text(secret)
    git('add', 'notes.txt')
    p.write_text('clean working copy')
    assert findings(*next(blobs(staged=True))) == ['dotenv PI key']
    assert not findings(*next(blobs()))
    git('commit', '-m', 'synthetic scanner fixture')
    git('add', 'notes.txt')
    git('commit', '-m', 'remove synthetic fixture')
    assert any(findings(name, content, check_path=False) for name, content in blobs(history=True))
