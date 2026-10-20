from pathlib import Path

config = Path('/workspace/fastApi/public/config/config.js')
text = config.read_text()
old = ": ('http://' + apiBackendHost)"
new = ": (window.location.origin)"
if old in text:
    backup = config.with_name('config.js.before-wslc-8080.bak')
    if not backup.exists():
        backup.write_text(text)
    text = text.replace(old, new, 1)
    text = text.replace("baseURL: 'http://localhost',", "baseURL: window.location.origin,", 1)
    config.write_text(text)
    print('API URL configured to use the page origin.')
elif new in text:
    print('API URL is already configured.')
else:
    raise SystemExit('Unknown config structure; configuration left unchanged.')
