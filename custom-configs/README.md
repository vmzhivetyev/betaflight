# Custom Configs

Store custom board configurations here without forking the config repository.

## Quick Start

**Override an existing config:**
```bash
mkdir -p custom-configs/MYBOARD
cp src/config/configs/MYBOARD/config.h custom-configs/MYBOARD/
# Edit custom-configs/MYBOARD/config.h
make CONFIG=MYBOARD
```

**Create a new config:**
```bash
mkdir -p custom-configs/MYBOARD
# Create config.h (and optionally config.c)
make CONFIG=MYBOARD
```

## How It Works

- Custom configs here override submodule configs with the same name
- If no custom config exists, falls back to `src/config/configs/`
- Committed to your repo by default (edit `.gitignore` to change)

## Advanced

Override the directory location:
```bash
make CONFIG=MYBOARD CUSTOM_CONFIGS_DIR=/path/to/my/configs
```
