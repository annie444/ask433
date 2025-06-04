targets := "aarch64-apple-darwin x86_64-unknown-linux-gnu x86_64-unknown-linux-musl \
  x86_64-unknown-freebsd x86_64-unknown-netbsd x86_64-apple-darwin \
  x86_64-pc-windows-gnu x86_64-unknown-linux-gnu x86_64-pc-windows-msvc \
  aarch64-pc-windows-msvc aarch64-unknown-linux-musl arm-unknown-linux-gnueabi \
  arm-unknown-linux-gnueabihf armv7-unknown-linux-gnueabihf \
  riscv64gc-unknown-linux-gnu riscv64gc-unknown-linux-musl \
  arm-unknown-linux-musleabi arm-unknown-linux-musleabihf \
  armv7-unknown-linux-gnueabi armv7-unknown-linux-musleabi \
  armv7-unknown-linux-musleabihf aarch64-unknown-none \
  aarch64-unknown-none-softfloat armebv7r-none-eabi \
  armebv7r-none-eabihf armv7a-none-eabi \
  armv7r-none-eabi armv7r-none-eabihf \
  riscv32i-unknown-none-elf riscv32im-unknown-none-elf \
  riscv32imac-unknown-none-elf riscv32imafc-unknown-none-elf \
  riscv32imc-unknown-none-elf riscv64gc-unknown-none-elf \
  riscv64imac-unknown-none-elf thumbv6m-none-eabi \
  thumbv7em-none-eabi thumbv7em-none-eabihf \
  thumbv7m-none-eabi thumbv8m.base-none-eabi \
  thumbv8m.main-none-eabi thumbv8m.main-none-eabihf"
bold_cyan := '\033[1;36m'
reset := '\033[0m'


default:
  @just --list

build-all:
  #!/usr/bin/env bash
  set -euo pipefail
  for target in {{ trim(targets) }}; do
    echo -e "{{ bold_cyan }}Building for target: $target{{ reset }}"
    cargo +stable build --target "$target"
  done

install-targets:
  #!/usr/bin/env bash
  set -uo pipefail
  last_target=$(/bin/cat .last-target 2>/dev/null || echo "")
  prev_target=""
  reached_last="false"
  for target in {{ trim(targets) }}; do
    if [[ ! -z "$last_target" ]] && [[ "$target" == *"$last_target"* ]] && [[ "$reached_last" == "false" ]]; then
      reached_last="true"
    elif [[ -z "$last_target" ]] || [ "$reached_last" = "true" ]; then
      echo -e "{{ bold_cyan }}Installing toolchain for target: $target{{ reset }}"
      if ! rustup toolchain install --target "$target" --profile minimal --component rust-src stable; then
        echo "Failed to install toolchain for target: $target"
        if [[ -n "$prev_target" ]]; then
          echo "$prev_target" > .last-target
        fi
        exit 1
      else
        prev_target="$target"
      fi
    fi
  done

check-features *args:
  #!/usr/bin/env python3
  import subprocess
  import itertools
  args = filter(lambda x: x != "", "{{ args }}".strip())
  tq_out = subprocess.Popen(["tq", ".features", "--file=Cargo.toml", "--output=json"], stdout=subprocess.PIPE)
  jq_out = subprocess.Popen(["jq", "--raw-output", "--monochrome-output", ". | keys | .[]", "-"], stdin=tq_out.stdout, stdout=subprocess.PIPE)
  tq_out.stdout.close()
  features, errors = jq_out.communicate()
  features = list(filter(lambda x : x != '', features.decode().split("\n")))
  print(features)
  for i in range(len(features) + 1):
    for subset in itertools.combinations(features, i):
      feats = ",".join(list(subset))
      print()
      print("{{ bold_cyan }}" + feats + "{{ reset }}")
      print()
      if len(args) > 0:
        check = subprocess.Popen(["cargo", "check", "-F", feats, args], shell=False)
      else:
        check = subprocess.Popen(["cargo", "check", "-F", feats], shell=False)
      check.communicate()
