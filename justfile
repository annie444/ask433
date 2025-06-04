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
  thumbv8m.main-none-eabi thumbv8m.main-none-eabihf \
  armv8r-none-eabihf riscv32em-unknown-none-elf \
  riscv32emc-unknown-none-elf riscv32gc-unknown-linux-gnu \
  riscv32gc-unknown-linux-musl riscv32ima-unknown-none-elf \
  riscv32imac-esp-espidf riscv32imafc-esp-espidf \
  riscv32imc-esp-espidf thumbv4t-none-eabi thumbv5te-none-eabi \
  xtensa-esp32-espidf xtensa-esp32-none-elf \
  xtensa-esp32s2-espidf xtensa-esp32s2-none-elf \
  xtensa-esp32s3-espidf xtensa-esp32s3-none-elf"
no_std := "armv8r-none-eabihf"
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
  set -euo pipefail
  for target in {{ trim(targets) }}; do
    if [[ "{{ no_std }}" == *"$target"*  ]]; then
      continue
    fi
    echo -e "{{ bold_cyan }}Installing toolchain for target: $target{{ reset }}"
    rustup toolchain install --target "$target" --profile minimal stable
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
