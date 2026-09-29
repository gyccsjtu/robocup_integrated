# Canonical head-on / crossing / dynamic-block scenario matrix.
#
# Line format:  <label>|<launcher>|<args>
# Launchers live under robocup_ws/ and default to seed 42 (the only seed the
# world-gen / endpoint pipeline fully supports; see project memory).
#
# Runs on the VM.  run_dynamic_block.sh exists only there.
headon#1|scripts/vm/run_headon.sh|42 headon 0.30
headon#2|scripts/vm/run_headon.sh|42 headon 0.30
headon#3|scripts/vm/run_headon.sh|42 headon 0.30
cross_north|scripts/vm/run_headon.sh|42 cross_north 0.30
cross_south|scripts/vm/run_headon.sh|42 cross_south 0.30
dynamic_block|scripts/vm/run_dynamic_block.sh|42
