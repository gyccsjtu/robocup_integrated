#!/bin/bash
# 按实例 ID 参数化 iris SDF 的 mavlink 端口，结果打到 stdout。
#
# 用法：gen_iris_sdf.sh <基础 sdf> <ID>
#   Gazebo mavlink_tcp_port = 4560 + ID
#   Gazebo mavlink_udp_port = 14560 + ID
#
# roslaunch 的 <param command="..." /> 会捕获 stdout 作为参数值，
# 所以本脚本除了 SDF 本身不能输出任何别的东西（日志一律走 stderr）。
set -e

SDF="$1"
ID="${2:-0}"

if [ -z "$SDF" ] || [ ! -f "$SDF" ]; then
    echo "gen_iris_sdf.sh: 找不到 SDF: $SDF" >&2
    exit 1
fi

TCP=$((4560 + ID))
UDP=$((14560 + ID))

python3 - "$SDF" "$TCP" "$UDP" <<'PY'
import re, sys

path, tcp, udp = sys.argv[1], sys.argv[2], sys.argv[3]
src = open(path, encoding="utf-8").read()

src, n1 = re.subn(r"(<mavlink_tcp_port>)\s*\d+\s*(</mavlink_tcp_port>)",
                  r"\g<1>" + tcp + r"\g<2>", src)
src, n2 = re.subn(r"(<mavlink_udp_port>)\s*\d+\s*(</mavlink_udp_port>)",
                  r"\g<1>" + udp + r"\g<2>", src)

if n1 == 0 or n2 == 0:
    sys.stderr.write("gen_iris_sdf.sh: 未在 %s 中找到 mavlink 端口字段（tcp=%d udp=%d）\n"
                     % (path, n1, n2))
    sys.exit(1)

sys.stdout.write(src)
PY

echo "gen_iris_sdf.sh: ID=$ID tcp=$TCP udp=$UDP" >&2
