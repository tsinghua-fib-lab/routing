#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname ${SCRIPT_DIR})"

DATA=${PROJECT_DIR}/data
echo "$0: use ${DATA} as data folder"

run_mongod.sh -d ${DATA}
run_etcd.sh -d ${DATA}

while sleep 1000
do
  :
done
