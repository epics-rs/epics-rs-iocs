#!/bin/bash
# Generate the GenICam feature database for a GigE camera model, once per
# model: the XML goes to xml/ and the database to db/, both to be committed.
# Load the database from the camera's own st.cmd, after st_base.cmd.
#
#   scripts/add_camera.sh <camera-ip>
#
# The camera only has to be reachable: reading its memory needs no control
# access, so an IOC may hold it and even be acquiring.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
top=$(dirname "$here")
[ $# -eq 1 ] || { sed -n '2,9p' "$0"; exit 1; }

xml=$(python3 "$here/genicam_xml.py" "$1" "$top/xml")
name=$(basename "$xml" .xml)
db="$top/db/$name.template"
echo "XML: $xml"

# ai/ao records, as ADGenICam's addCamera.sh does by default: asyn-rs 0.30
# builds no Int64Write from an int64out's Int64 VAL, so an int64out never
# reaches the driver. GenICam integers are limited to 52 bits this way.
python3 "$here/makeDb.py" "$xml" "$db"
python3 "$here/exclude_features.py" "$db" "$here/excluded_features.txt"
echo "DB:  $db"
echo "In st.cmd: dbLoadRecords(\"\$(ADPYLON)/db/$name.template\", \"P=\$(PREFIX),R=\$(CAM),PORT=\$(PORT)\")"
