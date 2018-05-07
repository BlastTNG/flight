#! /bin/bash

sudo rsync -rtvu --delete blast_config/linklists /data/etc/
rsync -rtvu --delete blast_config/linklists fc1:/data/etc/
rsync -rtvu --delete blast_config/linklists fc2:/data/etc/
#rsync -rtvu --delete blast_config/linklists fc1user@fc1mustang:/data/etc/

