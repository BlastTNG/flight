#! /bin/bash

sudo rsync -avz --delete blast_config/linklists/ /data/etc/linklists/
rsync -avz --delete blast_config/linklists/ fc1:/data/etc/linklists/
rsync -avz --delete blast_config/linklists/ fc2:/data/etc/linklists/
#rsync -avz --delete blast_config/linklists fc1user@fc1mustang:/data/etc/
