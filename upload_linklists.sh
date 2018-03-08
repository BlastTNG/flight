#! /bin/bash

sudo cp -r blast_config/linklists/*.ll /data/etc/linklists/
scp -r blast_config/linklists/*.ll fc1:/data/etc/linklists/
scp -r blast_config/linklists/*.ll fc2:/data/etc/linklists/
